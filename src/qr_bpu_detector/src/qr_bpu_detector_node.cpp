#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/hb_sys.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace {

constexpr std::array<float, 18> kDefaultAnchors = {
    10.0F, 13.0F, 16.0F, 30.0F, 33.0F, 23.0F,
    30.0F, 61.0F, 62.0F, 45.0F, 59.0F, 119.0F,
    116.0F, 90.0F, 156.0F, 198.0F, 373.0F, 326.0F};

constexpr int kYoloV5AnchorCount = 3;
constexpr int kYolo11DflBins = 16;
constexpr int kYolo11BBoxChannels = 4 * kYolo11DflBins;

struct ScaleMeta {
  int output_index = -1;
  int height = 0;
  int width = 0;
  int channels = 0;
  int stride = 0;
};

struct Yolo11Branch {
  int bbox_output_index = -1;
  int cls_output_index = -1;
  int height = 0;
  int width = 0;
  int stride = 0;
};

struct Detection {
  int class_id = 0;
  float score = 0.0F;
  cv::Rect2f box;
};

inline float Sigmoid(float value) {
  return 1.0F / (1.0F + std::exp(-value));
}

inline float ClampFloat(float value, float min_value, float max_value) {
  return std::min(std::max(value, min_value), max_value);
}

inline int ClampInt(int value, int min_value, int max_value) {
  return std::min(std::max(value, min_value), max_value);
}

inline float ConfidenceToLogit(float confidence) {
  const float clamped = ClampFloat(confidence, 1.0e-6F, 1.0F - 1.0e-6F);
  return -std::log(1.0F / clamped - 1.0F);
}

float TensorAt(const float *data, const hbDNNTensorProperties &properties,
               int h, int w, int c) {
  const auto &shape = properties.validShape;
  if (properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    const int height = shape.dimensionSize[2];
    const int width = shape.dimensionSize[3];
    return data[c * height * width + h * width + w];
  }

  const int width = shape.dimensionSize[2];
  const int channels = shape.dimensionSize[3];
  return data[(h * width + w) * channels + c];
}

bool GetHwc(const hbDNNTensorProperties &properties,
            int *height, int *width, int *channels) {
  if (properties.validShape.numDimensions != 4) {
    return false;
  }

  if (properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    *channels = properties.validShape.dimensionSize[1];
    *height = properties.validShape.dimensionSize[2];
    *width = properties.validShape.dimensionSize[3];
    return true;
  }

  *height = properties.validShape.dimensionSize[1];
  *width = properties.validShape.dimensionSize[2];
  *channels = properties.validShape.dimensionSize[3];
  return true;
}

bool IsSingleClassYolo11HeadForBBox(const ScaleMeta &cls_meta,
                                    const ScaleMeta &bbox_meta) {
  return (cls_meta.height == bbox_meta.height &&
          cls_meta.width == 1 &&
          cls_meta.channels == bbox_meta.width) ||
         (cls_meta.height == 1 &&
          cls_meta.width == bbox_meta.width &&
          cls_meta.channels == bbox_meta.height);
}

void Softmax16(const float *src, float *dst) {
  float max_value = src[0];
  for (int i = 1; i < kYolo11DflBins; ++i) {
    max_value = std::max(max_value, src[i]);
  }

  float sum = 0.0F;
  for (int i = 0; i < kYolo11DflBins; ++i) {
    dst[i] = std::exp(src[i] - max_value);
    sum += dst[i];
  }

  if (sum <= 0.0F) {
    return;
  }

  for (int i = 0; i < kYolo11DflBins; ++i) {
    dst[i] /= sum;
  }
}

}  // namespace

class QrBpuDetectorNode : public rclcpp::Node {
 public:
  explicit QrBpuDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~QrBpuDetectorNode() override;

 private:
  bool InitModel();
  bool AllocateTensors();
  bool CheckDnn(int ret, const char *action) const;
  bool PrepareOutputMeta();
  cv::Mat DecodeCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  cv::Mat DecodeRawImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void OnCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void OnRawImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void ProcessFrame(const cv::Mat &image, const std_msgs::msg::Header &header);
  std::vector<Detection> RunInference(const cv::Mat &image, double *infer_ms);
  cv::Rect BuildCenterRetryRect(const cv::Size &image_size) const;
  cv::Mat Letterbox(const cv::Mat &image, float *scale, int *x_shift, int *y_shift) const;
  bool BgrToNv12(const cv::Mat &bgr, std::vector<uint8_t> *nv12) const;
  std::vector<Detection> ParseYoloV5(const std::vector<hbDNNTensor> &outputs) const;
  std::vector<Detection> ParseYolo11(const std::vector<hbDNNTensor> &outputs) const;
  float Yolo11ClassAt(const hbDNNTensor &tensor, int h, int w, int class_id) const;
  std::vector<Detection> ApplyNms(const std::vector<Detection> &detections) const;
  void PublishDetections(const std::vector<Detection> &detections,
                         const std_msgs::msg::Header &header) const;
  void PublishDebugImage(const cv::Mat &image,
                         const std::vector<Detection> &detections,
                         const std_msgs::msg::Header &header) const;
  void UpdateFpsLog(double infer_ms, size_t detection_count);
  std::string ClassName(int class_id) const;

  hbPackedDNNHandle_t packed_dnn_handle_ = nullptr;
  hbDNNHandle_t dnn_handle_ = nullptr;
  hbDNNTensor input_tensor_{};
  std::vector<hbDNNTensor> output_tensors_;
  bool input_allocated_ = false;
  bool outputs_allocated_ = false;

  int model_input_height_ = 0;
  int model_input_width_ = 0;
  int output_count_ = 0;
  int class_count_ = 1;
  std::vector<ScaleMeta> yolo5_outputs_;
  std::vector<Yolo11Branch> yolo11_branches_;

  std::string model_file_;
  std::string sub_img_topic_;
  std::string image_msg_type_;
  std::string detection_topic_;
  std::string debug_image_topic_;
  std::vector<std::string> class_names_;
  double score_threshold_ = 0.15;
  double nms_threshold_ = 0.50;
  int nms_top_k_ = 100;
  bool publish_debug_image_ = true;
  bool log_fps_ = true;
  double fps_log_interval_sec_ = 1.0;
  bool enable_center_retry_ = true;
  int center_retry_interval_frames_ = 5;
  double center_crop_ratio_ = 0.70;
  int consecutive_empty_frames_ = 0;
  size_t fps_frame_count_ = 0;
  double fps_infer_ms_sum_ = 0.0;
  std::chrono::steady_clock::time_point fps_window_start_;
  std::vector<uint8_t> input_nv12_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_image_pub_;
};

QrBpuDetectorNode::QrBpuDetectorNode(const rclcpp::NodeOptions &options)
    : Node("qr_bpu_detector", options) {
  declare_parameter<std::string>("model_file", "");
  declare_parameter<std::string>("sub_img_topic", "/image");
  declare_parameter<std::string>("image_msg_type", "compressed");
  declare_parameter<std::string>("detection_topic", "/qr_detection");
  declare_parameter<std::string>("debug_image_topic", "/qr_detection/image/compressed");
  declare_parameter<std::vector<std::string>>("class_names", {"qr_code"});
  declare_parameter<double>("score_threshold", score_threshold_);
  declare_parameter<double>("nms_threshold", nms_threshold_);
  declare_parameter<int>("nms_top_k", nms_top_k_);
  declare_parameter<bool>("publish_debug_image", publish_debug_image_);
  declare_parameter<bool>("log_fps", log_fps_);
  declare_parameter<double>("fps_log_interval_sec", fps_log_interval_sec_);
  declare_parameter<bool>("enable_center_retry", enable_center_retry_);
  declare_parameter<int>("center_retry_interval_frames",
                         center_retry_interval_frames_);
  declare_parameter<double>("center_crop_ratio", center_crop_ratio_);

  get_parameter("model_file", model_file_);
  get_parameter("sub_img_topic", sub_img_topic_);
  get_parameter("image_msg_type", image_msg_type_);
  get_parameter("detection_topic", detection_topic_);
  get_parameter("debug_image_topic", debug_image_topic_);
  get_parameter("class_names", class_names_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("nms_threshold", nms_threshold_);
  get_parameter("nms_top_k", nms_top_k_);
  get_parameter("publish_debug_image", publish_debug_image_);
  get_parameter("log_fps", log_fps_);
  get_parameter("fps_log_interval_sec", fps_log_interval_sec_);
  get_parameter("enable_center_retry", enable_center_retry_);
  get_parameter("center_retry_interval_frames",
                center_retry_interval_frames_);
  get_parameter("center_crop_ratio", center_crop_ratio_);

  if (class_names_.empty()) {
    class_names_.push_back("qr_code");
  }
  fps_log_interval_sec_ = std::max(0.2, fps_log_interval_sec_);
  center_retry_interval_frames_ = std::max(1, center_retry_interval_frames_);
  center_crop_ratio_ = std::min(0.95, std::max(0.35, center_crop_ratio_));
  fps_window_start_ = std::chrono::steady_clock::now();

  if (!InitModel() || !AllocateTensors() || !PrepareOutputMeta()) {
    RCLCPP_FATAL(get_logger(), "QR BPU detector initialization failed.");
    throw std::runtime_error("QR BPU detector initialization failed");
  }

  detection_pub_ = create_publisher<ai_msgs::msg::PerceptionTargets>(detection_topic_, 10);
  if (publish_debug_image_) {
    debug_image_pub_ =
        create_publisher<sensor_msgs::msg::CompressedImage>(debug_image_topic_, 1);
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  if (image_msg_type_ == "raw") {
    raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        sub_img_topic_, qos,
        std::bind(&QrBpuDetectorNode::OnRawImage, this, std::placeholders::_1));
  } else {
    compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        sub_img_topic_, qos,
        std::bind(&QrBpuDetectorNode::OnCompressedImage, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
      get_logger(),
      "QR BPU detector started. model=%s, input=%dx%d, topic=%s, "
      "msg_type=%s, center_retry=%s/%d frames, center_ratio=%.2f",
      model_file_.c_str(), model_input_width_, model_input_height_,
      sub_img_topic_.c_str(), image_msg_type_.c_str(),
      enable_center_retry_ ? "on" : "off", center_retry_interval_frames_,
      center_crop_ratio_);
}

QrBpuDetectorNode::~QrBpuDetectorNode() {
  if (outputs_allocated_) {
    for (auto &tensor : output_tensors_) {
      hbSysFreeMem(&(tensor.sysMem[0]));
    }
  }
  if (input_allocated_) {
    hbSysFreeMem(&(input_tensor_.sysMem[0]));
  }
  if (packed_dnn_handle_ != nullptr) {
    hbDNNRelease(packed_dnn_handle_);
  }
}

bool QrBpuDetectorNode::CheckDnn(int ret, const char *action) const {
  if (ret != 0) {
    RCLCPP_ERROR(get_logger(), "%s failed, ret=%d", action, ret);
    return false;
  }
  return true;
}

bool QrBpuDetectorNode::InitModel() {
  if (model_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter model_file is empty.");
    return false;
  }

  const auto begin = std::chrono::steady_clock::now();
  const char *model_path = model_file_.c_str();
  if (!CheckDnn(hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_path, 1),
                "hbDNNInitializeFromFiles")) {
    return false;
  }

  const char **model_name_list = nullptr;
  int model_count = 0;
  if (!CheckDnn(hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_),
                "hbDNNGetModelNameList")) {
    return false;
  }
  if (model_count <= 0 || model_name_list == nullptr) {
    RCLCPP_ERROR(get_logger(), "No model found in %s", model_file_.c_str());
    return false;
  }
  if (model_count > 1) {
    RCLCPP_WARN(get_logger(), "Model file has %d models, only the first one is used.",
                model_count);
  }

  if (!CheckDnn(hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_list[0]),
                "hbDNNGetModelHandle")) {
    return false;
  }

  int input_count = 0;
  if (!CheckDnn(hbDNNGetInputCount(&input_count, dnn_handle_), "hbDNNGetInputCount")) {
    return false;
  }
  if (input_count != 1) {
    RCLCPP_ERROR(get_logger(), "Expected one model input, got %d", input_count);
    return false;
  }

  hbDNNTensorProperties input_properties;
  if (!CheckDnn(hbDNNGetInputTensorProperties(&input_properties, dnn_handle_, 0),
                "hbDNNGetInputTensorProperties")) {
    return false;
  }

  int channels = 0;
  if (!GetHwc(input_properties, &model_input_height_, &model_input_width_, &channels)) {
    RCLCPP_ERROR(get_logger(), "Input shape must be 4D.");
    return false;
  }
  if (model_input_height_ <= 0 || model_input_width_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Invalid model input size: %dx%d",
                 model_input_width_, model_input_height_);
    return false;
  }

  if (!CheckDnn(hbDNNGetOutputCount(&output_count_, dnn_handle_), "hbDNNGetOutputCount")) {
    return false;
  }
  if (output_count_ != 3 && output_count_ != 6) {
    RCLCPP_ERROR(get_logger(), "Unsupported output count %d. Expected YOLOv5(3) or YOLO11(6).",
                 output_count_);
    return false;
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - begin).count() / 1000.0;
  RCLCPP_INFO(get_logger(), "Loaded model %s in %.2f ms. output_count=%d",
              model_name_list[0], elapsed_ms, output_count_);
  return true;
}

bool QrBpuDetectorNode::AllocateTensors() {
  hbDNNTensorProperties input_properties;
  if (!CheckDnn(hbDNNGetInputTensorProperties(&input_properties, dnn_handle_, 0),
                "hbDNNGetInputTensorProperties")) {
    return false;
  }
  input_tensor_.properties = input_properties;
  const int input_size = model_input_height_ * model_input_width_ * 3 / 2;
  const int input_alloc_size = std::max(input_size, input_properties.alignedByteSize);
  if (!CheckDnn(hbSysAllocCachedMem(&(input_tensor_.sysMem[0]), input_alloc_size),
                "hbSysAllocCachedMem(input)")) {
    return false;
  }
  input_allocated_ = true;

  output_tensors_.resize(output_count_);
  for (int i = 0; i < output_count_; ++i) {
    hbDNNTensorProperties output_properties;
    if (!CheckDnn(hbDNNGetOutputTensorProperties(&output_properties, dnn_handle_, i),
                  "hbDNNGetOutputTensorProperties")) {
      return false;
    }
    output_tensors_[i].properties = output_properties;
    if (!CheckDnn(hbSysAllocCachedMem(&(output_tensors_[i].sysMem[0]),
                                      output_properties.alignedByteSize),
                  "hbSysAllocCachedMem(output)")) {
      return false;
    }
  }
  outputs_allocated_ = true;
  return true;
}

bool QrBpuDetectorNode::PrepareOutputMeta() {
  std::vector<ScaleMeta> metas;
  metas.reserve(output_count_);
  for (int i = 0; i < output_count_; ++i) {
    int height = 0;
    int width = 0;
    int channels = 0;
    if (!GetHwc(output_tensors_[i].properties, &height, &width, &channels)) {
      RCLCPP_ERROR(get_logger(), "Output %d shape must be 4D.", i);
      return false;
    }
    if (output_tensors_[i].properties.quantiType != NONE) {
      RCLCPP_ERROR(get_logger(), "Output %d is not dequantized float output.", i);
      return false;
    }
    const int stride = height > 0 ? model_input_height_ / height : 0;
    metas.push_back({i, height, width, channels, stride});
    RCLCPP_INFO(get_logger(), "output[%d] shape HWC=(%d,%d,%d), stride=%d",
                i, height, width, channels, stride);
  }

  if (output_count_ == 3) {
    yolo5_outputs_ = metas;
    std::sort(yolo5_outputs_.begin(), yolo5_outputs_.end(),
              [](const ScaleMeta &lhs, const ScaleMeta &rhs) {
                return lhs.stride < rhs.stride;
              });
    const int channels = yolo5_outputs_.front().channels;
    if (channels % kYoloV5AnchorCount != 0) {
      RCLCPP_ERROR(get_logger(), "YOLOv5 output channels %d is not divisible by %d.",
                   channels, kYoloV5AnchorCount);
      return false;
    }
    class_count_ = channels / kYoloV5AnchorCount - 5;
    if (class_count_ <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid YOLOv5 class count inferred from channels=%d.",
                   channels);
      return false;
    }
  } else {
    for (const auto &bbox_meta : metas) {
      if (bbox_meta.channels != kYolo11BBoxChannels) {
        continue;
      }
      auto cls_it = std::find_if(
          metas.begin(), metas.end(), [&bbox_meta](const ScaleMeta &meta) {
            return meta.output_index != bbox_meta.output_index &&
                   meta.channels != kYolo11BBoxChannels &&
                   ((meta.height == bbox_meta.height &&
                     meta.width == bbox_meta.width) ||
                    IsSingleClassYolo11HeadForBBox(meta, bbox_meta));
          });
      if (cls_it == metas.end()) {
        RCLCPP_ERROR(get_logger(), "No class head found for YOLO11 bbox output %d.",
                     bbox_meta.output_index);
        return false;
      }
      yolo11_branches_.push_back({bbox_meta.output_index, cls_it->output_index,
                                  bbox_meta.height, bbox_meta.width, bbox_meta.stride});
      RCLCPP_INFO(get_logger(),
                  "YOLO11 branch stride=%d: bbox output[%d], cls output[%d]",
                  bbox_meta.stride, bbox_meta.output_index, cls_it->output_index);
    }

    if (yolo11_branches_.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Expected 3 YOLO11 branches, got %zu.",
                   yolo11_branches_.size());
      return false;
    }
    std::sort(yolo11_branches_.begin(), yolo11_branches_.end(),
              [](const Yolo11Branch &lhs, const Yolo11Branch &rhs) {
                return lhs.stride < rhs.stride;
              });
    int cls_height = 0;
    int cls_width = 0;
    int cls_channels = 0;
    GetHwc(output_tensors_[yolo11_branches_.front().cls_output_index].properties,
           &cls_height, &cls_width, &cls_channels);
    if ((cls_width == 1 && cls_channels == yolo11_branches_.front().width) ||
        (cls_height == 1 && cls_channels == yolo11_branches_.front().height)) {
      class_count_ = 1;
    } else {
      class_count_ = cls_channels;
    }
  }

  while (static_cast<int>(class_names_.size()) < class_count_) {
    class_names_.push_back("class_" + std::to_string(class_names_.size()));
  }

  RCLCPP_INFO(get_logger(), "Using %d class(es), parser=%s",
              class_count_, output_count_ == 3 ? "YOLOv5" : "YOLO11");
  return true;
}

cv::Mat QrBpuDetectorNode::DecodeCompressedImage(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  if (!msg || msg->data.empty()) {
    return {};
  }
  cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
                  const_cast<uint8_t *>(msg->data.data()));
  return cv::imdecode(encoded, cv::IMREAD_COLOR);
}

cv::Mat QrBpuDetectorNode::DecodeRawImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (!msg) {
    return {};
  }

  const int height = static_cast<int>(msg->height);
  const int width = static_cast<int>(msg->width);
  if (height <= 0 || width <= 0 || msg->data.empty()) {
    return {};
  }

  if (msg->encoding == "bgr8") {
    cv::Mat view(height, width, CV_8UC3,
                 const_cast<uint8_t *>(msg->data.data()), msg->step);
    return view.clone();
  }

  if (msg->encoding == "rgb8") {
    cv::Mat rgb(height, width, CV_8UC3,
                const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }

  if (msg->encoding == "mono8") {
    cv::Mat mono(height, width, CV_8UC1,
                 const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  if (msg->encoding == "nv12" || msg->encoding == "NV12") {
    const size_t expected_size = static_cast<size_t>(height) * width * 3 / 2;
    if (msg->data.size() < expected_size) {
      RCLCPP_ERROR(get_logger(), "NV12 image data is too small: got %zu, need %zu",
                   msg->data.size(), expected_size);
      return {};
    }
    cv::Mat nv12(height * 3 / 2, width, CV_8UC1,
                 const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    return bgr;
  }

  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Unsupported raw image encoding: %s", msg->encoding.c_str());
  return {};
}

void QrBpuDetectorNode::OnCompressedImage(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  const cv::Mat image = DecodeCompressedImage(msg);
  if (image.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Received an empty compressed image.");
    return;
  }
  ProcessFrame(image, msg->header);
}

void QrBpuDetectorNode::OnRawImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  const cv::Mat image = DecodeRawImage(msg);
  if (image.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Received an empty raw image.");
    return;
  }
  ProcessFrame(image, msg->header);
}

cv::Mat QrBpuDetectorNode::Letterbox(
    const cv::Mat &image, float *scale, int *x_shift, int *y_shift) const {
  *scale = std::min(static_cast<float>(model_input_width_) / image.cols,
                    static_cast<float>(model_input_height_) / image.rows);
  const int new_width = static_cast<int>(std::round(image.cols * (*scale)));
  const int new_height = static_cast<int>(std::round(image.rows * (*scale)));
  *x_shift = (model_input_width_ - new_width) / 2;
  *y_shift = (model_input_height_ - new_height) / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_width, new_height), 0.0, 0.0,
             cv::INTER_LINEAR);

  cv::Mat padded(model_input_height_, model_input_width_, CV_8UC3,
                 cv::Scalar(127, 127, 127));
  resized.copyTo(padded(cv::Rect(*x_shift, *y_shift, new_width, new_height)));
  return padded;
}

bool QrBpuDetectorNode::BgrToNv12(const cv::Mat &bgr, std::vector<uint8_t> *nv12) const {
  cv::Mat yuv_i420;
  cv::cvtColor(bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);
  const int y_size = model_input_height_ * model_input_width_;
  const int uv_width = model_input_width_ / 2;
  const int uv_height = model_input_height_ / 2;
  nv12->resize(y_size * 3 / 2);

  const uint8_t *yuv = yuv_i420.ptr<uint8_t>();
  std::memcpy(nv12->data(), yuv, y_size);

  const uint8_t *u_data = yuv + y_size;
  const uint8_t *v_data = u_data + uv_width * uv_height;
  uint8_t *uv_data = nv12->data() + y_size;
  for (int i = 0; i < uv_width * uv_height; ++i) {
    *uv_data++ = *u_data++;
    *uv_data++ = *v_data++;
  }
  return true;
}

std::vector<Detection> QrBpuDetectorNode::RunInference(
    const cv::Mat &image, double *infer_ms) {
  if (infer_ms == nullptr || image.empty()) {
    return {};
  }
  *infer_ms = 0.0;

  float scale = 1.0F;
  int x_shift = 0;
  int y_shift = 0;
  const cv::Mat model_image = Letterbox(image, &scale, &x_shift, &y_shift);

  if (!BgrToNv12(model_image, &input_nv12_buffer_)) {
    RCLCPP_ERROR(get_logger(), "BGR to NV12 failed.");
    return {};
  }

  const int input_size = model_input_height_ * model_input_width_ * 3 / 2;
  std::memcpy(input_tensor_.sysMem[0].virAddr, input_nv12_buffer_.data(),
              input_size);
  hbSysFlushMem(&(input_tensor_.sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);

  hbDNNTensor *output = output_tensors_.data();
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

  const auto begin = std::chrono::steady_clock::now();
  if (!CheckDnn(hbDNNInfer(&task_handle, &output, &input_tensor_, dnn_handle_,
                           &infer_ctrl_param),
                "hbDNNInfer")) {
    return {};
  }
  if (!CheckDnn(hbDNNWaitTaskDone(task_handle, 0), "hbDNNWaitTaskDone")) {
    hbDNNReleaseTask(task_handle);
    return {};
  }
  *infer_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - begin)
                  .count() /
              1000.0;
  hbDNNReleaseTask(task_handle);

  for (auto &tensor : output_tensors_) {
    hbSysFlushMem(&(tensor.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }

  auto detections = output_count_ == 3 ? ParseYoloV5(output_tensors_)
                                       : ParseYolo11(output_tensors_);
  for (auto &detection : detections) {
    const float x1 = (detection.box.x - x_shift) / scale;
    const float y1 = (detection.box.y - y_shift) / scale;
    const float x2 = (detection.box.x + detection.box.width - x_shift) / scale;
    const float y2 = (detection.box.y + detection.box.height - y_shift) / scale;

    const float clipped_x1 = ClampFloat(x1, 0.0F, static_cast<float>(image.cols - 1));
    const float clipped_y1 = ClampFloat(y1, 0.0F, static_cast<float>(image.rows - 1));
    const float clipped_x2 = ClampFloat(x2, 0.0F, static_cast<float>(image.cols));
    const float clipped_y2 = ClampFloat(y2, 0.0F, static_cast<float>(image.rows));
    detection.box = cv::Rect2f(clipped_x1, clipped_y1,
                               std::max(0.0F, clipped_x2 - clipped_x1),
                               std::max(0.0F, clipped_y2 - clipped_y1));
  }
  return ApplyNms(detections);
}

cv::Rect QrBpuDetectorNode::BuildCenterRetryRect(
    const cv::Size &image_size) const {
  const int width = ClampInt(
      static_cast<int>(std::lround(image_size.width * center_crop_ratio_)), 1,
      image_size.width);
  const int height = ClampInt(
      static_cast<int>(std::lround(image_size.height * center_crop_ratio_)), 1,
      image_size.height);
  return cv::Rect(std::max(0, (image_size.width - width) / 2),
                  std::max(0, (image_size.height - height) / 2), width, height);
}

void QrBpuDetectorNode::ProcessFrame(const cv::Mat &image,
                                     const std_msgs::msg::Header &header) {
  double total_infer_ms = 0.0;
  double primary_infer_ms = 0.0;
  auto detections = RunInference(image, &primary_infer_ms);
  total_infer_ms += primary_infer_ms;

  if (detections.empty()) {
    ++consecutive_empty_frames_;
    const bool should_retry =
        enable_center_retry_ &&
        (consecutive_empty_frames_ == 1 ||
         consecutive_empty_frames_ % center_retry_interval_frames_ == 0);
    if (should_retry) {
      const cv::Rect center_rect = BuildCenterRetryRect(image.size());
      double retry_infer_ms = 0.0;
      auto retry_detections =
          RunInference(image(center_rect), &retry_infer_ms);
      total_infer_ms += retry_infer_ms;
      for (auto &detection : retry_detections) {
        detection.box.x += static_cast<float>(center_rect.x);
        detection.box.y += static_cast<float>(center_rect.y);
      }
      detections = std::move(retry_detections);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "QR center retry: crop=%dx%d, infer=%.2f ms, detections=%zu",
          center_rect.width, center_rect.height, retry_infer_ms,
          detections.size());
    }
  }

  if (!detections.empty()) {
    consecutive_empty_frames_ = 0;
  }

  PublishDetections(detections, header);
  if (publish_debug_image_) {
    PublishDebugImage(image, detections, header);
  }
  UpdateFpsLog(total_infer_ms, detections.size());

  RCLCPP_DEBUG(get_logger(), "infer=%.2f ms, detections=%zu",
               total_infer_ms, detections.size());
}

std::vector<Detection> QrBpuDetectorNode::ParseYoloV5(
    const std::vector<hbDNNTensor> &outputs) const {
  std::vector<Detection> detections;
  const float raw_threshold = ConfidenceToLogit(static_cast<float>(score_threshold_));

  for (size_t scale_id = 0; scale_id < yolo5_outputs_.size(); ++scale_id) {
    const auto &meta = yolo5_outputs_[scale_id];
    const auto &tensor = outputs[meta.output_index];
    const auto *data = reinterpret_cast<const float *>(tensor.sysMem[0].virAddr);

    const int anchor_offset = static_cast<int>(scale_id) * kYoloV5AnchorCount * 2;
    for (int h = 0; h < meta.height; ++h) {
      for (int w = 0; w < meta.width; ++w) {
        for (int anchor_id = 0; anchor_id < kYoloV5AnchorCount; ++anchor_id) {
          const int channel_base = anchor_id * (5 + class_count_);
          const float objectness = TensorAt(data, tensor.properties, h, w, channel_base + 4);
          if (objectness < raw_threshold) {
            continue;
          }

          int best_class = 0;
          float best_class_logit = TensorAt(data, tensor.properties, h, w, channel_base + 5);
          for (int class_id = 1; class_id < class_count_; ++class_id) {
            const float class_logit =
                TensorAt(data, tensor.properties, h, w, channel_base + 5 + class_id);
            if (class_logit > best_class_logit) {
              best_class_logit = class_logit;
              best_class = class_id;
            }
          }

          const float score = Sigmoid(objectness) * Sigmoid(best_class_logit);
          if (score < score_threshold_) {
            continue;
          }

          const float anchor_w = kDefaultAnchors[anchor_offset + anchor_id * 2];
          const float anchor_h = kDefaultAnchors[anchor_offset + anchor_id * 2 + 1];
          const float center_x =
              (Sigmoid(TensorAt(data, tensor.properties, h, w, channel_base + 0)) * 2.0F -
               0.5F + w) * meta.stride;
          const float center_y =
              (Sigmoid(TensorAt(data, tensor.properties, h, w, channel_base + 1)) * 2.0F -
               0.5F + h) * meta.stride;
          const float bbox_w =
              std::pow(Sigmoid(TensorAt(data, tensor.properties, h, w, channel_base + 2)) *
                           2.0F,
                       2.0F) *
              anchor_w;
          const float bbox_h =
              std::pow(Sigmoid(TensorAt(data, tensor.properties, h, w, channel_base + 3)) *
                           2.0F,
                       2.0F) *
              anchor_h;

          detections.push_back(
              {best_class, score,
               cv::Rect2f(center_x - bbox_w * 0.5F, center_y - bbox_h * 0.5F,
                          bbox_w, bbox_h)});
        }
      }
    }
  }
  return detections;
}

std::vector<Detection> QrBpuDetectorNode::ParseYolo11(
    const std::vector<hbDNNTensor> &outputs) const {
  std::vector<Detection> detections;
  const float raw_threshold = ConfidenceToLogit(static_cast<float>(score_threshold_));

  for (const auto &branch : yolo11_branches_) {
    const auto &bbox_tensor = outputs[branch.bbox_output_index];
    const auto &cls_tensor = outputs[branch.cls_output_index];
    const auto *bbox_data = reinterpret_cast<const float *>(bbox_tensor.sysMem[0].virAddr);

    for (int h = 0; h < branch.height; ++h) {
      for (int w = 0; w < branch.width; ++w) {
        int best_class = 0;
        float best_logit = Yolo11ClassAt(cls_tensor, h, w, 0);
        for (int class_id = 1; class_id < class_count_; ++class_id) {
          const float class_logit = Yolo11ClassAt(cls_tensor, h, w, class_id);
          if (class_logit > best_logit) {
            best_logit = class_logit;
            best_class = class_id;
          }
        }
        if (best_logit < raw_threshold) {
          continue;
        }

        const float score = Sigmoid(best_logit);
        if (score < score_threshold_) {
          continue;
        }

        float ltrb[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        for (int side = 0; side < 4; ++side) {
          float raw[kYolo11DflBins];
          float prob[kYolo11DflBins];
          for (int bin = 0; bin < kYolo11DflBins; ++bin) {
            raw[bin] = TensorAt(bbox_data, bbox_tensor.properties, h, w,
                                side * kYolo11DflBins + bin);
          }
          Softmax16(raw, prob);
          for (int bin = 0; bin < kYolo11DflBins; ++bin) {
            ltrb[side] += prob[bin] * static_cast<float>(bin);
          }
        }

        const float anchor_x = static_cast<float>(w) + 0.5F;
        const float anchor_y = static_cast<float>(h) + 0.5F;
        const float x1 = (anchor_x - ltrb[0]) * branch.stride;
        const float y1 = (anchor_y - ltrb[1]) * branch.stride;
        const float x2 = (anchor_x + ltrb[2]) * branch.stride;
        const float y2 = (anchor_y + ltrb[3]) * branch.stride;
        if (x2 <= x1 || y2 <= y1) {
          continue;
        }
        detections.push_back({best_class, score, cv::Rect2f(x1, y1, x2 - x1, y2 - y1)});
      }
    }
  }
  return detections;
}

float QrBpuDetectorNode::Yolo11ClassAt(
    const hbDNNTensor &tensor, int h, int w, int class_id) const {
  const auto *data = reinterpret_cast<const float *>(tensor.sysMem[0].virAddr);
  int height = 0;
  int width = 0;
  int channels = 0;
  GetHwc(tensor.properties, &height, &width, &channels);

  if (class_count_ == 1 && width == 1 && channels > 1) {
    return data[h * channels + w];
  }
  if (class_count_ == 1 && height == 1 && channels > 1) {
    return data[w * channels + h];
  }
  return TensorAt(data, tensor.properties, h, w, class_id);
}

std::vector<Detection> QrBpuDetectorNode::ApplyNms(
    const std::vector<Detection> &detections) const {
  std::vector<Detection> kept;
  for (int class_id = 0; class_id < class_count_; ++class_id) {
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<size_t> source_indices;
    for (size_t i = 0; i < detections.size(); ++i) {
      if (detections[i].class_id != class_id || detections[i].box.area() <= 0.0F) {
        continue;
      }
      boxes.emplace_back(static_cast<int>(std::round(detections[i].box.x)),
                         static_cast<int>(std::round(detections[i].box.y)),
                         static_cast<int>(std::round(detections[i].box.width)),
                         static_cast<int>(std::round(detections[i].box.height)));
      scores.push_back(detections[i].score);
      source_indices.push_back(i);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(score_threshold_),
                      static_cast<float>(nms_threshold_), indices, 1.0F, nms_top_k_);
    for (int index : indices) {
      kept.push_back(detections[source_indices[index]]);
    }
  }
  return kept;
}

void QrBpuDetectorNode::PublishDetections(
    const std::vector<Detection> &detections,
    const std_msgs::msg::Header &header) const {
  auto msg = std::make_unique<ai_msgs::msg::PerceptionTargets>();
  msg->header = header;

  for (const auto &detection : detections) {
    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(ClampInt(static_cast<int>(std::round(detection.box.x)), 0,
                                    100000));
    roi.rect.set__y_offset(ClampInt(static_cast<int>(std::round(detection.box.y)), 0,
                                    100000));
    roi.rect.set__width(std::max(0, static_cast<int>(std::round(detection.box.width))));
    roi.rect.set__height(std::max(0, static_cast<int>(std::round(detection.box.height))));
    roi.set__confidence(detection.score);

    ai_msgs::msg::Target target;
    target.set__type(ClassName(detection.class_id));
    target.rois.emplace_back(std::move(roi));
    msg->targets.emplace_back(std::move(target));
  }

  detection_pub_->publish(std::move(msg));
}

void QrBpuDetectorNode::PublishDebugImage(
    const cv::Mat &image, const std::vector<Detection> &detections,
    const std_msgs::msg::Header &header) const {
  if (!debug_image_pub_) {
    return;
  }

  cv::Mat debug = image.clone();
  for (const auto &detection : detections) {
    const cv::Rect rect(static_cast<int>(std::round(detection.box.x)),
                        static_cast<int>(std::round(detection.box.y)),
                        static_cast<int>(std::round(detection.box.width)),
                        static_cast<int>(std::round(detection.box.height)));
    if (rect.width <= 0 || rect.height <= 0) {
      continue;
    }
    cv::rectangle(debug, rect, cv::Scalar(0, 255, 0), 2);
    const std::string label =
        ClassName(detection.class_id) + " " + cv::format("%.2f", detection.score);
    cv::putText(debug, label, cv::Point(rect.x, std::max(0, rect.y - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  }

  std::vector<uint8_t> encoded;
  if (!cv::imencode(".jpg", debug, encoded, {cv::IMWRITE_JPEG_QUALITY, 80})) {
    RCLCPP_ERROR(get_logger(), "Failed to encode debug image.");
    return;
  }

  sensor_msgs::msg::CompressedImage msg;
  msg.header = header;
  msg.format = "jpeg";
  msg.data = std::move(encoded);
  debug_image_pub_->publish(std::move(msg));
}

void QrBpuDetectorNode::UpdateFpsLog(double infer_ms, size_t detection_count) {
  if (!log_fps_) {
    return;
  }

  ++fps_frame_count_;
  fps_infer_ms_sum_ += infer_ms;

  const auto now = std::chrono::steady_clock::now();
  const double elapsed_sec = std::chrono::duration_cast<std::chrono::microseconds>(
      now - fps_window_start_).count() / 1000000.0;
  if (elapsed_sec < fps_log_interval_sec_) {
    return;
  }

  const double fps = fps_frame_count_ / elapsed_sec;
  const double avg_infer_ms = fps_infer_ms_sum_ / fps_frame_count_;
  RCLCPP_INFO(get_logger(), "fps=%.2f, avg_infer=%.2f ms, detections=%zu",
              fps, avg_infer_ms, detection_count);

  fps_frame_count_ = 0;
  fps_infer_ms_sum_ = 0.0;
  fps_window_start_ = now;
}

std::string QrBpuDetectorNode::ClassName(int class_id) const {
  if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
    return class_names_[class_id];
  }
  return "class_" + std::to_string(class_id);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<QrBpuDetectorNode>());
  } catch (const std::exception &err) {
    RCLCPP_FATAL(rclcpp::get_logger("qr_bpu_detector"), "%s", err.what());
  }
  rclcpp::shutdown();
  return 0;
}
