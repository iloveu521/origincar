#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <zbar.h>

#include "ai_msgs/msg/perception_targets.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

enum class DecodeState {
  kWaitingForTarget,
  kDecoding,
  kSucceeded,
};

struct RoiCandidate {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
  float confidence = 0.0F;
};

inline int ClampInt(int value, int min_value, int max_value) {
  return std::min(std::max(value, min_value), max_value);
}

std::string StateName(DecodeState state) {
  switch (state) {
    case DecodeState::kWaitingForTarget:
      return "WAITING_FOR_TARGET";
    case DecodeState::kDecoding:
      return "DECODING";
    case DecodeState::kSucceeded:
      return "SUCCEEDED";
  }
  return "UNKNOWN";
}

std::string TrimAscii(const std::string &text) {
  size_t begin = 0;
  while (begin < text.size() &&
         std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
    ++begin;
  }

  size_t end = text.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
    --end;
  }

  return text.substr(begin, end - begin);
}

template <typename StampT>
bool SameStamp(const StampT &lhs, const StampT &rhs) {
  return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

}  // namespace

class QrRoiDecoderNode : public rclcpp::Node {
 public:
  explicit QrRoiDecoderNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  using SyncPolicyRaw = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, ai_msgs::msg::PerceptionTargets>;
  using SyncPolicyComp = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::CompressedImage, ai_msgs::msg::PerceptionTargets>;

  cv::Mat DecodeCompressedImage(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) const;
  cv::Mat DecodeRawImage(
      const sensor_msgs::msg::Image::ConstSharedPtr msg);

  void OnSyncedCompressed(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr &img_msg,
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &target_msg);
  void OnSyncedRaw(
      const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &target_msg);

  void ProcessFrame(
      const cv::Mat &image,
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);

  bool IsSucceeded() const;
  void SetState(DecodeState state);
  void PublishState(DecodeState state) const;

  cv::Rect BuildCropRect(
      const RoiCandidate &roi, const cv::Size &image_size, int padding) const;
  cv::Rect BuildCenterFallbackRect(const cv::Size &image_size) const;

  bool RegisterFailureAndShouldRunFallback();
  bool RunFallbackScan(const cv::Mat &image, const char *reason);
  bool DecodeValidNumber(const cv::Mat &image, int original_box_size,
                         int *number, std::string *decoded_text);
  bool DecodeQrFromRoi(const cv::Mat &roi, int original_box_size,
                       std::string *decoded_text);
  bool ScanGrayImage(const cv::Mat &gray, std::string *decoded_text);
  bool ParseValidQrNumber(const std::string &decoded_text, int *number) const;
  void PublishResult(int number);

  std::string image_topic_;
  std::string image_msg_type_;
  std::string detection_topic_;
  std::string direction_topic_;
  std::string raw_number_topic_;
  std::string state_topic_;
  std::string target_type_;

  double min_confidence_ = 0.35;
  double dynamic_padding_ratio_ = 0.15;
  double target_qr_pixels_ = 280.0;
  double max_upscale_ = 8.0;
  int fallback_frame_threshold_ = 10;
  int fallback_interval_frames_ = 10;
  double fallback_center_ratio_ = 0.75;
  double fallback_upscale_ = 1.5;
  int fallback_max_pixels_ = 1000000;
  int max_roi_candidates_ = 3;
  int sync_queue_size_ = 10;
  bool scan_full_image_on_roi_failure_ = true;
  bool publish_state_ = true;

  mutable std::mutex state_mutex_;
  DecodeState state_ = DecodeState::kWaitingForTarget;
  std::string last_state_name_;
  int consecutive_failure_count_ = 0;

  zbar::ImageScanner scanner_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      raw_sub_;
  std::shared_ptr<
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>
      comp_sub_;
  std::shared_ptr<
      message_filters::Subscriber<ai_msgs::msg::PerceptionTargets>>
      target_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicyRaw>> sync_raw_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyComp>> sync_comp_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_number_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

QrRoiDecoderNode::QrRoiDecoderNode(const rclcpp::NodeOptions &options)
    : Node("qr_roi_decoder", options) {
  declare_parameter<std::string>("image_topic", "/image");
  declare_parameter<std::string>("image_msg_type", "compressed");
  declare_parameter<std::string>("detection_topic", "/qr_detection");
  declare_parameter<std::string>("direction_topic", "/qr_direction");
  declare_parameter<std::string>("raw_number_topic", "/qr_number");
  declare_parameter<std::string>("state_topic", "/qr_decode_state");
  declare_parameter<std::string>("target_type", "qr_code");
  declare_parameter<double>("min_confidence", min_confidence_);
  declare_parameter<double>("dynamic_padding_ratio", dynamic_padding_ratio_);
  declare_parameter<double>("target_qr_pixels", target_qr_pixels_);
  declare_parameter<double>("max_upscale", max_upscale_);
  declare_parameter<int>("fallback_frame_threshold",
                         fallback_frame_threshold_);
  declare_parameter<int>("fallback_interval_frames",
                         fallback_interval_frames_);
  declare_parameter<double>("fallback_center_ratio", fallback_center_ratio_);
  declare_parameter<double>("fallback_upscale", fallback_upscale_);
  declare_parameter<int>("fallback_max_pixels", fallback_max_pixels_);
  declare_parameter<int>("max_roi_candidates", max_roi_candidates_);
  declare_parameter<int>("sync_queue_size", sync_queue_size_);
  declare_parameter<bool>("scan_full_image_on_roi_failure",
                          scan_full_image_on_roi_failure_);
  declare_parameter<bool>("publish_state", publish_state_);

  get_parameter("image_topic", image_topic_);
  get_parameter("image_msg_type", image_msg_type_);
  get_parameter("detection_topic", detection_topic_);
  get_parameter("direction_topic", direction_topic_);
  get_parameter("raw_number_topic", raw_number_topic_);
  get_parameter("state_topic", state_topic_);
  get_parameter("target_type", target_type_);
  get_parameter("min_confidence", min_confidence_);
  get_parameter("dynamic_padding_ratio", dynamic_padding_ratio_);
  get_parameter("target_qr_pixels", target_qr_pixels_);
  get_parameter("max_upscale", max_upscale_);
  get_parameter("fallback_frame_threshold", fallback_frame_threshold_);
  get_parameter("fallback_interval_frames", fallback_interval_frames_);
  get_parameter("fallback_center_ratio", fallback_center_ratio_);
  get_parameter("fallback_upscale", fallback_upscale_);
  get_parameter("fallback_max_pixels", fallback_max_pixels_);
  get_parameter("max_roi_candidates", max_roi_candidates_);
  get_parameter("sync_queue_size", sync_queue_size_);
  get_parameter("scan_full_image_on_roi_failure",
                scan_full_image_on_roi_failure_);
  get_parameter("publish_state", publish_state_);

  dynamic_padding_ratio_ =
      std::min(1.0, std::max(0.0, dynamic_padding_ratio_));
  target_qr_pixels_ = std::max(64.0, target_qr_pixels_);
  max_upscale_ = std::max(1.0, max_upscale_);
  fallback_frame_threshold_ = std::max(1, fallback_frame_threshold_);
  fallback_interval_frames_ = std::max(1, fallback_interval_frames_);
  fallback_center_ratio_ =
      std::min(1.0, std::max(0.25, fallback_center_ratio_));
  fallback_upscale_ = std::min(3.0, std::max(1.0, fallback_upscale_));
  fallback_max_pixels_ = std::max(250000, fallback_max_pixels_);
  max_roi_candidates_ = std::max(1, max_roi_candidates_);
  sync_queue_size_ = std::max(2, sync_queue_size_);

  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  direction_pub_ =
      create_publisher<std_msgs::msg::String>(direction_topic_, 10);
  raw_number_pub_ =
      create_publisher<std_msgs::msg::String>(raw_number_topic_, 10);
  if (publish_state_) {
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
  }

  const rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
  target_sub_ = std::make_shared<
      message_filters::Subscriber<ai_msgs::msg::PerceptionTargets>>(
      this, detection_topic_, sensor_qos);

  if (image_msg_type_ == "raw") {
    raw_sub_ = std::make_shared<
        message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, image_topic_, sensor_qos);
    sync_raw_ = std::make_shared<
        message_filters::Synchronizer<SyncPolicyRaw>>(
        SyncPolicyRaw(sync_queue_size_), *raw_sub_, *target_sub_);
    sync_raw_->registerCallback(
        std::bind(&QrRoiDecoderNode::OnSyncedRaw, this,
                  std::placeholders::_1, std::placeholders::_2));
  } else {
    comp_sub_ = std::make_shared<
        message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
        this, image_topic_, sensor_qos);
    sync_comp_ = std::make_shared<
        message_filters::Synchronizer<SyncPolicyComp>>(
        SyncPolicyComp(sync_queue_size_), *comp_sub_, *target_sub_);
    sync_comp_->registerCallback(
        std::bind(&QrRoiDecoderNode::OnSyncedCompressed, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  SetState(DecodeState::kWaitingForTarget);
  RCLCPP_INFO(
      get_logger(),
      "QR ROI decoder started: exact_sync=true, image=%s(%s), "
      "detections=%s, padding_ratio=%.2f, target_qr=%.0f px, "
      "max_upscale=%.1fx, fallback=%d/%d frames, center_ratio=%.2f, "
      "fallback_upscale=%.1fx, sync_queue=%d",
      image_topic_.c_str(), image_msg_type_.c_str(),
      detection_topic_.c_str(), dynamic_padding_ratio_, target_qr_pixels_,
      max_upscale_, fallback_frame_threshold_, fallback_interval_frames_,
      fallback_center_ratio_, fallback_upscale_, sync_queue_size_);
}

cv::Mat QrRoiDecoderNode::DecodeCompressedImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) const {
  if (!msg || msg->data.empty()) {
    return {};
  }

  cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
                  const_cast<uint8_t *>(msg->data.data()));
  return cv::imdecode(encoded, cv::IMREAD_COLOR);
}

cv::Mat QrRoiDecoderNode::DecodeRawImage(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!msg || msg->data.empty()) {
    return {};
  }

  const int height = static_cast<int>(msg->height);
  const int width = static_cast<int>(msg->width);
  if (height <= 0 || width <= 0) {
    return {};
  }

  if (msg->encoding == "bgr8") {
    return cv::Mat(height, width, CV_8UC3,
                   const_cast<uint8_t *>(msg->data.data()), msg->step)
        .clone();
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
    const size_t expected_size =
        static_cast<size_t>(height) * static_cast<size_t>(width) * 3U / 2U;
    if (msg->data.size() < expected_size) {
      RCLCPP_ERROR(get_logger(),
                   "NV12 image data is too small: got %zu, need %zu",
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
                        "Unsupported raw image encoding: %s",
                        msg->encoding.c_str());
  return {};
}

void QrRoiDecoderNode::OnSyncedCompressed(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr &img_msg,
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &target_msg) {
  if (IsSucceeded() || !img_msg || !target_msg) {
    return;
  }

  if (!SameStamp(img_msg->header.stamp, target_msg->header.stamp)) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected mismatched compressed image/detection timestamps.");
    return;
  }

  const cv::Mat image = DecodeCompressedImage(img_msg);
  if (!image.empty()) {
    ProcessFrame(image, target_msg);
  }
}

void QrRoiDecoderNode::OnSyncedRaw(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &target_msg) {
  if (IsSucceeded() || !img_msg || !target_msg) {
    return;
  }

  if (!SameStamp(img_msg->header.stamp, target_msg->header.stamp)) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected mismatched raw image/detection timestamps.");
    return;
  }

  const cv::Mat image = DecodeRawImage(img_msg);
  if (!image.empty()) {
    ProcessFrame(image, target_msg);
  }
}

void QrRoiDecoderNode::ProcessFrame(
    const cv::Mat &image,
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  std::vector<RoiCandidate> candidates;
  for (const auto &target : msg->targets) {
    if (!target_type_.empty() && target.type != target_type_) {
      continue;
    }

    for (const auto &roi : target.rois) {
      if (roi.confidence < min_confidence_) {
        continue;
      }

      const int width = static_cast<int>(roi.rect.width);
      const int height = static_cast<int>(roi.rect.height);
      if (width <= 0 || height <= 0) {
        continue;
      }

      candidates.push_back({
          static_cast<int>(roi.rect.x_offset),
          static_cast<int>(roi.rect.y_offset),
          width,
          height,
          roi.confidence,
      });
    }
  }

  if (candidates.empty()) {
    if (RegisterFailureAndShouldRunFallback()) {
      SetState(DecodeState::kDecoding);
      if (RunFallbackScan(image, "no valid detector ROI")) {
        return;
      }
    }
    SetState(DecodeState::kWaitingForTarget);
    return;
  }

  SetState(DecodeState::kDecoding);

  std::sort(candidates.begin(), candidates.end(),
            [](const RoiCandidate &lhs, const RoiCandidate &rhs) {
              return lhs.confidence > rhs.confidence;
            });
  if (static_cast<int>(candidates.size()) > max_roi_candidates_) {
    candidates.resize(max_roi_candidates_);
  }

  for (const auto &roi : candidates) {
    const int padding_box_size = std::max(roi.width, roi.height);
    const int scale_box_size = std::max(1, std::min(roi.width, roi.height));
    const int dynamic_padding = static_cast<int>(
        std::lround(static_cast<double>(padding_box_size) *
                    dynamic_padding_ratio_));
    const cv::Rect crop_rect =
        BuildCropRect(roi, image.size(), dynamic_padding);
    if (crop_rect.width <= 0 || crop_rect.height <= 0) {
      continue;
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "QR ROI: confidence=%.3f, box=%dx%d px, padding=%d px, "
        "crop=%dx%d px",
        roi.confidence, roi.width, roi.height, dynamic_padding,
        crop_rect.width, crop_rect.height);

    int number = 0;
    std::string decoded_text;
    if (!DecodeValidNumber(image(crop_rect), scale_box_size,
                           &number, &decoded_text)) {
      continue;
    }

    consecutive_failure_count_ = 0;
    PublishResult(number);
    SetState(DecodeState::kSucceeded);
    return;
  }

  if (RegisterFailureAndShouldRunFallback()) {
    if (RunFallbackScan(image, "all detector ROIs failed to decode")) {
      return;
    }
  }

  SetState(DecodeState::kWaitingForTarget);
}

bool QrRoiDecoderNode::IsSucceeded() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_ == DecodeState::kSucceeded;
}

void QrRoiDecoderNode::SetState(DecodeState state) {
  bool changed = false;
  std::string state_name;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    changed = state_ != state;
    state_ = state;
    state_name = StateName(state);
    if (!changed && state_name == last_state_name_) {
      return;
    }
    last_state_name_ = state_name;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "QR decode state: %s", state_name.c_str());
  PublishState(state);
}

void QrRoiDecoderNode::PublishState(DecodeState state) const {
  if (!state_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = StateName(state);
  state_pub_->publish(msg);
}

cv::Rect QrRoiDecoderNode::BuildCropRect(
    const RoiCandidate &roi, const cv::Size &image_size, int padding) const {
  const int x1 = ClampInt(roi.x - padding, 0, image_size.width);
  const int y1 = ClampInt(roi.y - padding, 0, image_size.height);
  const int x2 =
      ClampInt(roi.x + roi.width + padding, 0, image_size.width);
  const int y2 =
      ClampInt(roi.y + roi.height + padding, 0, image_size.height);
  return cv::Rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
}

cv::Rect QrRoiDecoderNode::BuildCenterFallbackRect(
    const cv::Size &image_size) const {
  const int width = ClampInt(
      static_cast<int>(std::lround(image_size.width * fallback_center_ratio_)),
      1, image_size.width);
  const int height = ClampInt(
      static_cast<int>(std::lround(image_size.height * fallback_center_ratio_)),
      1, image_size.height);
  const int x = std::max(0, (image_size.width - width) / 2);
  const int y = std::max(0, (image_size.height - height) / 2);
  return cv::Rect(x, y, width, height);
}

bool QrRoiDecoderNode::RegisterFailureAndShouldRunFallback() {
  ++consecutive_failure_count_;
  if (!scan_full_image_on_roi_failure_ ||
      consecutive_failure_count_ < fallback_frame_threshold_) {
    return false;
  }

  return (consecutive_failure_count_ - fallback_frame_threshold_) %
             fallback_interval_frames_ ==
         0;
}

bool QrRoiDecoderNode::RunFallbackScan(
    const cv::Mat &image, const char *reason) {
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Running low-frequency QR fallback after %d consecutive failures: %s",
      consecutive_failure_count_, reason);

  const cv::Rect center_rect = BuildCenterFallbackRect(image.size());
  int number = 0;
  std::string decoded_text;

  if (center_rect.width < image.cols || center_rect.height < image.rows) {
    if (DecodeValidNumber(image(center_rect), 0, &number, &decoded_text)) {
      consecutive_failure_count_ = 0;
      PublishResult(number);
      SetState(DecodeState::kSucceeded);
      return true;
    }
  }

  if (DecodeValidNumber(image, 0, &number, &decoded_text)) {
    consecutive_failure_count_ = 0;
    PublishResult(number);
    SetState(DecodeState::kSucceeded);
    return true;
  }

  return false;
}

bool QrRoiDecoderNode::DecodeValidNumber(
    const cv::Mat &image, int original_box_size,
    int *number, std::string *decoded_text) {
  if (number == nullptr || decoded_text == nullptr) {
    return false;
  }

  decoded_text->clear();
  if (!DecodeQrFromRoi(image, original_box_size, decoded_text)) {
    return false;
  }

  if (!ParseValidQrNumber(*decoded_text, number)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Ignored invalid QR payload: '%s'",
                         decoded_text->c_str());
    return false;
  }

  return true;
}

bool QrRoiDecoderNode::DecodeQrFromRoi(
    const cv::Mat &roi, int original_box_size,
    std::string *decoded_text) {
  if (roi.empty() || decoded_text == nullptr) {
    return false;
  }

  cv::Mat gray;
  if (roi.channels() == 1) {
    gray = roi;
  } else {
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  }

  if (ScanGrayImage(gray, decoded_text)) {
    return true;
  }

  double scale = 1.0;
  if (original_box_size > 0) {
    const double requested_scale =
        target_qr_pixels_ / static_cast<double>(original_box_size);
    scale = std::min(max_upscale_, requested_scale);
  } else {
    const double pixel_count = std::max(1.0, static_cast<double>(gray.total()));
    const double area_limited_scale =
        std::sqrt(static_cast<double>(fallback_max_pixels_) / pixel_count);
    scale = std::min(fallback_upscale_, area_limited_scale);
  }

  cv::Mat work = gray;
  if (scale > 1.05) {
    cv::resize(gray, work, cv::Size(), scale, scale, cv::INTER_CUBIC);
    if (ScanGrayImage(work, decoded_text)) {
      return true;
    }
  }

  cv::Mat blurred;
  cv::Mat sharpened;
  cv::GaussianBlur(work, blurred, cv::Size(0, 0), 1.0);
  cv::addWeighted(work, 1.6, blurred, -0.6, 0.0, sharpened);
  if (ScanGrayImage(sharpened, decoded_text)) {
    return true;
  }

  cv::Mat local_contrast;
  const auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  clahe->apply(work, local_contrast);
  if (ScanGrayImage(local_contrast, decoded_text)) {
    return true;
  }

  cv::Mat equalized;
  cv::equalizeHist(work, equalized);
  if (ScanGrayImage(equalized, decoded_text)) {
    return true;
  }

  cv::Mat otsu;
  cv::threshold(local_contrast, otsu, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU);
  if (ScanGrayImage(otsu, decoded_text)) {
    return true;
  }

  const int min_side = std::min(local_contrast.cols, local_contrast.rows);
  int block_size = std::min(31, min_side);
  if (block_size % 2 == 0) {
    --block_size;
  }
  if (block_size < 3) {
    return false;
  }

  cv::Mat binary;
  cv::adaptiveThreshold(local_contrast, binary, 255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY, block_size, 3);
  return ScanGrayImage(binary, decoded_text);
}

bool QrRoiDecoderNode::ScanGrayImage(
    const cv::Mat &gray, std::string *decoded_text) {
  if (gray.empty() || decoded_text == nullptr) {
    return false;
  }

  cv::Mat continuous = gray.isContinuous() ? gray : gray.clone();
  zbar::Image zbar_image(
      continuous.cols, continuous.rows, "Y800",
      continuous.data, continuous.cols * continuous.rows);

  const int scan_count = scanner_.scan(zbar_image);
  if (scan_count < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "ZBar scan failed.");
    zbar_image.set_data(nullptr, 0);
    return false;
  }

  for (auto symbol = zbar_image.symbol_begin();
       symbol != zbar_image.symbol_end(); ++symbol) {
    *decoded_text = symbol->get_data();
    zbar_image.set_data(nullptr, 0);
    return !decoded_text->empty();
  }

  zbar_image.set_data(nullptr, 0);
  return false;
}

bool QrRoiDecoderNode::ParseValidQrNumber(
    const std::string &decoded_text, int *number) const {
  if (number == nullptr) {
    return false;
  }

  const std::string text = TrimAscii(decoded_text);
  if (text.empty() || text.size() > 4) {
    return false;
  }

  int value = 0;
  for (char ch : text) {
    if (std::isdigit(static_cast<unsigned char>(ch)) == 0) {
      return false;
    }
    value = value * 10 + (ch - '0');
  }

  if (value < 1 || value > 9999) {
    return false;
  }

  *number = value;
  return true;
}

void QrRoiDecoderNode::PublishResult(int number) {
  std_msgs::msg::String number_msg;
  number_msg.data = std::to_string(number);
  raw_number_pub_->publish(number_msg);

  std_msgs::msg::String direction_msg;
  direction_msg.data = (number % 2 == 1) ? "顺时针" : "逆时针";
  direction_pub_->publish(direction_msg);

  RCLCPP_INFO(get_logger(), "QR decoded: number=%d, direction=%s",
              number, direction_msg.data.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<QrRoiDecoderNode>());
  } catch (const std::exception &err) {
    RCLCPP_FATAL(rclcpp::get_logger("qr_roi_decoder"), "%s", err.what());
  }
  rclcpp::shutdown();
  return 0;
}
