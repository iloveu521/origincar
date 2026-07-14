// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "racing_obstacle_detection/parser.h"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#include <memory>
#include <cmath>
#include <algorithm>
#include <numeric>

using hobot::dnn_node::DNNTensor;

namespace hobot {
namespace dnn_node {
namespace racing_obstacle_detection {
// 算法输出解析参数

float score_threshold_ = 0.25;
float nms_threshold_ = 0.65;
int nms_top_k_ = 100;



void ParseTensor(std::shared_ptr<DNNTensor> tensor,
                 int layer,
                 std::vector<YoloV5Result> &results);

void yolo5_nms(std::vector<YoloV5Result> &input,
               float iou_threshold,
               int top_k,
               std::vector<std::shared_ptr<YoloV5Result>> &result,
               bool suppress);

int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width);

/**
 * Finds the greatest element in the range [first, last)
 * @tparam[in] ForwardIterator: iterator type
 * @param[in] first: fist iterator
 * @param[in] last: last iterator
 * @return Iterator to the greatest element in the range [first, last)
 */
template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::max_element(first, last));
}

// softmax实现
inline void softmax(const float* src, float* dst, int len) {
  float max_val = *std::max_element(src, src + len);
  float sum = 0.f;
  for (int i = 0; i < len; ++i) {
    dst[i] = std::exp(src[i] - max_val);
    sum += dst[i];
  }
  for (int i = 0; i < len; ++i) {
    dst[i] /= sum;
  }
}

// YOLOv11 DFL期望权重
static float dfl_weights[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

// anchor生成函数（移除未用参数stride）
static void generate_anchors(int feat_size, std::vector<std::pair<float, float>>& anchors) {
  anchors.clear();
  for (int y = 0; y < feat_size; ++y) {
    for (int x = 0; x < feat_size; ++x) {
      anchors.emplace_back(x + 0.5f, y + 0.5f);
    }
  }
}

// YOLOv11 ParseTensor实现
void ParseTensor(std::shared_ptr<DNNTensor> tensor,
                 int layer,
                 std::vector<YoloV5Result> &results,
                 PTQYolo5Config &yolo5_config) {
  // 该函数已不再使用，且含有无效代码，直接留空实现防止编译错误
}



void yolo5_nms(std::vector<YoloV5Result> &input,
               float iou_threshold,
               int top_k,
               std::vector<std::shared_ptr<YoloV5Result>> &result,
               bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<YoloV5Result>());

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].xmax - input[i].xmin;
    float height = input[i].ymax - input[i].ymin;
    areas.push_back(width * height);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].id != input[j].id) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].xmin, input[j].xmin);
      float yy1 = std::max(input[i].ymin, input[j].ymin);
      float xx2 = std::min(input[i].xmax, input[j].xmax);
      float yy2 = std::min(input[i].ymax, input[j].ymax);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }

    auto yolo_res = std::make_shared<YoloV5Result>(input[i].id,
                                                   input[i].xmin,
                                                   input[i].ymin,
                                                   input[i].xmax,
                                                   input[i].ymax,
                                                   input[i].score,
                                                   input[i].class_name);
    if (!yolo_res) {
      RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                   "invalid yolo_res");
    }

    result.push_back(yolo_res);
  }
}

int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width) {
  int h_index = 0;
  int w_index = 0;
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    h_index = 1;
    w_index = 2;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    h_index = 2;
    w_index = 3;
  } else {
    return -1;
  }
  *height = tensor->properties.validShape.dimensionSize[h_index];
  *width = tensor->properties.validShape.dimensionSize[w_index];
  return 0;
}

// 处理单个分支
void ParseYOLO11Branch(float* cls_ptr, float* bbox_ptr, int feat_height, int feat_width, float stride,
                       const std::vector<std::string>& class_names, float conf_inverse, float conf_thresh,
                       std::vector<YoloV5Result>& results) {
    int num_classes = 3;
    int dfl_len = 16;
    int anchor_num = feat_height * feat_width;
    
    // 生成anchors
    std::vector<std::pair<float, float>> anchors;
    anchors.reserve(anchor_num);
    for (int y = 0; y < feat_height; ++y) {
        for (int x = 0; x < feat_width; ++x) {
            anchors.emplace_back(x + 0.5f, y + 0.5f);
        }
    }
    
    for (int idx = 0; idx < anchor_num; ++idx) {
        float* cur_cls = cls_ptr + idx * num_classes;
        float* cur_bbox = bbox_ptr + idx * 64;
        
        // 分类分支
        float max_score = cur_cls[0];
        int max_id = 0;
        for (int c = 1; c < num_classes; ++c) {
            if (cur_cls[c] > max_score) {
                max_score = cur_cls[c];
                max_id = c;
            }
        }
        if (max_score < conf_inverse) continue;
        float score = 1.0f / (1.0f + std::exp(-max_score));
        if (score < conf_thresh) continue;

        // 回归分支
        float ltrb[4] = {0};
        for (int k = 0; k < 4; ++k) {
            float dfl_prob[16];
            softmax(cur_bbox + k * dfl_len, dfl_prob, dfl_len);
            float expv = 0.f;
            for (int j = 0; j < dfl_len; ++j) {
                expv += dfl_prob[j] * dfl_weights[j];
            }
            ltrb[k] = expv;
        }
        
        float ax = anchors[idx].first;
        float ay = anchors[idx].second;
        float x1 = (ax - ltrb[0]) * stride;
        float y1 = (ay - ltrb[1]) * stride;
        float x2 = (ax + ltrb[2]) * stride;
        float y2 = (ay + ltrb[3]) * stride;
        
        if (x2 <= 0 || y2 <= 0) continue;
        if (x1 > x2 || y1 > y2) continue;
        
        results.emplace_back(
            YoloV5Result(
                max_id,
                x1, y1, x2, y2,
                score,
                class_names[max_id]
            )
        );
    }
}

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::vector<std::shared_ptr<YoloV5Result>> &results,
    PTQYolo5Config &yolo5_config) {
    std::vector<YoloV5Result> parse_results;

    float conf_inverse = -std::log(1.0f / score_threshold_ - 1.0f);

    // 第一个分支 (60x80)
    {
        auto& tensor_bbox = node_output->output_tensors[0];
        auto& tensor_cls = node_output->output_tensors[1];
        float* bbox_ptr = reinterpret_cast<float*>(tensor_bbox->sysMem[0].virAddr);
        float* cls_ptr = reinterpret_cast<float*>(tensor_cls->sysMem[0].virAddr);
        ParseYOLO11Branch(cls_ptr, bbox_ptr, 60, 80, 8, yolo5_config.class_names, conf_inverse, score_threshold_, parse_results);
    }
    
    // 第二个分支 (30x40)
    {
        auto& tensor_bbox = node_output->output_tensors[2];
        auto& tensor_cls = node_output->output_tensors[3];
        float* bbox_ptr = reinterpret_cast<float*>(tensor_bbox->sysMem[0].virAddr);
        float* cls_ptr = reinterpret_cast<float*>(tensor_cls->sysMem[0].virAddr);
        ParseYOLO11Branch(cls_ptr, bbox_ptr, 30, 40, 16, yolo5_config.class_names, conf_inverse, score_threshold_, parse_results);
    }
    
    // 第三个分支 (15x20)
    {
        auto& tensor_bbox = node_output->output_tensors[4];
        auto& tensor_cls = node_output->output_tensors[5];
        float* bbox_ptr = reinterpret_cast<float*>(tensor_bbox->sysMem[0].virAddr);
        float* cls_ptr = reinterpret_cast<float*>(tensor_cls->sysMem[0].virAddr);
        ParseYOLO11Branch(cls_ptr, bbox_ptr, 15, 20, 32, yolo5_config.class_names, conf_inverse, score_threshold_, parse_results);
    }

    yolo5_nms(parse_results, nms_threshold_, nms_top_k_, results, false);
    return 0;
}

}  // namespace racing_obstacle_detection
}  // namespace dnn_node
}  // namespace hobot