#ifndef YOLO_DETECTION_H_
#define YOLO_DETECTION_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

class YoloDetection {
 public:
  YoloDetection(float x, float y, float w, float h, float conf, int cls,
                const std::string& class_name = "unknown")
      : x_(x), y_(y), w_(w), h_(h), conf_(conf), cls_(cls), class_name_(class_name) {}

  // Getters
  float x() const { return x_; }
  float y() const { return y_; }
  float w() const { return w_; }
  float h() const { return h_; }
  float conf() const { return conf_; }
  int cls() const { return cls_; }
  const std::string& class_name() const { return class_name_; }

  // Bounding box corners (for NMS IoU calculation)
  float x1() const { return x_ - w_ / 2.0f; }
  float y1() const { return y_ - h_ / 2.0f; }
  float x2() const { return x_ + w_ / 2.0f; }
  float y2() const { return y_ + h_ / 2.0f; }

  friend std::ostream& operator<<(std::ostream& os, const YoloDetection& d) {
    os << "class: " << d.class_name_ << " (" << d.cls_ << "), "
       << "conf: " << d.conf_ << ", "
       << "bbox: [" << d.x_ << ", " << d.y_ << ", " << d.w_ << ", " << d.h_
       << "]";
    return os;
  }

 private:
  float x_;  // center x
  float y_;  // center y
  float w_;  // width
  float h_;  // height
  float conf_;
  int cls_;
  std::string class_name_;
};

/**
 * Calculate Intersection over Union (IoU) between two detections.
 */
inline float calculate_iou(const YoloDetection& a, const YoloDetection& b) {
  float x1 = std::max(a.x1(), b.x1());
  float y1 = std::max(a.y1(), b.y1());
  float x2 = std::min(a.x2(), b.x2());
  float y2 = std::min(a.y2(), b.y2());

  float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  float area_a = a.w() * a.h();
  float area_b = b.w() * b.h();
  float union_area = area_a + area_b - intersection;

  return union_area > 0 ? intersection / union_area : 0.0f;
}

/**
 * Apply Non-Maximum Suppression to filter overlapping detections.
 *
 * @param detections Input detections (will be sorted by confidence)
 * @param iou_threshold IoU threshold for suppression (default 0.45)
 * @return Filtered detections after NMS
 */
inline std::vector<YoloDetection> apply_nms(
    std::vector<YoloDetection>& detections, float iou_threshold = 0.45f) {
  if (detections.empty()) {
    return {};
  }

  // Sort by confidence (descending)
  std::sort(detections.begin(), detections.end(),
            [](const YoloDetection& a, const YoloDetection& b) {
              return a.conf() > b.conf();
            });

  std::vector<bool> suppressed(detections.size(), false);
  std::vector<YoloDetection> result;

  for (size_t i = 0; i < detections.size(); ++i) {
    if (suppressed[i]) continue;

    result.push_back(detections[i]);

    for (size_t j = i + 1; j < detections.size(); ++j) {
      if (suppressed[j]) continue;

      // Only suppress if same class
      if (detections[i].cls() == detections[j].cls()) {
        float iou = calculate_iou(detections[i], detections[j]);
        if (iou > iou_threshold) {
          suppressed[j] = true;
        }
      }
    }
  }

  return result;
}

/**
 * Parse raw YOLOv11 output tensor and extract detections.
 *
 * Raw output shape: [1, N, P]
 *   - N values: [x_center, y_center, width, height, class_0_score, ..., class_n_score]
 *   - P candidate predictions
 *
 * @param raw_output Pointer to raw model output
 * @param num_predictions Number of predictions (P dimension)
 * @param num_classes Number of classes in the model
 * @param class_names Vector of class names (size must match num_classes)
 * @param conf_threshold Minimum confidence to keep detection
 * @param iou_threshold IoU threshold for NMS
 * @return Vector of filtered detections after NMS
 */
inline std::vector<YoloDetection> parse_yolov11_output(
    const float* raw_output,
    int num_predictions,
    int num_classes,
    const std::vector<std::string>& class_names,
    float conf_threshold = 0.25f,
    float iou_threshold = 0.45f) {
  std::vector<YoloDetection> candidates;
  candidates.reserve(1000);  // Pre-allocate for efficiency

  // Raw output is [1, N, P] - stored as N rows of P values
  // Row 0: all x_center values
  // Row 1: all y_center values
  // Row 2: all width values
  // Row 3: all height values
  // Row 4+: class score values

  const float* x_row = raw_output;
  const float* y_row = raw_output + num_predictions;
  const float* w_row = raw_output + num_predictions * 2;
  const float* h_row = raw_output + num_predictions * 3;
  const float* class_scores = raw_output + num_predictions * 4;  // Start of class scores

  for (int i = 0; i < num_predictions; ++i) {
    // Find max class score and its index
    float max_score = 0.0f;
    int max_class = 0;

    for (int c = 0; c < num_classes; ++c) {
      float score = class_scores[c * num_predictions + i];
      if (score > max_score) {
        max_score = score;
        max_class = c;
      }
    }

    // Filter by confidence threshold
    if (max_score < conf_threshold) {
      continue;
    }

    // Extract bbox (already in pixel coordinates for model input size)
    float x = x_row[i];
    float y = y_row[i];
    float w = w_row[i];
    float h = h_row[i];

    // Get class name
    std::string name = (max_class >= 0 && max_class < static_cast<int>(class_names.size()))
                           ? class_names[max_class]
                           : "unknown";

    candidates.emplace_back(x, y, w, h, max_score, max_class, name);
  }

  // Apply NMS
  return apply_nms(candidates, iou_threshold);
}

/**
 * Scale detections from model input size to original image size.
 *
 * @param detections Detections in model input coordinates
 * @param model_width Model input width
 * @param model_height Model input height
 * @param orig_width Original image width
 * @param orig_height Original image height
 * @return Detections scaled to original image coordinates
 */
inline std::vector<YoloDetection> scale_detections(
    const std::vector<YoloDetection>& detections, int model_width,
    int model_height, int orig_width, int orig_height) {
  float scale_x = static_cast<float>(orig_width) / model_width;
  float scale_y = static_cast<float>(orig_height) / model_height;

  std::vector<YoloDetection> scaled;
  scaled.reserve(detections.size());

  for (const auto& det : detections) {
    scaled.emplace_back(det.x() * scale_x, det.y() * scale_y, det.w() * scale_x,
                        det.h() * scale_y, det.conf(), det.cls(), det.class_name());
  }

  return scaled;
}

/**
 * Print detections summary.
 */
inline void print_detections(const std::vector<YoloDetection>& detections) {
  std::cout << "Found " << detections.size() << " detections:" << std::endl;
  for (const auto& det : detections) {
    std::cout << "  " << det << std::endl;
  }
}

#endif  // YOLO_DETECTION_H_
