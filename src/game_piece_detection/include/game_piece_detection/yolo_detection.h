#ifndef YOLO_DETECTION_H_
#define YOLO_DETECTION_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

// Class names for the reefscape model
const std::vector<std::string> CLASS_NAMES = {"algae", "coral"};
constexpr int NUM_CLASSES = 2;
constexpr int NUM_PREDICTIONS = 8400;
constexpr int VALUES_PER_PREDICTION = 6;  // 4 bbox + 2 class scores

class YoloDetection {
 public:
  YoloDetection(float x, float y, float w, float h, float conf, int cls)
      : x_(x), y_(y), w_(w), h_(h), conf_(conf), cls_(cls) {}

  // Getters
  float x() const { return x_; }
  float y() const { return y_; }
  float w() const { return w_; }
  float h() const { return h_; }
  float conf() const { return conf_; }
  int cls() const { return cls_; }

  std::string class_name() const {
    if (cls_ >= 0 && cls_ < static_cast<int>(CLASS_NAMES.size())) {
      return CLASS_NAMES[cls_];
    }
    return "unknown";
  }

  // Bounding box corners (for NMS IoU calculation)
  float x1() const { return x_ - w_ / 2.0f; }
  float y1() const { return y_ - h_ / 2.0f; }
  float x2() const { return x_ + w_ / 2.0f; }
  float y2() const { return y_ + h_ / 2.0f; }

  friend std::ostream& operator<<(std::ostream& os, const YoloDetection& d) {
    os << "class: " << d.class_name() << " (" << d.cls_ << "), "
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
 * Raw output shape: [1, 6, 8400]
 *   - 6 values: [x_center, y_center, width, height, algae_score, coral_score]
 *   - 8400 candidate predictions
 *
 * @param raw_output Pointer to raw model output (1 * 6 * 8400 floats)
 * @param conf_threshold Minimum confidence to keep detection
 * @param iou_threshold IoU threshold for NMS
 * @return Vector of filtered detections after NMS
 */
inline std::vector<YoloDetection> parse_yolov11_output(
    const float* raw_output, float conf_threshold = 0.25f,
    float iou_threshold = 0.45f) {
  std::vector<YoloDetection> candidates;
  candidates.reserve(1000);  // Pre-allocate for efficiency

  // Raw output is [1, 6, 8400] - stored as 6 rows of 8400 values
  // Row 0: all x_center values
  // Row 1: all y_center values
  // Row 2: all width values
  // Row 3: all height values
  // Row 4: all algae_score values
  // Row 5: all coral_score values

  const float* x_row = raw_output;
  const float* y_row = raw_output + NUM_PREDICTIONS;
  const float* w_row = raw_output + NUM_PREDICTIONS * 2;
  const float* h_row = raw_output + NUM_PREDICTIONS * 3;
  const float* class_scores = raw_output + NUM_PREDICTIONS * 4;  // Start of class scores

  for (int i = 0; i < NUM_PREDICTIONS; ++i) {
    // Find max class score and its index
    float max_score = 0.0f;
    int max_class = 0;

    for (int c = 0; c < NUM_CLASSES; ++c) {
      float score = class_scores[c * NUM_PREDICTIONS + i];
      if (score > max_score) {
        max_score = score;
        max_class = c;
      }
    }

    // Filter by confidence threshold
    if (max_score < conf_threshold) {
      continue;
    }

    // Extract bbox (already in pixel coordinates for 640x640)
    float x = x_row[i];
    float y = y_row[i];
    float w = w_row[i];
    float h = h_row[i];

    candidates.emplace_back(x, y, w, h, max_score, max_class);
  }

  // Apply NMS
  return apply_nms(candidates, iou_threshold);
}

/**
 * Scale detections from model input size to original image size.
 *
 * @param detections Detections in model input coordinates (640x640)
 * @param model_width Model input width (default 640)
 * @param model_height Model input height (default 640)
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
                        det.h() * scale_y, det.conf(), det.cls());
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
