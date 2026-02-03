#ifndef GAME_PIECE_DETECTION__DETECTION_UTILS_H_
#define GAME_PIECE_DETECTION__DETECTION_UTILS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstring>
#include <algorithm>
#include "game_piece_detection/yolo_detection.h"

// Default thresholds (can be overridden when calling functions)
constexpr float DEFAULT_CONF_THRESHOLD = 0.25f;
constexpr float DEFAULT_IOU_THRESHOLD = 0.45f;

/**
 * Preprocess an image for YOLOv11 inference.
 *
 * Handles both RGB (3-channel) and grayscale (1-channel) models:
 * - input_channels == 3: Converts BGR input to RGB, then to CHW
 * - input_channels == 1: Converts BGR input to grayscale, outputs single channel
 *
 * Operations performed:
 * 1. Resize to model input size (input_width x input_height)
 * 2. Color conversion (BGR->RGB or BGR->Gray based on input_channels)
 * 3. Normalize pixel values to [0, 1]
 * 4. Convert from HWC to CHW format (channels first, for GPU)
 *
 * @param img Input BGR image (from cv_bridge, any size)
 * @param output Pre-allocated float buffer of size (input_channels * input_width * input_height)
 * @param input_width Model input width (e.g., 640)
 * @param input_height Model input height (e.g., 640)
 * @param input_channels Model input channels (1 for grayscale, 3 for RGB)
 */
inline void preprocess_image(const cv::Mat& img, float* output,
                             int input_width, int input_height, int input_channels) {
  // Resize to model input size
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(input_width, input_height));

  cv::Mat processed;
  if (input_channels == 3) {
    // Convert BGR to RGB
    cv::cvtColor(resized, processed, cv::COLOR_BGR2RGB);
  } else if (input_channels == 1) {
    // Convert to grayscale
    cv::cvtColor(resized, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = resized;
  }

  // Convert to float and normalize to [0, 1]
  cv::Mat float_img;
  int cv_type = (input_channels == 1) ? CV_32FC1 : CV_32FC3;
  processed.convertTo(float_img, cv_type, 1.0 / 255.0);

  // Convert from HWC to CHW (NCHW with batch=1)
  size_t channel_size = input_width * input_height;
  if (input_channels == 1) {
    memcpy(output, float_img.data, channel_size * sizeof(float));
  } else {
    std::vector<cv::Mat> channels(input_channels);
    cv::split(float_img, channels);
    for (int c = 0; c < input_channels; ++c) {
      memcpy(output + c * channel_size, channels[c].data, channel_size * sizeof(float));
    }
  }
}

/**
 * Draw detections on an image.
 *
 * Draws bounding boxes and labels for each detection. Colors are assigned
 * based on class index for visual distinction.
 *
 * @param img Image to draw on (modified in place, should be BGR for display)
 * @param detections Detections in image coordinates (after scaling if needed)
 */
inline void draw_detections(cv::Mat& img, const std::vector<YoloDetection>& detections) {
  // Generate colors for classes
  const std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 255, 0),    // green
      cv::Scalar(0, 165, 255),  // orange
      cv::Scalar(255, 0, 0),    // blue
      cv::Scalar(0, 255, 255),  // yellow
      cv::Scalar(255, 0, 255),  // magenta
      cv::Scalar(255, 255, 0),  // cyan
  };

  for (const auto& det : detections) {
    // Convert center format to corner format
    int x1 = static_cast<int>(det.x1());
    int y1 = static_cast<int>(det.y1());
    int x2 = static_cast<int>(det.x2());
    int y2 = static_cast<int>(det.y2());

    // Clamp to image bounds
    x1 = std::max(0, std::min(x1, img.cols - 1));
    y1 = std::max(0, std::min(y1, img.rows - 1));
    x2 = std::max(0, std::min(x2, img.cols - 1));
    y2 = std::max(0, std::min(y2, img.rows - 1));

    cv::Scalar color = colors[det.cls() % colors.size()];

    // Draw bounding box
    cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

    // Draw label background
    std::string label = det.class_name() + " " +
                        std::to_string(static_cast<int>(det.conf() * 100)) + "%";
    int baseline;
    cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int label_y = std::max(y1, label_size.height + 5);
    cv::rectangle(img,
                  cv::Point(x1, label_y - label_size.height - 5),
                  cv::Point(x1 + label_size.width, label_y + baseline - 5),
                  color, cv::FILLED);

    // Draw label text
    cv::putText(img, label, cv::Point(x1, label_y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }
}

#endif  // GAME_PIECE_DETECTION__DETECTION_UTILS_H_
