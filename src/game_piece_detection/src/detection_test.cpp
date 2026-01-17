#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <vector>
#include <string>

#include "game_piece_detection/ModelInference.h"
#include "game_piece_detection/yolo_detection.h"

namespace fs = std::filesystem;

constexpr int MODEL_INPUT_WIDTH = 640;
constexpr int MODEL_INPUT_HEIGHT = 640;
constexpr int MODEL_CHANNELS = 3;
constexpr float CONF_THRESHOLD = 0.25f;
constexpr float IOU_THRESHOLD = 0.45f;

/**
 * Preprocess an image for YOLOv11 inference.
 * - Resize to 640x640
 * - Convert BGR to RGB
 * - Normalize to [0, 1]
 * - Convert to NCHW format (channels first)
 *
 * @param img Input BGR image (any size)
 * @param output Pre-allocated buffer for output (size: 3 * 640 * 640)
 */
void preprocess_image(const cv::Mat& img, float* output) {
  // Resize to model input size
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT));

  // Convert BGR to RGB
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

  // Convert to float and normalize to [0, 1]
  cv::Mat float_img;
  rgb.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

  // Convert from HWC to CHW (NCHW with batch=1)
  // Output layout: [R channel (640*640), G channel (640*640), B channel (640*640)]
  std::vector<cv::Mat> channels(3);
  cv::split(float_img, channels);

  size_t channel_size = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT;
  memcpy(output, channels[0].data, channel_size * sizeof(float));
  memcpy(output + channel_size, channels[1].data, channel_size * sizeof(float));
  memcpy(output + 2 * channel_size, channels[2].data, channel_size * sizeof(float));
}

/**
 * Draw detections on an image.
 *
 * @param img Image to draw on (modified in place)
 * @param detections Detections in original image coordinates
 */
void draw_detections(cv::Mat& img, const std::vector<YoloDetection>& detections) {
  // Colors for different classes (BGR format)
  const std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 255, 0),   // algae - green
      cv::Scalar(0, 165, 255)  // coral - orange
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

void print_usage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " <engine_file> <image_dir> [output_dir]" << std::endl;
  std::cout << std::endl;
  std::cout << "Arguments:" << std::endl;
  std::cout << "  engine_file  Path to TensorRT engine file (.engine)" << std::endl;
  std::cout << "  image_dir    Directory containing test images" << std::endl;
  std::cout << "  output_dir   Optional: Directory to save annotated images" << std::endl;
  std::cout << std::endl;
  std::cout << "Example:" << std::endl;
  std::cout << "  " << prog_name << " model.engine /path/to/images /path/to/output" << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    print_usage(argv[0]);
    return 1;
  }

  std::string engine_path = argv[1];
  std::string image_dir = argv[2];
  std::string output_dir = (argc > 3) ? argv[3] : "";

  // Verify paths exist
  if (!fs::exists(engine_path)) {
    std::cerr << "Error: Engine file not found: " << engine_path << std::endl;
    return 1;
  }
  if (!fs::is_directory(image_dir)) {
    std::cerr << "Error: Image directory not found: " << image_dir << std::endl;
    return 1;
  }

  // Create output directory if specified
  if (!output_dir.empty()) {
    fs::create_directories(output_dir);
    std::cout << "Saving annotated images to: " << output_dir << std::endl;
  }

  // Load TensorRT engine
  std::cout << "Loading engine from: " << engine_path << std::endl;
  auto start_load = std::chrono::high_resolution_clock::now();
  ModelInference model(engine_path);
  auto end_load = std::chrono::high_resolution_clock::now();
  auto load_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_load - start_load).count();
  std::cout << "Engine loaded in " << load_time << " ms" << std::endl;

  // Print model info
  auto input_dims = model.getInputDims();
  auto output_dims = model.getOutputDims();
  std::cout << "Input shape: [";
  for (int i = 0; i < input_dims.nbDims; i++) {
    std::cout << input_dims.d[i] << (i < input_dims.nbDims - 1 ? ", " : "");
  }
  std::cout << "]" << std::endl;
  std::cout << "Output shape: [";
  for (int i = 0; i < output_dims.nbDims; i++) {
    std::cout << output_dims.d[i] << (i < output_dims.nbDims - 1 ? ", " : "");
  }
  std::cout << "]" << std::endl;

  // Allocate buffers
  size_t input_size = MODEL_CHANNELS * MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT;
  size_t output_size = model.getOutputSize() / sizeof(float);
  std::vector<float> input_buffer(input_size);
  std::vector<float> output_buffer(output_size);

  // Collect image paths
  std::vector<fs::path> image_paths;
  for (const auto& entry : fs::directory_iterator(image_dir)) {
    if (entry.is_regular_file()) {
      std::string ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
        image_paths.push_back(entry.path());
      }
    }
  }
  std::sort(image_paths.begin(), image_paths.end());

  std::cout << "\nFound " << image_paths.size() << " images to process\n" << std::endl;

  // Process each image
  int total_detections = 0;
  double total_inference_time = 0;

  for (const auto& img_path : image_paths) {
    std::cout << "Processing: " << img_path.filename() << std::endl;

    // Load image
    cv::Mat img = cv::imread(img_path.string());
    if (img.empty()) {
      std::cerr << "  Error: Could not load image" << std::endl;
      continue;
    }

    int orig_width = img.cols;
    int orig_height = img.rows;

    // Preprocess
    preprocess_image(img, input_buffer.data());

    // Run inference
    auto start_infer = std::chrono::high_resolution_clock::now();
    if (!model.infer(input_buffer.data(), output_buffer.data())) {
      std::cerr << "  Error: Inference failed" << std::endl;
      continue;
    }
    auto end_infer = std::chrono::high_resolution_clock::now();
    auto infer_time = std::chrono::duration_cast<std::chrono::microseconds>(end_infer - start_infer).count() / 1000.0;
    total_inference_time += infer_time;

    // Parse detections
    auto detections = parse_yolov11_output(output_buffer.data(), CONF_THRESHOLD, IOU_THRESHOLD);

    // Scale to original image size
    auto scaled_detections = scale_detections(detections, MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT,
                                               orig_width, orig_height);

    total_detections += scaled_detections.size();

    std::cout << "  Size: " << orig_width << "x" << orig_height
              << ", Inference: " << std::fixed << std::setprecision(2) << infer_time << " ms"
              << ", Detections: " << scaled_detections.size() << std::endl;

    for (const auto& det : scaled_detections) {
      std::cout << "    " << det << std::endl;
    }

    // Save annotated image if output directory specified
    if (!output_dir.empty()) {
      draw_detections(img, scaled_detections);
      std::string output_path = (fs::path(output_dir) / img_path.filename()).string();
      cv::imwrite(output_path, img);
    }
  }

  // Print summary
  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << "Images processed: " << image_paths.size() << std::endl;
  std::cout << "Total detections: " << total_detections << std::endl;
  if (!image_paths.empty()) {
    std::cout << "Average inference time: " << std::fixed << std::setprecision(2)
              << (total_inference_time / image_paths.size()) << " ms" << std::endl;
  }

  return 0;
}
