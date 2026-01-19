#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>

#include "game_piece_detection/ModelInference.h"
#include "game_piece_detection/yolo_detection.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

constexpr float CONF_THRESHOLD = 0.25f;
constexpr float IOU_THRESHOLD = 0.45f;

/**
 * Preprocess an image for YOLOv11 inference.
 * - Resize to model input size
 * - Convert BGR to RGB (if 3 channels) or grayscale
 * - Normalize to [0, 1]
 * - Convert to NCHW format (channels first)
 *
 * @param img Input BGR image (any size)
 * @param output Pre-allocated buffer for output
 * @param input_width Model input width
 * @param input_height Model input height
 * @param input_channels Number of input channels (1 or 3)
 */
void preprocess_image(const cv::Mat& img, float* output,
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
 * @param img Image to draw on (modified in place)
 * @param detections Detections in original image coordinates
 */
void draw_detections(cv::Mat& img, const std::vector<YoloDetection>& detections) {
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

void print_usage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " --config <config_file> --images <image_dir> [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Arguments:" << std::endl;
  std::cout << "  --config FILE    Path to system_config.json" << std::endl;
  std::cout << "  --images DIR     Directory containing test images" << std::endl;
  std::cout << "  --output DIR     Optional: Directory to save annotated images" << std::endl;
  std::cout << "  --engine FILE    Optional: Override engine file from config" << std::endl;
  std::cout << std::endl;
  std::cout << "Example:" << std::endl;
  std::cout << "  " << prog_name << " --config system_config.json --images /path/to/images" << std::endl;
}

int main(int argc, char* argv[]) {
  std::string config_path;
  std::string image_dir;
  std::string output_dir;
  std::string engine_override;

  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--images" && i + 1 < argc) {
      image_dir = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_dir = argv[++i];
    } else if (arg == "--engine" && i + 1 < argc) {
      engine_override = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
  }

  if (config_path.empty() || image_dir.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  // Load configuration
  if (!fs::exists(config_path)) {
    std::cerr << "Error: Config file not found: " << config_path << std::endl;
    return 1;
  }

  std::ifstream config_file(config_path);
  json config;
  try {
    config_file >> config;
  } catch (const json::parse_error& e) {
    std::cerr << "Error parsing config file: " << e.what() << std::endl;
    return 1;
  }

  // Extract game_piece_detection configuration
  if (!config.contains("game_piece_detection")) {
    std::cerr << "Error: Config file missing 'game_piece_detection' section" << std::endl;
    return 1;
  }

  auto& gpd_config = config["game_piece_detection"];

  std::string engine_path = engine_override.empty()
      ? gpd_config.value("engine_file", "")
      : engine_override;
  int config_input_channels = gpd_config.value("input_channels", 3);
  std::vector<std::string> class_names;
  if (gpd_config.contains("class_names")) {
    for (const auto& name : gpd_config["class_names"]) {
      class_names.push_back(name.get<std::string>());
    }
  }

  if (engine_path.empty()) {
    std::cerr << "Error: No engine file specified in config or command line" << std::endl;
    return 1;
  }

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

  // Print model info from engine
  model.printEngineInfo();

  // Get dimensions from engine
  int input_channels = model.getInputChannels();
  int input_height = model.getInputHeight();
  int input_width = model.getInputWidth();
  int num_classes = model.getNumClasses();
  int num_predictions = model.getNumPredictions();

  // Verify input channels match config
  if (input_channels != config_input_channels) {
    std::cerr << "Warning: Engine input channels (" << input_channels
              << ") differs from config (" << config_input_channels << ")" << std::endl;
  }

  // Verify class names match number of classes
  if (static_cast<int>(class_names.size()) != num_classes) {
    std::cerr << "Warning: Number of class names in config (" << class_names.size()
              << ") differs from model classes (" << num_classes << ")" << std::endl;
    // Pad or truncate class names as needed
    while (static_cast<int>(class_names.size()) < num_classes) {
      class_names.push_back("class_" + std::to_string(class_names.size()));
    }
  }

  std::cout << "Class names: [";
  for (size_t i = 0; i < class_names.size(); ++i) {
    std::cout << class_names[i] << (i < class_names.size() - 1 ? ", " : "");
  }
  std::cout << "]" << std::endl;
  std::cout << std::endl;

  // Allocate buffers using engine dimensions
  size_t input_size = input_channels * input_width * input_height;
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

  std::cout << "Found " << image_paths.size() << " images to process\n" << std::endl;

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

    // Preprocess using engine dimensions
    preprocess_image(img, input_buffer.data(), input_width, input_height, input_channels);

    // Run inference
    auto start_infer = std::chrono::high_resolution_clock::now();
    if (!model.infer(input_buffer.data(), output_buffer.data())) {
      std::cerr << "  Error: Inference failed" << std::endl;
      continue;
    }
    auto end_infer = std::chrono::high_resolution_clock::now();
    auto infer_time = std::chrono::duration_cast<std::chrono::microseconds>(end_infer - start_infer).count() / 1000.0;
    total_inference_time += infer_time;

    // Parse detections using engine dimensions and config class names
    auto detections = parse_yolov11_output(
        output_buffer.data(), num_predictions, num_classes, class_names,
        CONF_THRESHOLD, IOU_THRESHOLD);

    // Scale to original image size
    auto scaled_detections = scale_detections(detections, input_width, input_height,
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
