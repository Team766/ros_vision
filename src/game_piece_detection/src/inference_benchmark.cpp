#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <cmath>

#include "game_piece_detection/ModelInference.h"
#include "game_piece_detection/yolo_detection.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

/**
 * Preprocess an image for YOLOv11 inference.
 *
 * @param img Input BGR image (any size)
 * @param output Pre-allocated buffer for output
 * @param input_width Model input width
 * @param input_height Model input height
 * @param input_channels Number of input channels (1 or 3)
 */
void preprocess_image(const cv::Mat& img, float* output,
                      int input_width, int input_height, int input_channels) {
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(input_width, input_height));

  cv::Mat processed;
  if (input_channels == 3) {
    cv::cvtColor(resized, processed, cv::COLOR_BGR2RGB);
  } else if (input_channels == 1) {
    cv::cvtColor(resized, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = resized;
  }

  cv::Mat float_img;
  int cv_type = (input_channels == 1) ? CV_32FC1 : CV_32FC3;
  processed.convertTo(float_img, cv_type, 1.0 / 255.0);

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

struct BenchmarkStats {
  double mean;
  double std_dev;
  double min;
  double max;
  double median;
  double p95;
  double p99;
};

BenchmarkStats calculate_stats(std::vector<double>& times) {
  BenchmarkStats stats;

  size_t n = times.size();
  if (n == 0) {
    return {0, 0, 0, 0, 0, 0, 0};
  }

  // Sort for percentiles
  std::sort(times.begin(), times.end());

  // Mean
  stats.mean = std::accumulate(times.begin(), times.end(), 0.0) / n;

  // Std dev
  double sq_sum = 0;
  for (double t : times) {
    sq_sum += (t - stats.mean) * (t - stats.mean);
  }
  stats.std_dev = std::sqrt(sq_sum / n);

  // Min/Max
  stats.min = times.front();
  stats.max = times.back();

  // Median
  if (n % 2 == 0) {
    stats.median = (times[n/2 - 1] + times[n/2]) / 2.0;
  } else {
    stats.median = times[n/2];
  }

  // Percentiles
  stats.p95 = times[static_cast<size_t>(n * 0.95)];
  stats.p99 = times[static_cast<size_t>(n * 0.99)];

  return stats;
}

void print_usage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " --config <config_file> --image <image_file> [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Arguments:" << std::endl;
  std::cout << "  --config FILE    Path to system_config.json" << std::endl;
  std::cout << "  --image FILE     Path to test image" << std::endl;
  std::cout << "  --engine FILE    Optional: Override engine file from config" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --warmup N       Number of warmup iterations (default: 50)" << std::endl;
  std::cout << "  --iterations N   Number of benchmark iterations (default: 1000)" << std::endl;
  std::cout << "  --output FILE    Output CSV file (default: benchmark_results.csv)" << std::endl;
  std::cout << std::endl;
  std::cout << "Example:" << std::endl;
  std::cout << "  " << prog_name << " --config system_config.json --image test.jpg --iterations 500" << std::endl;
}

int main(int argc, char* argv[]) {
  std::string config_path;
  std::string image_path;
  std::string engine_override;
  int warmup_iterations = 50;
  int benchmark_iterations = 1000;
  std::string output_file = "benchmark_results.csv";

  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--image" && i + 1 < argc) {
      image_path = argv[++i];
    } else if (arg == "--engine" && i + 1 < argc) {
      engine_override = argv[++i];
    } else if (arg == "--warmup" && i + 1 < argc) {
      warmup_iterations = std::stoi(argv[++i]);
    } else if (arg == "--iterations" && i + 1 < argc) {
      benchmark_iterations = std::stoi(argv[++i]);
    } else if (arg == "--output" && i + 1 < argc) {
      output_file = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
  }

  if (config_path.empty() || image_path.empty()) {
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

  // Verify paths
  if (!fs::exists(engine_path)) {
    std::cerr << "Error: Engine file not found: " << engine_path << std::endl;
    return 1;
  }
  if (!fs::exists(image_path)) {
    std::cerr << "Error: Image file not found: " << image_path << std::endl;
    return 1;
  }

  std::cout << "=== Inference Benchmark ===" << std::endl;
  std::cout << "Config: " << config_path << std::endl;
  std::cout << "Engine: " << engine_path << std::endl;
  std::cout << "Image: " << image_path << std::endl;
  std::cout << "Warmup iterations: " << warmup_iterations << std::endl;
  std::cout << "Benchmark iterations: " << benchmark_iterations << std::endl;
  std::cout << "Output file: " << output_file << std::endl;
  std::cout << std::endl;

  // Load engine
  std::cout << "Loading TensorRT engine..." << std::endl;
  auto engine_load_start = std::chrono::high_resolution_clock::now();
  ModelInference model(engine_path);
  auto engine_load_end = std::chrono::high_resolution_clock::now();
  double engine_load_time = std::chrono::duration<double, std::milli>(engine_load_end - engine_load_start).count();
  std::cout << "Engine loaded in " << engine_load_time << " ms" << std::endl;

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

  // Load and preprocess image once
  std::cout << "Loading and preprocessing image..." << std::endl;
  cv::Mat img = cv::imread(image_path);
  if (img.empty()) {
    std::cerr << "Error: Could not load image: " << image_path << std::endl;
    return 1;
  }
  std::cout << "Image size: " << img.cols << "x" << img.rows << std::endl;

  size_t input_size = input_channels * input_width * input_height;
  size_t output_size = model.getOutputSize() / sizeof(float);
  std::vector<float> input_buffer(input_size);
  std::vector<float> output_buffer(output_size);

  // Preprocess image into input buffer (done once)
  preprocess_image(img, input_buffer.data(), input_width, input_height, input_channels);
  std::cout << std::endl;

  // Warmup phase
  std::cout << "Running warmup (" << warmup_iterations << " iterations)..." << std::endl;
  for (int i = 0; i < warmup_iterations; i++) {
    if (!model.infer(input_buffer.data(), output_buffer.data())) {
      std::cerr << "Error: Inference failed during warmup" << std::endl;
      return 1;
    }
  }
  std::cout << "Warmup complete." << std::endl;
  std::cout << std::endl;

  // Benchmark phase
  std::cout << "Running benchmark (" << benchmark_iterations << " iterations)..." << std::endl;
  std::vector<double> inference_times;
  std::vector<double> postprocess_times;
  std::vector<double> total_times;
  inference_times.reserve(benchmark_iterations);
  postprocess_times.reserve(benchmark_iterations);
  total_times.reserve(benchmark_iterations);

  for (int i = 0; i < benchmark_iterations; i++) {
    auto total_start = std::chrono::high_resolution_clock::now();

    // Inference
    auto infer_start = std::chrono::high_resolution_clock::now();
    if (!model.infer(input_buffer.data(), output_buffer.data())) {
      std::cerr << "Error: Inference failed at iteration " << i << std::endl;
      return 1;
    }
    auto infer_end = std::chrono::high_resolution_clock::now();

    // Post-processing (parsing detections)
    auto postprocess_start = std::chrono::high_resolution_clock::now();
    auto detections = parse_yolov11_output(
        output_buffer.data(), num_predictions, num_classes, class_names, 0.25f, 0.45f);
    auto postprocess_end = std::chrono::high_resolution_clock::now();

    auto total_end = std::chrono::high_resolution_clock::now();

    double infer_time = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
    double postprocess_time = std::chrono::duration<double, std::milli>(postprocess_end - postprocess_start).count();
    double total_time = std::chrono::duration<double, std::milli>(total_end - total_start).count();

    inference_times.push_back(infer_time);
    postprocess_times.push_back(postprocess_time);
    total_times.push_back(total_time);

    // Progress indicator
    if ((i + 1) % 100 == 0) {
      std::cout << "  " << (i + 1) << "/" << benchmark_iterations << " iterations complete" << std::endl;
    }
  }
  std::cout << "Benchmark complete." << std::endl;
  std::cout << std::endl;

  // Calculate statistics
  auto infer_stats = calculate_stats(inference_times);
  auto postprocess_stats = calculate_stats(postprocess_times);
  auto total_stats = calculate_stats(total_times);

  // Print results
  std::cout << "=== Results ===" << std::endl;
  std::cout << std::fixed << std::setprecision(3);
  std::cout << std::endl;

  std::cout << "Inference Time (ms):" << std::endl;
  std::cout << "  Mean:   " << infer_stats.mean << std::endl;
  std::cout << "  Std:    " << infer_stats.std_dev << std::endl;
  std::cout << "  Min:    " << infer_stats.min << std::endl;
  std::cout << "  Max:    " << infer_stats.max << std::endl;
  std::cout << "  Median: " << infer_stats.median << std::endl;
  std::cout << "  P95:    " << infer_stats.p95 << std::endl;
  std::cout << "  P99:    " << infer_stats.p99 << std::endl;
  std::cout << std::endl;

  std::cout << "Post-processing Time (ms):" << std::endl;
  std::cout << "  Mean:   " << postprocess_stats.mean << std::endl;
  std::cout << "  Std:    " << postprocess_stats.std_dev << std::endl;
  std::cout << "  Min:    " << postprocess_stats.min << std::endl;
  std::cout << "  Max:    " << postprocess_stats.max << std::endl;
  std::cout << std::endl;

  std::cout << "Total Time (ms):" << std::endl;
  std::cout << "  Mean:   " << total_stats.mean << std::endl;
  std::cout << "  Std:    " << total_stats.std_dev << std::endl;
  std::cout << "  Min:    " << total_stats.min << std::endl;
  std::cout << "  Max:    " << total_stats.max << std::endl;
  std::cout << "  Median: " << total_stats.median << std::endl;
  std::cout << "  P95:    " << total_stats.p95 << std::endl;
  std::cout << "  P99:    " << total_stats.p99 << std::endl;
  std::cout << std::endl;

  std::cout << "Throughput: " << (1000.0 / total_stats.mean) << " FPS" << std::endl;
  std::cout << std::endl;

  // Write CSV file
  std::ofstream csv(output_file);
  if (!csv.is_open()) {
    std::cerr << "Error: Could not open output file: " << output_file << std::endl;
    return 1;
  }

  csv << "iteration,inference_ms,postprocess_ms,total_ms" << std::endl;
  for (int i = 0; i < benchmark_iterations; i++) {
    csv << i << "," << inference_times[i] << "," << postprocess_times[i] << "," << total_times[i] << std::endl;
  }
  csv.close();

  std::cout << "Results written to: " << output_file << std::endl;

  return 0;
}
