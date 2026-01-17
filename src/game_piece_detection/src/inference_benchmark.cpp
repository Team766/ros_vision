#include <opencv2/opencv.hpp>
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

constexpr int MODEL_INPUT_WIDTH = 640;
constexpr int MODEL_INPUT_HEIGHT = 640;
constexpr int MODEL_CHANNELS = 3;

/**
 * Preprocess an image for YOLOv11 inference.
 */
void preprocess_image(const cv::Mat& img, float* output) {
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT));

  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

  cv::Mat float_img;
  rgb.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

  std::vector<cv::Mat> channels(3);
  cv::split(float_img, channels);

  size_t channel_size = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT;
  memcpy(output, channels[0].data, channel_size * sizeof(float));
  memcpy(output + channel_size, channels[1].data, channel_size * sizeof(float));
  memcpy(output + 2 * channel_size, channels[2].data, channel_size * sizeof(float));
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
  std::cout << "Usage: " << prog_name << " <engine_file> <image_file> [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Arguments:" << std::endl;
  std::cout << "  engine_file    Path to TensorRT engine file (.engine)" << std::endl;
  std::cout << "  image_file     Path to test image" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --warmup N     Number of warmup iterations (default: 50)" << std::endl;
  std::cout << "  --iterations N Number of benchmark iterations (default: 1000)" << std::endl;
  std::cout << "  --output FILE  Output CSV file (default: benchmark_results.csv)" << std::endl;
  std::cout << std::endl;
  std::cout << "Example:" << std::endl;
  std::cout << "  " << prog_name << " model.engine test.jpg --iterations 500 --output results.csv" << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    print_usage(argv[0]);
    return 1;
  }

  std::string engine_path = argv[1];
  std::string image_path = argv[2];
  int warmup_iterations = 50;
  int benchmark_iterations = 1000;
  std::string output_file = "benchmark_results.csv";

  // Parse optional arguments
  for (int i = 3; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--warmup" && i + 1 < argc) {
      warmup_iterations = std::stoi(argv[++i]);
    } else if (arg == "--iterations" && i + 1 < argc) {
      benchmark_iterations = std::stoi(argv[++i]);
    } else if (arg == "--output" && i + 1 < argc) {
      output_file = argv[++i];
    }
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
  std::cout << std::endl;

  // Load and preprocess image once
  std::cout << "Loading and preprocessing image..." << std::endl;
  cv::Mat img = cv::imread(image_path);
  if (img.empty()) {
    std::cerr << "Error: Could not load image: " << image_path << std::endl;
    return 1;
  }
  std::cout << "Image size: " << img.cols << "x" << img.rows << std::endl;

  size_t input_size = MODEL_CHANNELS * MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT;
  size_t output_size = model.getOutputSize() / sizeof(float);
  std::vector<float> input_buffer(input_size);
  std::vector<float> output_buffer(output_size);

  // Preprocess image into input buffer (done once)
  preprocess_image(img, input_buffer.data());
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
    auto detections = parse_yolov11_output(output_buffer.data(), 0.25f, 0.45f);
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
