#ifndef GAME_PIECE_DETECTION__MOCK_MODEL_INFERENCE_H_
#define GAME_PIECE_DETECTION__MOCK_MODEL_INFERENCE_H_

#include <algorithm>
#include <string>
#include <vector>

/**
 * Mock implementation of ModelInference for testing.
 * Returns configurable fake detections without requiring TensorRT.
 *
 * This allows unit tests to run without GPU or TensorRT dependencies,
 * making it suitable for CI environments.
 */
class MockModelInference {
 public:
  explicit MockModelInference(const std::string& enginePath)
      : engine_path_(enginePath) {
    // Simulate successful initialization
  }

  bool infer(const float* input, float* output) {
    (void)input;  // Suppress unused warning
    if (should_fail_inference_) {
      return false;
    }
    // Fill output with pre-configured detection data, clamped to the
    // configured output size to avoid overrunning the caller's buffer.
    if (!mock_output_.empty() && output_size_ > 0) {
      const size_t expected_floats = output_size_ / sizeof(float);
      const size_t copy_count = std::min(mock_output_.size(), expected_floats);
      std::copy_n(mock_output_.begin(), copy_count, output);
    }
    return true;
  }

  // Getters matching ModelInference interface
  int getInputChannels() const { return input_channels_; }
  int getInputHeight() const { return input_height_; }
  int getInputWidth() const { return input_width_; }
  int getNumClasses() const { return num_classes_; }
  int getNumPredictions() const { return num_predictions_; }
  size_t getOutputSize() const { return output_size_; }

  // Test configuration methods
  void setInputDims(int channels, int height, int width) {
    input_channels_ = channels;
    input_height_ = height;
    input_width_ = width;
  }

  void setOutputDims(int num_classes, int num_predictions) {
    num_classes_ = num_classes;
    num_predictions_ = num_predictions;
    output_size_ = (4 + num_classes_) * num_predictions_ * sizeof(float);
  }

  void setMockOutput(const std::vector<float>& output) {
    mock_output_ = output;
  }

  void setShouldFailInference(bool fail) {
    should_fail_inference_ = fail;
  }

  const std::string& getEnginePath() const { return engine_path_; }

 private:
  std::string engine_path_;
  int input_channels_ = 3;
  int input_height_ = 640;
  int input_width_ = 640;
  int num_classes_ = 1;
  int num_predictions_ = 8400;
  size_t output_size_ = (4 + 1) * 8400 * sizeof(float);
  std::vector<float> mock_output_;
  bool should_fail_inference_ = false;
};

#endif  // GAME_PIECE_DETECTION__MOCK_MODEL_INFERENCE_H_
