#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "game_piece_detection/detection_utils.h"
#include "game_piece_detection/yolo_detection.h"
#include "game_piece_detection/MockModelInference.h"

namespace fs = std::filesystem;

class GamePieceDetectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create temporary test directory
    test_dir_ = fs::temp_directory_path() / "game_piece_detection_test";
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  fs::path test_dir_;
};

// =============================================================================
// preprocess_image tests
// =============================================================================

// Test preprocess_image function for RGB (3-channel) model
TEST_F(GamePieceDetectionTest, PreprocessImageRGB) {
  // Simulate BGR input from camera (what cv_bridge gives us)
  cv::Mat input(480, 640, CV_8UC3, cv::Scalar(100, 150, 200));  // BGR values
  std::vector<float> output(3 * 640 * 640);

  preprocess_image(input, output.data(), 640, 640, 3);

  // Verify output is normalized (values between 0 and 1)
  for (float val : output) {
    EXPECT_GE(val, 0.0f);
    EXPECT_LE(val, 1.0f);
  }

  // Verify BGR->RGB conversion happened (first channel should be R, not B)
  // Input was BGR(100, 150, 200), so after conversion to RGB:
  // R=200, G=150, B=100 normalized to ~0.78, ~0.59, ~0.39
  float expected_r = 200.0f / 255.0f;
  float expected_g = 150.0f / 255.0f;
  float expected_b = 100.0f / 255.0f;

  // Check first pixel of each channel (CHW format)
  size_t channel_size = 640 * 640;
  EXPECT_NEAR(output[0], expected_r, 0.01f);                    // R channel
  EXPECT_NEAR(output[channel_size], expected_g, 0.01f);         // G channel
  EXPECT_NEAR(output[2 * channel_size], expected_b, 0.01f);     // B channel
}

// Test preprocess_image function for mono (1-channel) model
TEST_F(GamePieceDetectionTest, PreprocessImageMono) {
  // Simulate BGR input - preprocess should convert to grayscale for mono model
  cv::Mat input(480, 640, CV_8UC3, cv::Scalar(100, 100, 100));  // Gray in BGR
  std::vector<float> output(1 * 640 * 640);

  preprocess_image(input, output.data(), 640, 640, 1);

  // Verify output is normalized (values between 0 and 1)
  for (float val : output) {
    EXPECT_GE(val, 0.0f);
    EXPECT_LE(val, 1.0f);
  }

  // For uniform gray input, all output values should be approximately the same
  float expected_gray = 100.0f / 255.0f;
  EXPECT_NEAR(output[0], expected_gray, 0.01f);
  EXPECT_NEAR(output[640 * 320], expected_gray, 0.01f);  // Middle pixel
}

// Test preprocess_image handles resize correctly
TEST_F(GamePieceDetectionTest, PreprocessImageResizes) {
  // Non-square input should be resized to model dimensions
  cv::Mat input(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
  std::vector<float> output(3 * 640 * 640);

  // Should not crash and should produce correct output size
  EXPECT_NO_THROW(preprocess_image(input, output.data(), 640, 640, 3));

  // All values should be normalized
  float expected = 128.0f / 255.0f;
  EXPECT_NEAR(output[0], expected, 0.01f);
}

// =============================================================================
// draw_detections tests
// =============================================================================

// Test draw_detections function
TEST_F(GamePieceDetectionTest, DrawDetectionsDoesNotCrash) {
  cv::Mat img(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<YoloDetection> detections = {
    YoloDetection(320.0f, 320.0f, 100.0f, 100.0f, 0.95f, 0, "ball")
  };

  // Should not throw
  EXPECT_NO_THROW(draw_detections(img, detections));

  // Image should be modified (not all black anymore)
  cv::Scalar mean = cv::mean(img);
  EXPECT_GT(mean[0] + mean[1] + mean[2], 0);
}

TEST_F(GamePieceDetectionTest, DrawDetectionsHandlesEmptyList) {
  cv::Mat img(640, 640, CV_8UC3, cv::Scalar(50, 50, 50));
  std::vector<YoloDetection> detections;

  // Should not crash with empty list
  EXPECT_NO_THROW(draw_detections(img, detections));

  // Image should be unchanged
  cv::Scalar mean = cv::mean(img);
  EXPECT_NEAR(mean[0], 50, 1);
  EXPECT_NEAR(mean[1], 50, 1);
  EXPECT_NEAR(mean[2], 50, 1);
}

TEST_F(GamePieceDetectionTest, DrawDetectionsClampsToImageBounds) {
  cv::Mat img(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
  // Detection partially outside image bounds
  std::vector<YoloDetection> detections = {
    YoloDetection(90.0f, 90.0f, 50.0f, 50.0f, 0.95f, 0, "ball")  // Extends past 100x100
  };

  // Should not crash
  EXPECT_NO_THROW(draw_detections(img, detections));
}

// =============================================================================
// MockModelInference tests
// =============================================================================

TEST_F(GamePieceDetectionTest, MockModelInferenceReturnsConfiguredOutput) {
  MockModelInference mock("fake_engine.engine");
  mock.setInputDims(3, 640, 640);
  mock.setOutputDims(1, 8400);

  // Configure a simple detection
  std::vector<float> fake_output((4 + 1) * 8400, 0.0f);
  // Set one detection at center with high confidence
  fake_output[0] = 320.0f;           // x
  fake_output[8400] = 320.0f;        // y
  fake_output[8400*2] = 100.0f;      // w
  fake_output[8400*3] = 100.0f;      // h
  fake_output[8400*4] = 0.99f;       // class 0 confidence
  mock.setMockOutput(fake_output);

  std::vector<float> input(3 * 640 * 640, 0.5f);
  std::vector<float> output((4 + 1) * 8400);

  EXPECT_TRUE(mock.infer(input.data(), output.data()));
  EXPECT_FLOAT_EQ(output[0], 320.0f);
}

TEST_F(GamePieceDetectionTest, MockModelInferenceCanFail) {
  MockModelInference mock("fake_engine.engine");
  mock.setShouldFailInference(true);

  std::vector<float> input(3 * 640 * 640);
  std::vector<float> output(5 * 8400);

  EXPECT_FALSE(mock.infer(input.data(), output.data()));
}

TEST_F(GamePieceDetectionTest, MockModelInferenceGetters) {
  MockModelInference mock("test_engine.engine");
  mock.setInputDims(1, 320, 320);
  mock.setOutputDims(2, 4200);

  EXPECT_EQ(mock.getInputChannels(), 1);
  EXPECT_EQ(mock.getInputHeight(), 320);
  EXPECT_EQ(mock.getInputWidth(), 320);
  EXPECT_EQ(mock.getNumClasses(), 2);
  EXPECT_EQ(mock.getNumPredictions(), 4200);
  EXPECT_EQ(mock.getEnginePath(), "test_engine.engine");
}

// =============================================================================
// YOLO output parsing tests
// =============================================================================

TEST_F(GamePieceDetectionTest, ParseYoloOutputFindsDetection) {
  const int num_predictions = 8400;
  const int num_classes = 1;
  std::vector<float> output((4 + num_classes) * num_predictions, 0.0f);

  // Add one detection at index 0
  output[0] = 320.0f;                      // x center
  output[num_predictions] = 320.0f;        // y center
  output[num_predictions*2] = 100.0f;      // width
  output[num_predictions*3] = 100.0f;      // height
  output[num_predictions*4] = 0.95f;       // class 0 score

  std::vector<std::string> class_names = {"ball"};
  auto detections = parse_yolov11_output(
      output.data(), num_predictions, num_classes, class_names, 0.25f, 0.45f);

  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].class_name(), "ball");
  EXPECT_FLOAT_EQ(detections[0].x(), 320.0f);
  EXPECT_FLOAT_EQ(detections[0].y(), 320.0f);
  EXPECT_GT(detections[0].conf(), 0.9f);
}

TEST_F(GamePieceDetectionTest, ParseYoloOutputFiltersLowConfidence) {
  const int num_predictions = 8400;
  const int num_classes = 1;
  std::vector<float> output((4 + num_classes) * num_predictions, 0.0f);

  // Add detection with confidence below threshold
  output[0] = 320.0f;
  output[num_predictions] = 320.0f;
  output[num_predictions*2] = 100.0f;
  output[num_predictions*3] = 100.0f;
  output[num_predictions*4] = 0.10f;  // Below 0.25 threshold

  std::vector<std::string> class_names = {"ball"};
  auto detections = parse_yolov11_output(
      output.data(), num_predictions, num_classes, class_names, 0.25f, 0.45f);

  EXPECT_EQ(detections.size(), 0u);
}

TEST_F(GamePieceDetectionTest, ParseYoloOutputMultipleClasses) {
  const int num_predictions = 8400;
  const int num_classes = 3;
  std::vector<float> output((4 + num_classes) * num_predictions, 0.0f);

  // Add detection for class 2 (highest score)
  output[0] = 100.0f;                        // x
  output[num_predictions] = 100.0f;          // y
  output[num_predictions*2] = 50.0f;         // w
  output[num_predictions*3] = 50.0f;         // h
  output[num_predictions*4] = 0.30f;         // class 0 score
  output[num_predictions*5] = 0.20f;         // class 1 score
  output[num_predictions*6] = 0.90f;         // class 2 score (highest)

  std::vector<std::string> class_names = {"ball", "cone", "cube"};
  auto detections = parse_yolov11_output(
      output.data(), num_predictions, num_classes, class_names, 0.25f, 0.45f);

  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].class_name(), "cube");
  EXPECT_EQ(detections[0].cls(), 2);
}

// =============================================================================
// scale_detections tests
// =============================================================================

TEST_F(GamePieceDetectionTest, ScaleDetectionsCorrectly) {
  std::vector<YoloDetection> detections = {
    YoloDetection(320.0f, 320.0f, 100.0f, 100.0f, 0.95f, 0, "ball")
  };

  // Scale from 640x640 to 1280x960
  auto scaled = scale_detections(detections, 640, 640, 1280, 960);

  ASSERT_EQ(scaled.size(), 1u);
  EXPECT_FLOAT_EQ(scaled[0].x(), 640.0f);   // 320 * 2
  EXPECT_FLOAT_EQ(scaled[0].y(), 480.0f);   // 320 * 1.5
  EXPECT_FLOAT_EQ(scaled[0].w(), 200.0f);   // 100 * 2
  EXPECT_FLOAT_EQ(scaled[0].h(), 150.0f);   // 100 * 1.5
}

TEST_F(GamePieceDetectionTest, ScaleDetectionsPreservesMetadata) {
  std::vector<YoloDetection> detections = {
    YoloDetection(100.0f, 100.0f, 50.0f, 50.0f, 0.87f, 2, "cube")
  };

  auto scaled = scale_detections(detections, 640, 640, 640, 640);

  ASSERT_EQ(scaled.size(), 1u);
  EXPECT_FLOAT_EQ(scaled[0].conf(), 0.87f);
  EXPECT_EQ(scaled[0].cls(), 2);
  EXPECT_EQ(scaled[0].class_name(), "cube");
}

TEST_F(GamePieceDetectionTest, ScaleDetectionsHandlesEmptyList) {
  std::vector<YoloDetection> detections;
  auto scaled = scale_detections(detections, 640, 640, 1920, 1080);
  EXPECT_EQ(scaled.size(), 0u);
}

// =============================================================================
// YoloDetection class tests
// =============================================================================

TEST_F(GamePieceDetectionTest, YoloDetectionCornerCoordinates) {
  // Center at (100, 100) with width=40, height=60
  YoloDetection det(100.0f, 100.0f, 40.0f, 60.0f, 0.9f, 0, "test");

  // x1 = 100 - 40/2 = 80
  // y1 = 100 - 60/2 = 70
  // x2 = 100 + 40/2 = 120
  // y2 = 100 + 60/2 = 130
  EXPECT_FLOAT_EQ(det.x1(), 80.0f);
  EXPECT_FLOAT_EQ(det.y1(), 70.0f);
  EXPECT_FLOAT_EQ(det.x2(), 120.0f);
  EXPECT_FLOAT_EQ(det.y2(), 130.0f);
}

// =============================================================================
// Integration-style test
// =============================================================================

TEST_F(GamePieceDetectionTest, FullPipelineWithMock) {
  // Simulate the full detection pipeline using mock
  MockModelInference mock("test.engine");
  mock.setInputDims(3, 640, 640);
  mock.setOutputDims(1, 8400);

  // Configure mock to return a detection
  std::vector<float> fake_output((4 + 1) * 8400, 0.0f);
  fake_output[0] = 320.0f;           // x center
  fake_output[8400] = 240.0f;        // y center
  fake_output[8400*2] = 80.0f;       // width
  fake_output[8400*3] = 80.0f;       // height
  fake_output[8400*4] = 0.92f;       // confidence
  mock.setMockOutput(fake_output);

  // Create input image
  cv::Mat bgr_img(480, 640, CV_8UC3, cv::Scalar(100, 100, 100));

  // Allocate buffers
  std::vector<float> input_buffer(3 * 640 * 640);
  std::vector<float> output_buffer((4 + 1) * 8400);

  // Preprocess
  preprocess_image(bgr_img, input_buffer.data(),
                   mock.getInputWidth(), mock.getInputHeight(),
                   mock.getInputChannels());

  // Run mock inference
  ASSERT_TRUE(mock.infer(input_buffer.data(), output_buffer.data()));

  // Parse detections
  std::vector<std::string> class_names = {"ball"};
  auto detections = parse_yolov11_output(
      output_buffer.data(),
      mock.getNumPredictions(),
      mock.getNumClasses(),
      class_names,
      DEFAULT_CONF_THRESHOLD,
      DEFAULT_IOU_THRESHOLD);

  ASSERT_EQ(detections.size(), 1u);

  // Scale to original size
  auto scaled = scale_detections(detections, 640, 640, 640, 480);
  ASSERT_EQ(scaled.size(), 1u);

  // Draw on image
  draw_detections(bgr_img, scaled);

  // Verify image was modified (drawing adds some pixels with different colors)
  cv::Scalar mean = cv::mean(bgr_img);
  EXPECT_GT(mean[0] + mean[1] + mean[2], 290);  // Should have some color from drawing
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
