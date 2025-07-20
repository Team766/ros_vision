#include <gtest/gtest.h>
#include "vision_utils/config_loader.hpp"

using namespace vision_utils;

class ConfigLoaderUtilityTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Test utility functions that don't require config files
  }
};

// Test format string to FourCC conversion
TEST_F(ConfigLoaderUtilityTest, FormatStringToFourCC) {
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("MJPG"), 1196444237);
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("MJPEG"), 1196444237);
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("YUYV"), 1448695129);
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("BGR3"), 861030210);
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("H264"), 875967048);
  
  // Test unknown format defaults to MJPG
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("RGB3"), 1196444237); // Unknown, defaults to MJPG
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("UNKNOWN"), 1196444237);
  EXPECT_EQ(ConfigLoader::formatStringToFourCC(""), 1196444237);
}

// Test API preference string to code conversion
TEST_F(ConfigLoaderUtilityTest, ApiStringToCode) {
  EXPECT_EQ(ConfigLoader::apiStringToCode("V4L2"), 200);
  EXPECT_EQ(ConfigLoader::apiStringToCode("ANY"), 0);
  EXPECT_EQ(ConfigLoader::apiStringToCode("GSTREAMER"), 1800);
  EXPECT_EQ(ConfigLoader::apiStringToCode("FFMPEG"), 1900);
  
  // Test unknown API defaults to V4L2
  EXPECT_EQ(ConfigLoader::apiStringToCode("UNKNOWN"), 200);
  EXPECT_EQ(ConfigLoader::apiStringToCode(""), 200);
}

// Test case sensitivity
TEST_F(ConfigLoaderUtilityTest, CaseSensitivity) {
  // These should not match (case sensitive)
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("mjpg"), 1196444237); // defaults to MJPG
  EXPECT_EQ(ConfigLoader::formatStringToFourCC("Mjpg"), 1196444237); // defaults to MJPG
  
  EXPECT_EQ(ConfigLoader::apiStringToCode("v4l2"), 200); // defaults to V4L2
  EXPECT_EQ(ConfigLoader::apiStringToCode("V4l2"), 200); // defaults to V4L2
}

// Test edge cases
TEST_F(ConfigLoaderUtilityTest, EdgeCases) {
  // Empty strings
  auto empty_config = ConfigLoader::getCameraConfig("");
  EXPECT_FALSE(empty_config.has_value());
  
  // Very long strings
  std::string long_serial(1000, 'A');
  auto long_config = ConfigLoader::getCameraConfig(long_serial);
  EXPECT_FALSE(long_config.has_value());
  
  // Special characters
  auto special_config = ConfigLoader::getCameraConfig("CAM@#$%");
  EXPECT_FALSE(special_config.has_value());
}
