#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include "vision_utils/config_loader.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

class ConfigLoaderTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a temporary directory for test files
    test_dir_ = fs::temp_directory_path() / "vision_utils_test";
    fs::create_directories(test_dir_);
    
    // Create test config files
    createValidConfig();
    createLegacyConfig();
    createInvalidConfig();
    createMissingCameraConfig();
  }

  void TearDown() override {
    // Clean up test files
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  void createValidConfig() {
    json config = {
      {"camera_mounted_positions", {
        {"CAM001", {
          {"location", "center_front"},
          {"format", "MJPG"},
          {"width", 1920},
          {"height", 1080},
          {"frame_rate", 60},
          {"api_preference", "V4L2"}
        }},
        {"CAM002", {
          {"location", "left_side"},
          {"format", "YUYV"},
          {"width", 1280},
          {"height", 720},
          {"frame_rate", 30},
          {"api_preference", "V4L2"}
        }}
      }}
    };
    
    fs::path config_file = test_dir_ / "valid_config.json";
    std::ofstream file(config_file);
    file << config.dump(2);
    file.close();
    valid_config_path_ = config_file.string();
  }

  void createLegacyConfig() {
    json config = {
      {"camera_mounted_positions", {
        {"CAM003", "rear_camera"},
        {"CAM004", "side_camera"}
      }}
    };
    
    fs::path config_file = test_dir_ / "legacy_config.json";
    std::ofstream file(config_file);
    file << config.dump(2);
    file.close();
    legacy_config_path_ = config_file.string();
  }

  void createInvalidConfig() {
    fs::path config_file = test_dir_ / "invalid_config.json";
    std::ofstream file(config_file);
    file << "{ invalid json content";
    file.close();
    invalid_config_path_ = config_file.string();
  }

  void createMissingCameraConfig() {
    json config = {
      {"camera_mounted_positions", {
        {"CAM999", {
          {"location", "test_location"},
          {"format", "MJPG"}
          // Missing width, height, frame_rate, api_preference
        }}
      }}
    };
    
    fs::path config_file = test_dir_ / "missing_camera_config.json";
    std::ofstream file(config_file);
    file << config.dump(2);
    file.close();
    missing_camera_config_path_ = config_file.string();
  }

  fs::path test_dir_;
  std::string valid_config_path_;
  std::string legacy_config_path_;
  std::string invalid_config_path_;
  std::string missing_camera_config_path_;
};

// Test loading a valid camera configuration
TEST_F(ConfigLoaderTest, LoadValidCameraConfig) {
  // Override the config file path for testing
  vision_utils::ConfigLoader::setConfigFilePath(valid_config_path_);
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->location, "center_front");
  EXPECT_EQ(config->format, "MJPG");
  EXPECT_EQ(config->width, 1920);
  EXPECT_EQ(config->height, 1080);
  EXPECT_EQ(config->frame_rate, 60);
  EXPECT_EQ(config->api_preference, "V4L2");
}

// Test loading a second valid camera configuration
TEST_F(ConfigLoaderTest, LoadSecondValidCameraConfig) {
  vision_utils::ConfigLoader::setConfigFilePath(valid_config_path_);
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM002");
  
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->location, "left_side");
  EXPECT_EQ(config->format, "YUYV");
  EXPECT_EQ(config->width, 1280);
  EXPECT_EQ(config->height, 720);
  EXPECT_EQ(config->frame_rate, 30);
  EXPECT_EQ(config->api_preference, "V4L2");
}

// Test handling non-existent camera serial
TEST_F(ConfigLoaderTest, NonExistentCameraSerial) {
  vision_utils::ConfigLoader::setConfigFilePath(valid_config_path_);
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("NONEXISTENT");
  
  EXPECT_FALSE(config.has_value());
}

// Test handling legacy configuration format (string values)
TEST_F(ConfigLoaderTest, LegacyConfigFormat) {
  vision_utils::ConfigLoader::setConfigFilePath(legacy_config_path_);
  
  // Legacy format should return nullopt since it doesn't have required fields
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM003");
  
  EXPECT_FALSE(config.has_value());
}

// Test handling invalid JSON file
TEST_F(ConfigLoaderTest, InvalidJsonFile) {
  vision_utils::ConfigLoader::setConfigFilePath(invalid_config_path_);
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  
  EXPECT_FALSE(config.has_value());
}

// Test handling non-existent config file
TEST_F(ConfigLoaderTest, NonExistentConfigFile) {
  vision_utils::ConfigLoader::setConfigFilePath("/nonexistent/path/config.json");
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  
  EXPECT_FALSE(config.has_value());
}

// Test handling missing required fields in camera config
TEST_F(ConfigLoaderTest, MissingRequiredFields) {
  vision_utils::ConfigLoader::setConfigFilePath(missing_camera_config_path_);
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM999");
  
  EXPECT_FALSE(config.has_value());
}

// Test configuration caching behavior
TEST_F(ConfigLoaderTest, ConfigurationCaching) {
  vision_utils::ConfigLoader::setConfigFilePath(valid_config_path_);
  
  // Load config twice - should use cached version on second call
  auto config1 = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  auto config2 = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  
  ASSERT_TRUE(config1.has_value());
  ASSERT_TRUE(config2.has_value());
  
  // Verify they have the same values
  EXPECT_EQ(config1->location, config2->location);
  EXPECT_EQ(config1->width, config2->width);
  EXPECT_EQ(config1->height, config2->height);
}

// Test force reload functionality
TEST_F(ConfigLoaderTest, ForceReload) {
  vision_utils::ConfigLoader::setConfigFilePath(valid_config_path_);
  
  // Load initial config
  auto config1 = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  ASSERT_TRUE(config1.has_value());
  
  // Force reload
  vision_utils::ConfigLoader::reloadConfig();
  
  // Load again
  auto config2 = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  ASSERT_TRUE(config2.has_value());
  
  // Should still have same values
  EXPECT_EQ(config1->location, config2->location);
  EXPECT_EQ(config1->width, config2->width);
}

// Test empty config file
TEST_F(ConfigLoaderTest, EmptyConfigFile) {
  fs::path empty_config = test_dir_ / "empty_config.json";
  std::ofstream file(empty_config);
  file << "{}";
  file.close();
  
  vision_utils::ConfigLoader::setConfigFilePath(empty_config.string());
  
  auto config = vision_utils::ConfigLoader::getCameraConfig("CAM001");
  
  EXPECT_FALSE(config.has_value());
}
