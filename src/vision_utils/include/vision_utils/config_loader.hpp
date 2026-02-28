// Copyright 2025 Team766
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISION_UTILS__CONFIG_LOADER_HPP_
#define VISION_UTILS__CONFIG_LOADER_HPP_

#include <string>
#include <memory>
#include <optional>
#include <array>
#include <map>

namespace vision_utils
{

/**
 * @brief Camera configuration parameters
 */
struct CameraConfig
{
  std::string location;
  std::string format;
  int height;
  int width;
  int frame_rate;
  std::string api_preference;

  // Default constructor with sensible defaults
  CameraConfig()
  : location("center_front"),
    format("MJPG"),
    height(800),
    width(1280),
    frame_rate(100),
    api_preference("V4L2")
  {
  }
};

/**
 * @brief Extrinsic camera parameters
 */
struct ExtrinsicConfig
{
  std::array<std::array<double, 3>, 3> rotation;
  std::array<double, 3> offset;

  // Default constructor with identity transform
  ExtrinsicConfig()
  : rotation{{{0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}}},
    offset{{0.0, 0.0, 0.0}}
  {
  }
};

/**
 * @brief Network Tables configuration
 */
struct NetworkTablesConfig
{
  std::string table_address;
  std::string table_name;

  // Default constructor
  NetworkTablesConfig()
  : table_address("10.7.66.2"),
    table_name("/SmartDashboard")
  {
  }
};

/**
 * @brief Configuration loader for vision system
 * 
 * Loads configuration data from JSON files in the vision_config_data package.
 * Provides access to camera settings, extrinsics, and network configuration.
 */
class ConfigLoader
{
public:
  /**
   * @brief Get camera configuration by serial number
   * @param camera_serial The camera serial number to look up
   * @return Camera configuration if found, std::nullopt otherwise
   */
  static std::optional<CameraConfig> getCameraConfig(const std::string & camera_serial);

  /**
   * @brief Get extrinsic configuration by location name
   * @param location The camera location name
   * @return Extrinsic configuration if found, std::nullopt otherwise
   */
  static std::optional<ExtrinsicConfig> getExtrinsicConfig(const std::string & location);

  /**
   * @brief Get network tables configuration
   * @return Network tables configuration
   */
  static NetworkTablesConfig getNetworkTablesConfig();

  /**
   * @brief Convert OpenCV format string to fourcc code
   * @param format_str Format string (e.g., "MJPG", "YUYV")
   * @return OpenCV fourcc code
   */
  static int formatStringToFourCC(const std::string & format_str);

  /**
   * @brief Convert API preference string to OpenCV API code
   * @param api_str API preference string (e.g., "V4L2", "ANY")
   * @return OpenCV API preference code
   */
  static int apiStringToCode(const std::string & api_str);

  /**
   * @brief Get visualization downsample factor
   * @return Downsample factor (1 = no downsampling)
   */
  static int getVisualizationDownsampleFactor();

  /**
   * @brief Force reload of configuration (useful for testing)
   */
  static void reloadConfig();

  /**
   * @brief Set custom config file path (for testing)
   * @param path Path to config file
   */
  static void setConfigFilePath(const std::string & path);

private:
  /**
   * @brief Load and parse the system configuration file
   * @return True if loaded successfully, false otherwise
   */
  static bool loadSystemConfig();

  /**
   * @brief Get the path to the system configuration file
   * @return Path to system_config.json
   */
  static std::string getConfigFilePath();

  // Static data members to cache loaded configuration
  static bool config_loaded_;
  static std::map<std::string, CameraConfig> camera_configs_;
  static std::map<std::string, ExtrinsicConfig> extrinsic_configs_;
  static NetworkTablesConfig network_config_;
  static std::string custom_config_path_;  // For testing
  static int visualization_downsample_factor_;
};

}  // namespace vision_utils

#endif  // VISION_UTILS__CONFIG_LOADER_HPP_
