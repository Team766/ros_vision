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

#include "vision_utils/config_loader.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace vision_utils
{

// Static member definitions
bool ConfigLoader::config_loaded_ = false;
std::map<std::string, CameraConfig> ConfigLoader::camera_configs_;
std::map<std::string, ExtrinsicConfig> ConfigLoader::extrinsic_configs_;
NetworkTablesConfig ConfigLoader::network_config_;
std::string ConfigLoader::custom_config_path_ = "";

std::string ConfigLoader::getConfigFilePath()
{
  // Use custom path if set (for testing)
  if (!custom_config_path_.empty()) {
    return custom_config_path_;
  }
  
  try {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("vision_config_data");
    return package_share_directory + "/data/system_config.json";
  } catch (const std::exception& e) {
    std::cerr << "Error finding vision_config_data package: " << e.what() << std::endl;
    return "";
  }
}

bool ConfigLoader::loadSystemConfig()
{
  if (config_loaded_) {
    return true;
  }

  std::string config_path = getConfigFilePath();
  if (config_path.empty()) {
    std::cerr << "Could not find system config file path" << std::endl;
    return false;
  }

  std::ifstream file(config_path);
  if (!file.is_open()) {
    std::cerr << "Could not open system config file: " << config_path << std::endl;
    return false;
  }

  json root;
  try {
    file >> root;
  } catch (const json::parse_error& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
    return false;
  }

  // Parse camera configurations
  if (root.contains("camera_mounted_positions") && root["camera_mounted_positions"].is_object()) {
    for (const auto& [serial, config_obj] : root["camera_mounted_positions"].items()) {
      if (config_obj.is_object()) {
        CameraConfig config;
        
        // Validate that all required fields are present
        bool has_all_required = 
          config_obj.contains("location") && config_obj["location"].is_string() &&
          config_obj.contains("format") && config_obj["format"].is_string() &&
          config_obj.contains("height") && config_obj["height"].is_number_integer() &&
          config_obj.contains("width") && config_obj["width"].is_number_integer() &&
          config_obj.contains("frame_rate") && config_obj["frame_rate"].is_number_integer() &&
          config_obj.contains("api_preference") && config_obj["api_preference"].is_string();
        
        if (!has_all_required) {
          // Skip this camera config if required fields are missing
          continue;
        }
        
        config.location = config_obj["location"];
        config.format = config_obj["format"];
        config.height = config_obj["height"];
        config.width = config_obj["width"];
        config.frame_rate = config_obj["frame_rate"];
        config.api_preference = config_obj["api_preference"];
        
        camera_configs_[serial] = config;
      }
    }
  }

  // Parse extrinsic configurations
  if (root.contains("extrinsics") && root["extrinsics"].is_object()) {
    for (const auto& [location, extrinsic_obj] : root["extrinsics"].items()) {
      if (extrinsic_obj.is_object()) {
        ExtrinsicConfig config;
        
        if (extrinsic_obj.contains("rotation") && extrinsic_obj["rotation"].is_array()) {
          const auto& rotation_array = extrinsic_obj["rotation"];
          for (size_t i = 0; i < std::min(rotation_array.size(), size_t(3)); ++i) {
            if (rotation_array[i].is_array()) {
              const auto& row = rotation_array[i];
              for (size_t j = 0; j < std::min(row.size(), size_t(3)); ++j) {
                if (row[j].is_number()) {
                  config.rotation[i][j] = row[j];
                }
              }
            }
          }
        }
        
        if (extrinsic_obj.contains("offset") && extrinsic_obj["offset"].is_array()) {
          const auto& offset_array = extrinsic_obj["offset"];
          for (size_t i = 0; i < std::min(offset_array.size(), size_t(3)); ++i) {
            if (offset_array[i].is_number()) {
              config.offset[i] = offset_array[i];
            }
          }
        }
        
        extrinsic_configs_[location] = config;
      }
    }
  }

  // Parse network tables configuration
  if (root.contains("network_tables_config") && root["network_tables_config"].is_object()) {
    const auto& nt_config = root["network_tables_config"];
    
    if (nt_config.contains("table_address") && nt_config["table_address"].is_string()) {
      network_config_.table_address = nt_config["table_address"];
    }
    if (nt_config.contains("table_name") && nt_config["table_name"].is_string()) {
      network_config_.table_name = nt_config["table_name"];
    }
  }

  config_loaded_ = true;
  return true;
}

std::optional<CameraConfig> ConfigLoader::getCameraConfig(const std::string & camera_serial)
{
  if (!loadSystemConfig()) {
    return std::nullopt;
  }

  auto it = camera_configs_.find(camera_serial);
  if (it != camera_configs_.end()) {
    return it->second;
  }

  return std::nullopt;
}

std::optional<ExtrinsicConfig> ConfigLoader::getExtrinsicConfig(const std::string & location)
{
  if (!loadSystemConfig()) {
    return std::nullopt;
  }

  auto it = extrinsic_configs_.find(location);
  if (it != extrinsic_configs_.end()) {
    return it->second;
  }

  return std::nullopt;
}

NetworkTablesConfig ConfigLoader::getNetworkTablesConfig()
{
  if (!loadSystemConfig()) {
    return NetworkTablesConfig(); // Return default config
  }

  return network_config_;
}

int ConfigLoader::formatStringToFourCC(const std::string & format_str)
{
  // Return a simple hash or enum value that can be converted to OpenCV fourcc later
  if (format_str == "MJPG" || format_str == "MJPEG") {
    return 1196444237; // 'MJPG' as int
  } else if (format_str == "YUYV") {
    return 1448695129; // 'YUYV' as int  
  } else if (format_str == "H264") {
    return 875967048; // 'H264' as int
  } else if (format_str == "BGR3") {
    return 861030210; // 'BGR3' as int
  }
  
  // Default to MJPG if unknown format
  return 1196444237;
}

int ConfigLoader::apiStringToCode(const std::string & api_str)
{
  // Return constants that match OpenCV CAP_ constants
  if (api_str == "V4L2") {
    return 200; // cv::CAP_V4L2
  } else if (api_str == "ANY") {
    return 0; // cv::CAP_ANY
  } else if (api_str == "GSTREAMER") {
    return 1800; // cv::CAP_GSTREAMER
  } else if (api_str == "FFMPEG") {
    return 1900; // cv::CAP_FFMPEG
  }
  
  // Default to V4L2 if unknown API
  return 200;
}

void ConfigLoader::reloadConfig()
{
  config_loaded_ = false;
  camera_configs_.clear();
  extrinsic_configs_.clear();
  network_config_ = NetworkTablesConfig();
}

void ConfigLoader::setConfigFilePath(const std::string & path)
{
  custom_config_path_ = path;
  // Force reload with new path
  reloadConfig();
}

}  // namespace vision_utils
