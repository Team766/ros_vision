#pragma once
#include <string>
#include <nlohmann/json.hpp>

namespace vision_utils {

struct CameraData {
    int width;
    int height;
    int frame_rate;
    std::string serial_number;
};

// Throws std::runtime_error on error
CameraData get_camera_data(const std::string& camera_serial);

} // namespace vision_utils
