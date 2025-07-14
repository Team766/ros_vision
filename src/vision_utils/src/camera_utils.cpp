#include "vision_utils/camera_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace vision_utils {

CameraData get_camera_data(const std::string& camera_serial) {
    namespace fs = std::filesystem;
    using json = nlohmann::json;

    fs::path config_path = ament_index_cpp::get_package_share_directory("vision_config_data");
    fs::path config_file = config_path / "data" / "system_config.json";

    if (!fs::exists(config_file)) {
        throw std::runtime_error("Unable to find system config file at path: " + config_file.string());
    }

    std::ifstream f(config_file);
    json data = json::parse(f);

    if (!data.contains("camera_mounted_positions")) {
        throw std::runtime_error("Unable to find key 'camera_mounted_positions' in system config file");
    }

    if (!data["camera_mounted_positions"].contains(camera_serial)) {
        throw std::runtime_error("Unable to find camera serial '" + camera_serial + "' in system config file");
    }
    auto camera_data = data["camera_mounted_positions"][camera_serial];
    CameraData result;
    result.width = camera_data["width"].get<int>();
    result.height = camera_data["height"].get<int>();
    result.frame_rate = camera_data["frame_rate"].get<int>();
    result.serial_number = camera_serial;
    return result;
}

} // namespace vision_utils
