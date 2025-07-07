#pragma once
#include "apriltags_cuda/msg/tag_detection.hpp"
#include "apriltags_cuda/msg/tag_detection_array.hpp"

namespace apriltags_cuda {

inline apriltags_cuda::msg::TagDetection make_tag_detection(int id, double x, double y, double z) {
    apriltags_cuda::msg::TagDetection msg;
    msg.id = id;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    return msg;
}

inline void add_tag_detection(
    apriltags_cuda::msg::TagDetectionArray &array,
    int id, double x, double y, double z) {
    array.detections.push_back(make_tag_detection(id, x, y, z));
}

} // namespace apriltags_cuda
