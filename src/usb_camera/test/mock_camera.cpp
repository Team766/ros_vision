#include "mock_camera.hpp"
#include <opencv2/opencv.hpp>

bool MockCamera::open(int device_id, int api_preference) {
  open_call_count_++;
  last_device_id_ = device_id;
  last_api_preference_ = api_preference;
  
  // Simulate successful opening unless explicitly configured otherwise
  is_opened_ = true;
  return is_opened_;
}

bool MockCamera::isOpened() const {
  return is_opened_;
}

bool MockCamera::read(cv::Mat& frame) {
  read_call_count_++;
  
  if (!is_opened_) {
    return false;
  }
  
  if (read_should_fail_) {
    return false;
  }
  
  if (synthetic_frames_.empty()) {
    // Generate a default test frame if none provided
    frame = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::circle(frame, cv::Point(320, 240), 50, cv::Scalar(0, 255, 0), -1);
    cv::putText(frame, "Test Frame", cv::Point(50, 100), 
                cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
    return true;
  }
  
  // Return frames in sequence, looping back to start
  frame = synthetic_frames_[current_frame_index_].clone();
  current_frame_index_ = (current_frame_index_ + 1) % synthetic_frames_.size();
  return true;
}

bool MockCamera::set(int prop_id, double value) {
  if (!is_opened_) {
    return false;
  }
  
  properties_[prop_id] = value;
  return true;
}

double MockCamera::get(int prop_id) const {
  auto it = properties_.find(prop_id);
  if (it != properties_.end()) {
    return it->second;
  }
  
  // Return reasonable defaults for common properties
  switch (prop_id) {
    case cv::CAP_PROP_FRAME_WIDTH:
      return 640.0;
    case cv::CAP_PROP_FRAME_HEIGHT:
      return 480.0;
    case cv::CAP_PROP_FPS:
      return 30.0;
    case cv::CAP_PROP_BUFFERSIZE:
      return 1.0;
    default:
      return 0.0;
  }
}

void MockCamera::release() {
  is_opened_ = false;
  synthetic_frames_.clear();
  current_frame_index_ = 0;
  properties_.clear();
}

void MockCamera::setSyntheticFrame(const cv::Mat& frame) {
  synthetic_frames_.clear();
  synthetic_frames_.push_back(frame.clone());
  current_frame_index_ = 0;
}

void MockCamera::setSyntheticFrames(const std::vector<cv::Mat>& frames) {
  synthetic_frames_.clear();
  for (const auto& frame : frames) {
    synthetic_frames_.push_back(frame.clone());
  }
  current_frame_index_ = 0;
}

void MockCamera::setOpened(bool opened) {
  is_opened_ = opened;
}

void MockCamera::setReadFailure(bool should_fail) {
  read_should_fail_ = should_fail;
}

void MockCamera::setPropertyValue(int prop_id, double value) {
  properties_[prop_id] = value;
}
