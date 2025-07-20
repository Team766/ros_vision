#ifndef USB_CAMERA__TEST__MOCK_CAMERA_HPP_
#define USB_CAMERA__TEST__MOCK_CAMERA_HPP_

#include "usb_camera/camera_interface.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

/**
 * @brief Mock camera implementation for testing
 * 
 * This class provides a fake camera that can be configured to return
 * specific frames, simulate camera failures, and test various scenarios
 * without requiring actual hardware.
 */
class MockCamera : public CameraInterface {
 public:
  MockCamera() = default;
  ~MockCamera() override = default;

  // CameraInterface implementation
  bool open(int device_id, int api_preference) override;
  bool isOpened() const override;
  bool read(cv::Mat& frame) override;
  bool set(int prop_id, double value) override;
  double get(int prop_id) const override;
  void release() override;

  // Test configuration methods
  void setSyntheticFrame(const cv::Mat& frame);
  void setSyntheticFrames(const std::vector<cv::Mat>& frames);
  void setOpened(bool opened);
  void setReadFailure(bool should_fail);
  void setPropertyValue(int prop_id, double value);

  // Test inspection methods
  int getOpenCallCount() const { return open_call_count_; }
  int getReadCallCount() const { return read_call_count_; }
  int getLastDeviceId() const { return last_device_id_; }
  int getLastApiPreference() const { return last_api_preference_; }

 private:
  bool is_opened_ = false;
  bool read_should_fail_ = false;
  std::vector<cv::Mat> synthetic_frames_;
  size_t current_frame_index_ = 0;
  std::map<int, double> properties_;

  // Test tracking
  int open_call_count_ = 0;
  int read_call_count_ = 0;
  int last_device_id_ = -1;
  int last_api_preference_ = -1;
};

#endif  // USB_CAMERA__TEST__MOCK_CAMERA_HPP_
