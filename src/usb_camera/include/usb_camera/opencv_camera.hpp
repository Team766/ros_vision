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

#ifndef USB_CAMERA__OPENCV_CAMERA_HPP_
#define USB_CAMERA__OPENCV_CAMERA_HPP_

#include "usb_camera/camera_interface.hpp"
#include <opencv2/opencv.hpp>

/**
 * @brief Real camera implementation using OpenCV VideoCapture
 *
 * This class wraps cv::VideoCapture to implement the CameraInterface,
 * providing access to real USB cameras through OpenCV's backends.
 */
class OpenCVCamera : public CameraInterface
{
public:
  OpenCVCamera() = default;
  ~OpenCVCamera() override;

  bool open(int device_id, int api_preference) override;
  bool isOpened() const override;
  bool read(cv::Mat & frame) override;
  bool set(int prop_id, double value) override;
  double get(int prop_id) const override;
  void release() override;

private:
  cv::VideoCapture cap_;
};

#endif  // USB_CAMERA__OPENCV_CAMERA_HPP_
