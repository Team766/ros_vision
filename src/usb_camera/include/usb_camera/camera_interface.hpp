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

#ifndef USB_CAMERA__CAMERA_INTERFACE_HPP_
#define USB_CAMERA__CAMERA_INTERFACE_HPP_

#include <opencv2/opencv.hpp>

/**
 * @brief Abstract interface for camera operations
 *
 * This interface allows for dependency injection of camera implementations,
 * enabling testing with mock cameras and supporting different camera backends.
 */
class CameraInterface
{
public:
  virtual ~CameraInterface() = default;

  /**
   * @brief Open camera device
   * @param device_id Camera device identifier (e.g., 0, 1, 2...)
   * @param api_preference OpenCV capture API preference (e.g., cv::CAP_V4L2)
   * @return true if camera opened successfully, false otherwise
   */
  virtual bool open(int device_id, int api_preference) = 0;

  /**
   * @brief Check if camera is currently opened
   * @return true if camera is open and ready, false otherwise
   */
  virtual bool isOpened() const = 0;

  /**
   * @brief Capture a frame from the camera
   * @param frame Output frame that will be populated with captured image
   * @return true if frame captured successfully, false otherwise
   */
  virtual bool read(cv::Mat & frame) = 0;

  /**
   * @brief Set camera property
   * @param prop_id OpenCV property ID (e.g., cv::CAP_PROP_FRAME_WIDTH)
   * @param value Property value to set
   * @return true if property was set successfully, false otherwise
   */
  virtual bool set(int prop_id, double value) = 0;

  /**
   * @brief Get camera property value
   * @param prop_id OpenCV property ID (e.g., cv::CAP_PROP_FRAME_WIDTH)
   * @return Current property value
   */
  virtual double get(int prop_id) const = 0;

  /**
   * @brief Release camera resources
   */
  virtual void release() = 0;
};

#endif  // USB_CAMERA__CAMERA_INTERFACE_HPP_
