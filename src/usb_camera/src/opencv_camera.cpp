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

#include "usb_camera/opencv_camera.hpp"

OpenCVCamera::~OpenCVCamera()
{
  release();
}

bool OpenCVCamera::open(int device_id, int api_preference)
{
  return cap_.open(device_id, api_preference);
}

bool OpenCVCamera::isOpened() const
{
  return cap_.isOpened();
}

bool OpenCVCamera::read(cv::Mat & frame)
{
  return cap_.read(frame);
}

bool OpenCVCamera::set(int prop_id, double value)
{
  return cap_.set(prop_id, value);
}

double OpenCVCamera::get(int prop_id) const
{
  return cap_.get(prop_id);
}

void OpenCVCamera::release()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
}
