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

#ifndef VISION_UTILS__PROCESS_SCHEDULER_HPP_
#define VISION_UTILS__PROCESS_SCHEDULER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace vision_utils {

/**
 * @brief Utility class for CPU pinning and real-time scheduling
 */
class ProcessScheduler {
public:
  /**
   * @brief Apply CPU pinning and real-time FIFO scheduling to the current thread
   * 
   * @param pin_to_core CPU core to pin to (-1 to disable pinning)
   * @param priority Real-time scheduling priority (1-99, higher is more priority)
   * @param logger Optional ROS logger for status messages
   * @return true if successful, false otherwise
   */
  static bool applyCpuPinningAndScheduling(int pin_to_core, int priority, 
                                          rclcpp::Logger* logger = nullptr);

  /**
   * @brief Pin the current thread to a specific CPU core
   * 
   * @param core_id CPU core ID to pin to
   * @param logger Optional ROS logger for status messages
   * @return true if successful, false otherwise
   */
  static bool pinToCpuCore(int core_id, rclcpp::Logger* logger = nullptr);

  /**
   * @brief Set real-time FIFO scheduling for the current thread
   * 
   * @param priority Scheduling priority (1-99, higher is more priority)
   * @param logger Optional ROS logger for status messages
   * @return true if successful, false otherwise
   */
  static bool setRealTimeScheduling(int priority, rclcpp::Logger* logger = nullptr);

  /**
   * @brief Verify current thread's CPU affinity and scheduling policy
   * 
   * @param expected_core Expected CPU core (-1 to skip core verification)
   * @param expected_priority Expected scheduling priority (-1 to skip priority verification)
   * @param logger Optional ROS logger for status messages
   * @return true if settings match expectations, false otherwise
   */
  static bool verifySettings(int expected_core = -1, int expected_priority = -1,
                            rclcpp::Logger* logger = nullptr);

  /**
   * @brief Get the number of available CPU cores
   * 
   * @return Number of CPU cores available to the system
   */
  static int getAvailableCpuCores();

private:
  ProcessScheduler() = default;  // Static class, no instantiation
};

}  // namespace vision_utils

#endif  // VISION_UTILS__PROCESS_SCHEDULER_HPP_