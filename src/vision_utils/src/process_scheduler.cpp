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

#include "vision_utils/process_scheduler.hpp"
#include <sched.h>
#include <pthread.h>
#include <unistd.h>
#include <cstring>

namespace vision_utils {

bool ProcessScheduler::applyCpuPinningAndScheduling(int pin_to_core, int priority, 
                                                   rclcpp::Logger* logger) {
  if (pin_to_core == -1) {
    if (logger) {
      RCLCPP_INFO(*logger, "CPU pinning disabled (pin_to_core = -1)");
    }
    return true;
  }

  bool success = true;
  
  // Apply CPU pinning
  if (!pinToCpuCore(pin_to_core, logger)) {
    success = false;
  }
  
  // Apply real-time scheduling
  if (!setRealTimeScheduling(priority, logger)) {
    success = false;
  }
  
  // Verify settings if both operations were attempted
  if (success) {
    verifySettings(pin_to_core, priority, logger);
  }
  
  return success;
}

bool ProcessScheduler::pinToCpuCore(int core_id, rclcpp::Logger* logger) {
  // Validate core ID
  int num_cores = getAvailableCpuCores();
  if (core_id < 0 || core_id >= num_cores) {
    if (logger) {
      RCLCPP_ERROR(*logger, "Invalid CPU core %d. System has %d cores (0-%d)", 
                   core_id, num_cores, num_cores - 1);
    }
    return false;
  }

  // Get current thread ID
  pthread_t current_thread = pthread_self();
  
  // Set CPU affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  
  int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
  if (result != 0) {
    if (logger) {
      RCLCPP_ERROR(*logger, "Failed to set CPU affinity to core %d: %s", 
                   core_id, strerror(result));
    }
    return false;
  }
  
  if (logger) {
    RCLCPP_INFO(*logger, "Successfully pinned process to CPU core %d", core_id);
  }
  
  return true;
}

bool ProcessScheduler::setRealTimeScheduling(int priority, rclcpp::Logger* logger) {
  // Validate priority range
  int min_priority = sched_get_priority_min(SCHED_FIFO);
  int max_priority = sched_get_priority_max(SCHED_FIFO);
  
  if (priority < min_priority || priority > max_priority) {
    if (logger) {
      RCLCPP_ERROR(*logger, "Invalid priority %d. Valid range for SCHED_FIFO: %d-%d", 
                   priority, min_priority, max_priority);
    }
    return false;
  }

  // Get current thread ID
  pthread_t current_thread = pthread_self();
  
  // Set real-time scheduling policy (FIFO)
  struct sched_param param;
  param.sched_priority = priority;
  
  int result = pthread_setschedparam(current_thread, SCHED_FIFO, &param);
  if (result != 0) {
    if (logger) {
      RCLCPP_ERROR(*logger, "Failed to set real-time scheduling priority %d: %s", 
                   priority, strerror(result));
      RCLCPP_WARN(*logger, "Note: Real-time scheduling requires root privileges or CAP_SYS_NICE capability");
    }
    return false;
  }
  
  if (logger) {
    RCLCPP_INFO(*logger, "Successfully set real-time FIFO scheduling with priority %d", priority);
  }
  
  return true;
}

bool ProcessScheduler::verifySettings(int expected_core, int expected_priority,
                                     rclcpp::Logger* logger) {
  pthread_t current_thread = pthread_self();
  bool success = true;
  
  // Verify CPU affinity if requested
  if (expected_core >= 0) {
    cpu_set_t verify_cpuset;
    CPU_ZERO(&verify_cpuset);
    int result = pthread_getaffinity_np(current_thread, sizeof(cpu_set_t), &verify_cpuset);
    
    if (result == 0 && CPU_ISSET(expected_core, &verify_cpuset)) {
      if (logger) {
        RCLCPP_INFO(*logger, "CPU affinity verification: Successfully pinned to core %d", expected_core);
      }
    } else {
      if (logger) {
        RCLCPP_WARN(*logger, "CPU affinity verification failed for core %d", expected_core);
      }
      success = false;
    }
  }
  
  // Verify scheduling policy and priority if requested
  if (expected_priority >= 0) {
    int policy;
    struct sched_param verify_param;
    int result = pthread_getschedparam(current_thread, &policy, &verify_param);
    
    if (result == 0 && policy == SCHED_FIFO && verify_param.sched_priority == expected_priority) {
      if (logger) {
        RCLCPP_INFO(*logger, "Scheduling verification: FIFO policy with priority %d", expected_priority);
      }
    } else {
      if (logger) {
        if (result != 0) {
          RCLCPP_WARN(*logger, "Failed to get scheduling parameters: %s", strerror(result));
        } else {
          RCLCPP_WARN(*logger, "Scheduling verification failed. Expected: FIFO/%d, Actual: %d/%d", 
                      expected_priority, policy, verify_param.sched_priority);
        }
      }
      success = false;
    }
  }
  
  return success;
}

int ProcessScheduler::getAvailableCpuCores() {
  return static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
}

}  // namespace vision_utils