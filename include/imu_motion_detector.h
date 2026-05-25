// Copyright (c) 2026，D-Robotics.
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

#pragma once

#include <cstdint>
#include <deque>
#include <mutex>

namespace auto_collect {

struct ImuSample {
  uint64_t timestamp_ns{0};
  double ax{0.0};
  double ay{0.0};
  double az{0.0};
  double wx{0.0};
  double wy{0.0};
  double wz{0.0};
};

class ImuMotionDetector {
public:
  /**
   * @brief The status of the IMU motion detector
   * INIT: the IMU motion detector is not initialized
   * STATIC: the IMU motion detector is in static state
   * MOTION: the IMU motion detector is in motion state
   * IMU_TOO_OLD: the IMU motion detector is too old
   */
  enum class Status { INIT, STATIC, MOTION, IMU_TOO_OLD };

  /**
   * @brief Construct a new ImuMotionDetector object
   * @param window_size The size of the IMU window
   * @param accel_std_th The threshold of the acceleration standard deviation
   * @param gyro_std_th The threshold of the gyroscope standard deviation
   * @param max_imu_age_ns The maximum age of the IMU samples
   */
  ImuMotionDetector(size_t window_size, double accel_std_th, double gyro_std_th, uint64_t max_imu_age_ns);

  /**
   * @brief Feed the IMU samples
   * @param sample The IMU sample
   */
  void Feed(const ImuSample &sample);

  /**
   * @brief Get the status of the IMU motion detector
   * @param target_timestamp_ns The target timestamp
   * @param accel_std The acceleration standard deviation
   * @param gyro_std The gyroscope standard deviation
   * @param last_imu_timestamp_ns The last IMU timestamp
   * @return The status of the IMU motion detector
   */
  Status GetStatus(uint64_t target_timestamp_ns, double *accel_std = nullptr, double *gyro_std = nullptr,
                   uint64_t *last_imu_timestamp_ns = nullptr);

private:
  /**
   * @brief Build the IMU window
   * @param target_timestamp_ns The target timestamp
   * @param window The IMU window
   * @return True if the window is built successfully, false otherwise
   */
  bool BuildWindow(uint64_t target_timestamp_ns, std::deque<ImuSample> *window);

private:
  size_t window_size_{200};
  double accel_std_th_{0.03};
  double gyro_std_th_{0.01};
  uint64_t max_imu_age_ns_{200000000};

  std::mutex mutex_;
  std::deque<ImuSample> imu_buffer_;
  size_t max_buffer_size_{4000};
};

} // namespace auto_collect