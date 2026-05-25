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

#include "imu_motion_detector.h"
#include <cmath>

namespace auto_collect {

ImuMotionDetector::ImuMotionDetector(size_t window_size, double accel_std_th, double gyro_std_th,
                                     uint64_t max_imu_age_ns)
    : window_size_(window_size), accel_std_th_(accel_std_th), gyro_std_th_(gyro_std_th),
      max_imu_age_ns_(max_imu_age_ns) {
}

void ImuMotionDetector::Feed(const ImuSample &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_buffer_.push_back(sample);

  while (imu_buffer_.size() > max_buffer_size_) {
    imu_buffer_.pop_front();
  }
}

bool ImuMotionDetector::BuildWindow(uint64_t target_timestamp_ns, std::deque<ImuSample> *window) {
  if (window == nullptr) {
    return false;
  }

  window->clear();

  std::lock_guard<std::mutex> lock(mutex_);

  if (imu_buffer_.size() < window_size_) {
    return false;
  }

  for (auto it = imu_buffer_.rbegin(); it != imu_buffer_.rend(); ++it) {
    if (it->timestamp_ns <= target_timestamp_ns) {
      window->push_front(*it);
    }

    if (window->size() >= window_size_) {
      break;
    }
  }

  return window->size() >= window_size_;
}

ImuMotionDetector::Status ImuMotionDetector::GetStatus(uint64_t target_timestamp_ns, double *accel_std,
                                                       double *gyro_std, uint64_t *last_imu_timestamp_ns) {
  std::deque<ImuSample> window;

  if (!BuildWindow(target_timestamp_ns, &window)) {
    return Status::INIT;
  }

  const uint64_t latest_imu_timestamp_ns = window.back().timestamp_ns;

  if (last_imu_timestamp_ns != nullptr) {
    *last_imu_timestamp_ns = latest_imu_timestamp_ns;
  }

  if (target_timestamp_ns > latest_imu_timestamp_ns &&
      target_timestamp_ns - latest_imu_timestamp_ns > max_imu_age_ns_) {
    return Status::IMU_TOO_OLD;
  }

  double ax_mean = 0.0;
  double ay_mean = 0.0;
  double az_mean = 0.0;
  double wx_mean = 0.0;
  double wy_mean = 0.0;
  double wz_mean = 0.0;

  for (const auto &sample : window) {
    ax_mean += sample.ax;
    ay_mean += sample.ay;
    az_mean += sample.az;
    wx_mean += sample.wx;
    wy_mean += sample.wy;
    wz_mean += sample.wz;
  }

  const double n = static_cast<double>(window.size());

  ax_mean /= n;
  ay_mean /= n;
  az_mean /= n;
  wx_mean /= n;
  wy_mean /= n;
  wz_mean /= n;

  double accel_var_sum = 0.0;
  double gyro_var_sum = 0.0;

  for (const auto &sample : window) {
    const double dax = sample.ax - ax_mean;
    const double day = sample.ay - ay_mean;
    const double daz = sample.az - az_mean;
    const double dwx = sample.wx - wx_mean;
    const double dwy = sample.wy - wy_mean;
    const double dwz = sample.wz - wz_mean;

    accel_var_sum += dax * dax + day * day + daz * daz;
    gyro_var_sum += dwx * dwx + dwy * dwy + dwz * dwz;
  }

  const double denom = static_cast<double>(window.size() - 1);
  const double accel_std_value = std::sqrt(accel_var_sum / denom);
  const double gyro_std_value = std::sqrt(gyro_var_sum / denom);

  if (accel_std != nullptr) {
    *accel_std = accel_std_value;
  }

  if (gyro_std != nullptr) {
    *gyro_std = gyro_std_value;
  }

  if (accel_std_value < accel_std_th_ && gyro_std_value < gyro_std_th_) {
    return Status::STATIC;
  }

  return Status::MOTION;
}

} // namespace auto_collect