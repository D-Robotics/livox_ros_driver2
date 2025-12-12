// Copyright (c) 2025，D-Robotics.
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

struct ImuData {
  explicit ImuData(uint64_t ts) : timestamp(ts) {}
  uint64_t timestamp;
  double ax{}, ay{}, az{}, wx{}, wy{}, wz{};
};

struct MotionDetector {
  enum Status {
    INIT,
    STATIC,
    MOTION,
    ENTERING_STATIC,
    ENTERING_MOTION,
    IMU_TOO_FAR
  };

  explicit MotionDetector(uint32_t motion_window_size,
      double a_th, double w_th) : window_size_(motion_window_size), a_th_(a_th), w_th_(w_th), status_(INIT) {

  }

  void FeedImu(const ImuData &imu_data) {
    std::lock_guard<std::mutex> lck(mtx_);
    imu_buffer_.push_back(imu_data);
    if (imu_buffer_.size() > 4000) {
      imu_buffer_.pop_front();
    }
  }

  enum Status GetStatus(
      uint64_t timestamp, uint64_t &last_imu_timestamp,
      double &a_var, double &w_var) {
    std::deque<ImuData> imu_window;
    std::vector<double> a_avg{0, 0, 0}, w_avg{0, 0, 0};
    {
      std::lock_guard<std::mutex> lck(mtx_);
      if (imu_buffer_.size() < window_size_) return status_;
      auto i = imu_buffer_.rbegin();
      for (;i != imu_buffer_.rend(); ++i) {
        if (i->timestamp < timestamp) {
          imu_window.push_front(*i);
        }
        if (imu_window.size() >= window_size_) {
          break;
        }
      }
    }

    if (imu_window.size() < window_size_) {
      status_ = IMU_TOO_FAR;
      return status_;
    }

    last_imu_timestamp = imu_window.front().timestamp;

    a_var = 0;
    w_var = 0;

    for (const auto &data : imu_window) {
      a_avg[0] += data.ax;
      a_avg[1] += data.ay;
      a_avg[2] += data.az;
      w_avg[0] += data.wx;
      w_avg[1] += data.wy;
      w_avg[2] += data.wz;
    }

    for (int i = 0; i < 3; ++i) {
      a_avg[i] /= imu_window.size();
      w_avg[i] /= imu_window.size();
    }

    for (const auto &data : imu_window) {
      a_var += ((data.ax - a_avg[0]) * (data.ax - a_avg[0]) + (data.ay - a_avg[1]) * (data.ay - a_avg[1]) + (data.az - a_avg[2]) * (data.az - a_avg[2]));
      w_var += ((data.wx - w_avg[0]) * (data.wx - w_avg[0]) + (data.wy - w_avg[1]) * (data.wy - w_avg[1]) + (data.wz - w_avg[2]) * (data.wz - w_avg[2]));
    }
    a_var = std::sqrt(a_var / (imu_window.size() - 1));
    w_var = std::sqrt(w_var / (imu_window.size() - 1));
    bool jark = a_var > a_th_ && w_var > w_th_;
    if (jark) {
      motion_count_++;
      if (motion_count_ >= 2) motion_count_ = 2;
      switch (status_) {
        case ENTERING_MOTION:
        case MOTION:
          status_ = MOTION;
          break;
        default:
          status_ = ENTERING_MOTION;
          enter_motion_timestamp_ = last_timestamp_;
          break;
      }
    } else {
      motion_count_--;
      if (motion_count_ <= -2) motion_count_ = -2;
      switch (status_) {
        case INIT:
        case ENTERING_MOTION:
        case MOTION:
          if (motion_count_ < 0) {
            status_ = ENTERING_STATIC;
            enter_static_timestamp_ = timestamp;
          }
          break;
        default:
          status_ = STATIC;
          break;
      }
    }

    while (!imu_buffer_.empty() && imu_buffer_.front().timestamp < last_timestamp_) {
      imu_buffer_.pop_front();
    }
    last_timestamp_ = timestamp;
    last_a_var_ = a_var;
    last_w_var_ = w_var;

    return status_;
  }

  uint64_t GetEnterStaticTimeStamp() {
    return enter_static_timestamp_;
  }

  uint64_t GetEnterMotionTimeStamp() {
    return enter_motion_timestamp_;
  }

 private:
  uint32_t window_size_;
  int motion_count_{0};
  double last_a_var_{0.0}, last_w_var_{0};
  double a_th_{1.1}, w_th_{0.0};
  uint64_t last_timestamp_{0}, enter_static_timestamp_{0}, enter_motion_timestamp_{0};
  std::mutex mtx_;
  enum Status status_{INIT};
  std::deque<ImuData> imu_buffer_;
};
