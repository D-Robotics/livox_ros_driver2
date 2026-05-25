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

#include "auto_data_collector_node.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace auto_collect {

AutoDataCollectorNode::AutoDataCollectorNode() : Node("auto_data_collector_node") {
  // ================================================= Params ======================================================
  RCLCPP_WARN_STREAM(this->get_logger(), std::endl
                                             << "=> ===================== init " << this->get_name()
                                             << " =====================");
  output_dir_ = this->declare_parameter<std::string>("output_dir", "/userdata/lidar_img_data_collect");
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/livox/imu");
  lidar_topic_ = this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  image_topic_ = this->declare_parameter<std::string>("image_topic", "/husq_stereo_cam_node/image_combine_rgb");

  image_format_ = this->declare_parameter<std::string>("image_format", "jpg");
  image_collect_fps_ = this->declare_parameter<double>("image_collect_fps", 0.5);
  if (image_collect_fps_ <= 0) {
    image_collect_fps_ = 0.5;
  }
  image_collect_interval_ns_ = static_cast<uint64_t>(1e9 / image_collect_fps_);
  save_motion_image_ = this->declare_parameter<bool>("save_motion_image", true);

  target_pcd_count_ = this->declare_parameter<int>("target_pcd_count", 200);
  save_pcd_binary_ = this->declare_parameter<bool>("save_pcd_binary", true);
  gravity_ = this->declare_parameter<double>("gravity", 9.81);

  const int imu_window_size = this->declare_parameter<int>("imu_window_size", 200);
  const double accel_std_th = this->declare_parameter<double>("accel_std_th", 0.03);
  const double gyro_std_th = this->declare_parameter<double>("gyro_std_th", 0.01);
  const double max_imu_age_sec = this->declare_parameter<double>("max_imu_age_sec", 0.2);

  static_confirm_count_th_ = this->declare_parameter<int>("static_confirm_count_th", 10);
  motion_confirm_count_th_ = this->declare_parameter<int>("motion_confirm_count_th", 3);

  enable_image_motion_check_ = this->declare_parameter<bool>("enable_image_motion_check", true);
  const double image_diff_ratio_th = this->declare_parameter<double>("image_diff_ratio_th", 0.02);
  const int image_diff_gray_th = this->declare_parameter<int>("image_diff_gray_th", 25);

  save_data_flag_ = this->declare_parameter<bool>("save_data_flag", false);

  disk_usage_string_ = GetDiskUsageString();

  RCLCPP_WARN_STREAM(this->get_logger(),
                     std::endl
                         << "=> output_dir: " << output_dir_ << std::endl
                         << "=> imu_topic: " << imu_topic_ << std::endl
                         << "=> lidar_topic: " << lidar_topic_ << std::endl
                         << "=> image_topic: " << image_topic_ << std::endl
                         << "=> [image_collect_fps, image_format, save_motion_image]: [" << image_collect_fps_ << ", "
                         << image_format_ << ", " << save_motion_image_ << "]" << std::endl
                         << "=> [target_pcd_count, save_pcd_binary]: [" << target_pcd_count_ << ", " << save_pcd_binary_
                         << "]" << std::endl
                         << "=> [gravity, imu_window_size, accel_std_th, gyro_std_th, max_imu_age_sec]: [" << gravity_
                         << ", " << imu_window_size << ", " << accel_std_th << ", " << gyro_std_th << ", "
                         << max_imu_age_sec << "]" << std::endl
                         << "=> [static_confirm_count_th, motion_confirm_count_th]: [" << static_confirm_count_th_
                         << ", " << motion_confirm_count_th_ << "]" << std::endl
                         << "=> [enable_image_motion_check, image_diff_ratio_th, image_diff_gray_th]: ["
                         << enable_image_motion_check_ << ", " << image_diff_ratio_th << ", " << image_diff_gray_th
                         << "]" << std::endl
                         << "=> save_data_flag: " << save_data_flag_ << std::endl
                         << "=> " << disk_usage_string_ << std::endl
                         << "=> ==================================================================" << std::endl);

  param_callback_handle_ =
      this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
          if (param.get_name() == "save_data_flag") {
            save_data_flag_.store(param.as_bool());

            RCLCPP_WARN(this->get_logger(), "save_data_flag: %s", save_data_flag_.load() ? "true" : "false");

            if (!save_data_flag_.load()) {
              std::lock_guard<std::mutex> lock(mutex_);
              ResetCollection();
              state_ = State::WAIT_STATIC;
              static_confirm_count_ = 0;
              motion_confirm_count_ = 0;
            }
          }
        }

        return result;
      });

  // ================================================= Tools ========================================================
  imu_detector_ = std::make_shared<ImuMotionDetector>(static_cast<size_t>(imu_window_size), accel_std_th, gyro_std_th,
                                                      static_cast<uint64_t>(max_imu_age_sec * 1e9));
  image_motion_detector_ = std::make_shared<ImageMotionDetector>(image_diff_ratio_th, image_diff_gray_th);
  data_saver_ = std::make_shared<DataSaver>(output_dir_);

  // ================================================= Sub & Pub =====================================================
  imu_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pcd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_cb_group_;
  rclcpp::SubscriptionOptions image_options;
  image_options.callback_group = image_cb_group_;
  rclcpp::SubscriptionOptions pcd_options;
  pcd_options.callback_group = pcd_cb_group_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(imu_window_size));
  imu_qos.reliable();
  imu_qos.durability_volatile();

  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  image_qos.reliable();
  image_qos.durability_volatile();

  auto pcd_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pcd_qos.reliable();

  pcd_qos.durability_volatile();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos, std::bind(&AutoDataCollectorNode::ImuCallback, this, std::placeholders::_1), imu_options);
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos, std::bind(&AutoDataCollectorNode::ImageCallback, this, std::placeholders::_1),
      image_options);
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, pcd_qos, std::bind(&AutoDataCollectorNode::PcdCallback, this, std::placeholders::_1), pcd_options);
  status_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/status_image", 10);
}

uint64_t AutoDataCollectorNode::GetTimestampNs(const builtin_interfaces::msg::Time &stamp) const {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

const char *AutoDataCollectorNode::StateToString(State state) const {
  switch (state) {
  case State::WAIT_STATIC: return "WAIT_STATIC";
  case State::COLLECTING: return "COLLECTING";
  case State::WAIT_MOTION: return "WAIT_MOTION";
  default: return "UNKNOWN";
  }
}

void AutoDataCollectorNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Received imu from topic %s.", imu_topic_.c_str());
  if (!save_data_flag_.load()) {
    return;
  }

  ImuSample sample;
  sample.timestamp_ns = GetTimestampNs(msg->header.stamp);
  sample.wx = msg->angular_velocity.x;
  sample.wy = msg->angular_velocity.y;
  sample.wz = msg->angular_velocity.z;
  sample.ax = msg->linear_acceleration.x;
  sample.ay = msg->linear_acceleration.y;
  sample.az = msg->linear_acceleration.z;

  imu_detector_->Feed(sample);

  double accel_std = 0.0;
  double gyro_std = 0.0;
  const auto status = imu_detector_->GetStatus(sample.timestamp_ns, &accel_std, &gyro_std, nullptr);

  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ == State::COLLECTING && status != ImuMotionDetector::Status::STATIC) {
    AbortCollection("IMU motion detected during collection");
    return;
  }

  if (status == ImuMotionDetector::Status::STATIC) {
    ++static_confirm_count_;
    motion_confirm_count_ = 0;
  } else if (status == ImuMotionDetector::Status::MOTION) {
    ++motion_confirm_count_;
    static_confirm_count_ = 0;
  }

  if (state_ == State::WAIT_STATIC && static_confirm_count_ >= static_confirm_count_th_) {
    TryStartCollection(sample.timestamp_ns);
  }

  if (state_ == State::WAIT_MOTION && motion_confirm_count_ >= motion_confirm_count_th_) {
    RCLCPP_WARN(this->get_logger(), "Motion detected. Ready for next static collection.");
    state_ = State::WAIT_STATIC;
    static_confirm_count_ = 0;
    motion_confirm_count_ = 0;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "state=%s, accel_std=%.6f, gyro_std=%.6f, imgs=%zu, pcd=%zu", StateToString(state_), accel_std,
                       gyro_std, group_images_.size(), group_pcds_.size());
}

void AutoDataCollectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Received img from topic %s.", image_topic_.c_str());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishStatusImage(msg);
  }

  if (!save_data_flag_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  latest_image_ = msg;

  const uint64_t image_timestamp_ns = GetTimestampNs(msg->header.stamp);

  if (save_motion_image_ && state_ == State::WAIT_STATIC && group_id_ > 0) {
    if (last_motion_image_timestamp_ns_ == 0 ||
        image_timestamp_ns - last_motion_image_timestamp_ns_ >= image_collect_interval_ns_) {
      auto motion_image = std::make_shared<sensor_msgs::msg::Image>(*msg);

      if (data_saver_->SaveMotionImage(motion_image, image_format_)) {
        motion_image_count_++;
      }

      last_motion_image_timestamp_ns_ = image_timestamp_ns;
    }
  }

  if (state_ != State::COLLECTING) {
    return;
  }

  if (last_collect_image_timestamp_ns_ == 0 ||
      image_timestamp_ns - last_collect_image_timestamp_ns_ >= image_collect_interval_ns_) {
    group_images_.push_back(std::make_shared<sensor_msgs::msg::Image>(*msg));
    last_collect_image_timestamp_ns_ = image_timestamp_ns;
  }

  if (!enable_image_motion_check_) {
    return;
  }

  double diff_ratio = 0.0;
  if (image_motion_detector_->IsMoving(msg, &diff_ratio)) {
    AbortCollection("Moving object detected in image");
    RCLCPP_ERROR(this->get_logger(), "Image moving object detected. diff_ratio=%.6f", diff_ratio);
  }
}

void AutoDataCollectorNode::PcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Received pcd from topic %s.", lidar_topic_.c_str());
  if (!save_data_flag_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ != State::COLLECTING) {
    return;
  }

  group_pcds_.push_back(msg);

  if (static_cast<int>(group_pcds_.size()) >= target_pcd_count_) {
    FinishCollection();
  }
}

void AutoDataCollectorNode::TryStartCollection(uint64_t timestamp_ns) {
  if (!latest_image_) {
    RCLCPP_WARN(this->get_logger(), "Static detected, but no image received yet.");
    return;
  }

  group_images_.clear();
  group_pcds_.clear();
  last_collect_image_timestamp_ns_ = 0;

  auto first_image = std::make_shared<sensor_msgs::msg::Image>(*latest_image_);
  group_images_.push_back(first_image);
  last_collect_image_timestamp_ns_ = GetTimestampNs(first_image->header.stamp);

  if (enable_image_motion_check_) {
    if (!image_motion_detector_->SetReference(first_image)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set image motion reference.");
      ResetCollection();
      return;
    }
  }

  state_ = State::COLLECTING;

  RCLCPP_WARN(this->get_logger(), "Start collecting group %u at %.6f", group_id_,
              static_cast<double>(timestamp_ns) * 1e-9);
}

void AutoDataCollectorNode::AbortCollection(const std::string &reason) {
  RCLCPP_WARN(this->get_logger(), "Abort group %u. reason=%s, collected_pcd=%zu", group_id_, reason.c_str(),
              group_pcds_.size());

  ResetCollection();

  state_ = State::WAIT_MOTION;
  static_confirm_count_ = 0;
  motion_confirm_count_ = 0;
}

void AutoDataCollectorNode::FinishCollection() {
  RCLCPP_WARN(this->get_logger(), "Finish group %u. image_count=%zu, pcd_count=%zu", group_id_, group_images_.size(),
              group_pcds_.size());

  const bool ok = data_saver_->SaveGroup(group_id_, group_images_, group_pcds_, save_pcd_binary_, image_format_);

  if (ok) {
    RCLCPP_WARN(this->get_logger(), "Save group %u success.", group_id_);
    ++group_id_;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Save group %u failed.", group_id_);
  }

  ResetCollection();

  state_ = State::WAIT_MOTION;
  static_confirm_count_ = 0;
  motion_confirm_count_ = 0;
}

void AutoDataCollectorNode::ResetCollection() {
  group_images_.clear();
  group_pcds_.clear();
  last_collect_image_timestamp_ns_ = 0;
  image_motion_detector_->Reset();
}

void AutoDataCollectorNode::PublishStatusImage(const sensor_msgs::msg::Image::SharedPtr &msg) {
  if (!msg || status_image_pub_->get_subscription_count() == 0) {
    return;
  }

  cv::Mat bgr;

  if (msg->encoding == "bgr8") {
    bgr = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data())).clone();
  } else if (msg->encoding == "rgb8") {
    cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  } else {
    cv::Mat nv12(msg->height * 3 / 2, msg->width, CV_8UC1, const_cast<uint8_t *>(msg->data.data()));
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
  }

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

  std::vector<std::string> lines;
  lines.push_back(std::string("STATE: ") + StateToString(state_));
  if (!save_data_flag_.load()) {
    lines.push_back("SAVE: DISABLED");
  } else {
    lines.push_back("SAVE: ENABLED");
  }
  if (state_ == State::WAIT_STATIC) {
    lines.push_back("MOTION IMG: " + std::to_string(motion_image_count_.load()));
  } else if (state_ == State::COLLECTING) {
    lines.push_back("GROUP IMG: " + std::to_string(group_images_.size()) +
                    "  PCD: " + std::to_string(group_pcds_.size()) + "/" + std::to_string(target_pcd_count_));
    std::stringstream ss;
    ss << "group_" << std::setfill('0') << std::setw(6) << group_id_;
    lines.push_back("GROUP ID: " + ss.str());
  } else {
    lines.push_back("WAIT NEXT MOTION");
  }
  lines.push_back("SAVE DIR: " + data_saver_->GetSessionDir());
  lines.push_back(disk_usage_string_);
  const int x = 20;
  const int y = 30;
  const double font_scale = 0.8;
  const int thickness = 2;
  const int line_gap = 12;
  const int padding = 12;
  int baseline = 0;
  int max_text_width = 0;
  int total_text_height = 0;
  for (const auto &line : lines) {
    const auto text_size = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
    max_text_width = std::max(max_text_width, text_size.width);
    total_text_height += text_size.height + line_gap;
  }
  const int box_w = max_text_width + padding * 2;
  const int box_h = total_text_height + padding * 2;
  cv::Mat overlay = resized.clone();
  cv::rectangle(overlay, cv::Rect(x, y, box_w, box_h), cv::Scalar(0, 0, 0), cv::FILLED);
  cv::addWeighted(overlay, 0.6, resized, 0.4, 0.0, resized);
  int text_y = y + padding + 24;
  for (const auto &line : lines) {
    cv::putText(resized, line, cv::Point(x + padding, text_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(255, 255, 255), thickness);
    text_y += 32;
  }

  auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
  out_msg->header = msg->header;
  out_msg->height = resized.rows;
  out_msg->width = resized.cols;
  out_msg->encoding = "bgr8";
  out_msg->is_bigendian = false;
  out_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(resized.cols * resized.elemSize());
  out_msg->data.assign(resized.data, resized.data + resized.total() * resized.elemSize());

  status_image_pub_->publish(std::move(out_msg));
}

std::string AutoDataCollectorNode::GetDiskUsageString() const {
  struct statvfs stat{};

  if (statvfs(output_dir_.c_str(), &stat) != 0) {
    return "DISK: UNKNOWN";
  }

  const uint64_t total = static_cast<uint64_t>(stat.f_blocks) * stat.f_frsize;
  const uint64_t available = static_cast<uint64_t>(stat.f_bavail) * stat.f_frsize;
  const uint64_t used = total - available;
  const double total_gb = static_cast<double>(total) / 1024.0 / 1024.0 / 1024.0;
  const double used_gb = static_cast<double>(used) / 1024.0 / 1024.0 / 1024.0;
  const double percent = total > 0 ? static_cast<double>(used) * 100.0 / total : 0.0;

  std::stringstream ss;
  ss << std::fixed << std::setprecision(1) << "DISK: " << used_gb << "G / " << total_gb << "G (" << percent << "%)";
  return ss.str();
}

} // namespace auto_collect