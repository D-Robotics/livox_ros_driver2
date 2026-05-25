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

#include "data_saver.h"
#include "image_motion_detector.h"
#include "imu_motion_detector.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <mutex>
#include <vector>
#include <sys/statvfs.h>

namespace auto_collect {

class AutoDataCollectorNode : public rclcpp::Node {
public:
  AutoDataCollectorNode();

private:
  /**
   * @brief The state of the data collection
   */
  enum class State { WAIT_STATIC, COLLECTING, WAIT_MOTION };

  /**
   * @brief Callback function for IMU topic
   * @param msg The IMU message
   */
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  /**
   * @brief Callback function for image topic
   * @param msg The image message
   */
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  /**
   * @brief Callback function for point cloud topic
   * @param msg The point cloud message
   */
  void PcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Try to start a new collection
   * @param timestamp_ns The timestamp of the first image
   */
  void TryStartCollection(uint64_t timestamp_ns);
  /**
   * @brief Abort the current collection
   * @param reason The reason for aborting
   */
  void AbortCollection(const std::string &reason);
  /**
   * @brief Finish the current collection
   */
  void FinishCollection();
  /**
   * @brief Reset the current collection
   */
  void ResetCollection();

  /**
   * @brief Get the timestamp in nanoseconds
   * @param stamp The timestamp message
   * @return The timestamp in nanoseconds
   */
  uint64_t GetTimestampNs(const builtin_interfaces::msg::Time &stamp) const;
  /**
   * @brief Get the state as a string
   * @param state The state
   * @return The state as a string
   */
  const char *StateToString(State state) const;

  /**
   * @brief Publish the status image
   * @param msg The status image message
   */
  void PublishStatusImage(const sensor_msgs::msg::Image::SharedPtr &msg);

  /**
   * @brief Get the disk usage string
   * @return The disk usage string
   */
  std::string GetDiskUsageString() const;

private:
  std::mutex mutex_;

  State state_{State::WAIT_STATIC};

  std::shared_ptr<ImuMotionDetector> imu_detector_;
  std::shared_ptr<ImageMotionDetector> image_motion_detector_;
  std::shared_ptr<DataSaver> data_saver_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr pcd_cb_group_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr status_image_pub_;

  sensor_msgs::msg::Image::SharedPtr latest_image_;
  std::vector<sensor_msgs::msg::Image::SharedPtr> group_images_;
  double image_collect_fps_{0.5};
  uint64_t image_collect_interval_ns_{500000000};
  uint64_t last_collect_image_timestamp_ns_{0};
  bool save_motion_image_{true};
  uint64_t last_motion_image_timestamp_ns_{0};
  std::atomic_uint32_t motion_image_count_{0};
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> group_pcds_;

  uint32_t group_id_{0};

  std::string image_topic_;
  std::string imu_topic_;
  std::string lidar_topic_;
  std::string output_dir_;
  std::string image_format_{"jpg"};

  int target_pcd_count_{200};
  int static_confirm_count_th_{10};
  int motion_confirm_count_th_{3};

  int static_confirm_count_{0};
  int motion_confirm_count_{0};

  bool enable_image_motion_check_{true};

  double gravity_{9.81};
  bool save_pcd_binary_{true};

  std::atomic_bool save_data_flag_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string disk_usage_string_;
};

} // namespace auto_collect