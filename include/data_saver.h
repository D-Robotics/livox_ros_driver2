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

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <array>
#include <cstdio>

namespace auto_collect {

class DataSaver {
public:
  /**
   * @brief Construct a new DataSaver object
   * @param root_dir The root directory to save data
   */
  explicit DataSaver(const std::string &root_dir);

  /**
   * @brief Save the group of data
   * @param group_id The group ID
   * @param image_msgs The image messages
   * @param pcd_msgs The point cloud messages
   * @param save_pcd_binary Whether to save the point cloud data in binary format
   * @param image_format The image format
   * @return True if the data is saved successfully, false otherwise
   */
  bool SaveGroup(uint32_t group_id, const std::vector<sensor_msgs::msg::Image::SharedPtr> &image_msgs,
                 const std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> &pcd_msgs, bool save_pcd_binary,
                 const std::string &image_format);

  /**
   * @brief Save the motion image
   * @param image_msg The image message
   * @param image_format The image format
   * @return True if the image is saved successfully, false otherwise
   */
  bool SaveMotionImage(const sensor_msgs::msg::Image::SharedPtr &image_msg, const std::string &image_format);

  /**
   * @brief Get the session dir
   * @return The session dir
   */
  std::string GetSessionDir() const {
    return session_dir_.parent_path().filename().string() + "/" + session_dir_.filename().string();
  }

private:
  /**
   * @brief Save the image
   * @param path The path to save the image
   * @param msg The image message
   * @param image_format The image format
   * @return True if the image is saved successfully, false otherwise
   */
  bool SaveImage(const std::filesystem::path &path, const sensor_msgs::msg::Image::SharedPtr &msg,
                 const std::string &image_format);

  /**
   * @brief Save the point cloud
   * @param path The path to save the point cloud
   * @param msg The point cloud message
   * @param save_binary Whether to save the point cloud data in binary format
   * @return True if the point cloud is saved successfully, false otherwise
   */
  bool SavePcd(const std::filesystem::path &path, const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
               bool save_binary);

  /**
   * @brief Get the timestamp in nanoseconds
   * @param stamp The timestamp message
   * @return The timestamp in nanoseconds
   */
  uint64_t GetTimestampNs(const builtin_interfaces::msg::Time &stamp);

  /**
   * @brief Generate a session name
   * @return The session name
   */
  std::string GenerateSessionName();

  /**
   * @brief Run a command
   * @param cmd The command
   * @return The output of the command
   */
  std::string RunCommand(const std::string &cmd);

  /**
   * @brief Get the IP address of the eth0 interface
   * @return The IP address of the eth0 interface
   */
  std::string GetEth0Ip();

private:
  std::filesystem::path root_dir_;

  std::filesystem::path session_dir_;
};

} // namespace auto_collect