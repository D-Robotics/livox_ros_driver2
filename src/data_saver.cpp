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

#include "data_saver.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip>

namespace auto_collect {

DataSaver::DataSaver(const std::string &root_dir) : root_dir_(root_dir) {
  std::filesystem::create_directories(root_dir_);
  session_dir_ = root_dir_ / GenerateSessionName();
  std::filesystem::create_directories(session_dir_);
  std::cout << "session_dir: " << session_dir_ << std::endl << std::endl;
}

uint64_t DataSaver::GetTimestampNs(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

bool DataSaver::SaveGroup(uint32_t group_id, const std::vector<sensor_msgs::msg::Image::SharedPtr> &image_msgs,
                          const std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> &pcd_msgs, bool save_pcd_binary,
                          const std::string &image_format) {
  std::stringstream group_name_ss;
  group_name_ss << "group_" << std::setfill('0') << std::setw(6) << group_id;
  const auto group_dir = session_dir_ / group_name_ss.str();
  const auto image_dir = group_dir / "image";
  const auto pcd_dir = group_dir / "pcd";

  std::filesystem::create_directories(image_dir);
  std::filesystem::create_directories(pcd_dir);

  for (const auto &image_msg : image_msgs) {
    if (!image_msg) {
      continue;
    }

    const uint64_t image_ts = GetTimestampNs(image_msg->header.stamp);
    const auto image_path = image_dir / (std::to_string(image_ts) + "." + image_format);

    if (!SaveImage(image_path, image_msg, image_format)) {
      return false;
    }
  }

  for (const auto &pcd_msg : pcd_msgs) {
    if (!pcd_msg) {
      continue;
    }

    const uint64_t pcd_ts = GetTimestampNs(pcd_msg->header.stamp);
    SavePcd(pcd_dir / (std::to_string(pcd_ts) + ".pcd"), pcd_msg, save_pcd_binary);
  }

  std::ofstream info_file(group_dir / "group.info", std::ios::out);
  if (info_file.is_open()) {
    info_file << "group_id: " << group_id << "\n";
    info_file << "uname: " << RunCommand("uname -a") << "\n";
    info_file << "eth0_ip: " << GetEth0Ip() << "\n";
    info_file << "image_count: " << image_msgs.size() << "\n";
    info_file << "pcd_count: " << pcd_msgs.size() << "\n";

    if (!image_msgs.empty()) {
      info_file << "first_image_timestamp_ns: " << GetTimestampNs(image_msgs.front()->header.stamp) << "\n";
      info_file << "last_image_timestamp_ns: " << GetTimestampNs(image_msgs.back()->header.stamp) << "\n";
    }

    if (!pcd_msgs.empty()) {
      info_file << "first_pcd_timestamp_ns: " << GetTimestampNs(pcd_msgs.front()->header.stamp) << "\n";
      info_file << "last_pcd_timestamp_ns: " << GetTimestampNs(pcd_msgs.back()->header.stamp) << "\n";
    }
  }

  return true;
}

bool DataSaver::SaveImage(const std::filesystem::path &path, const sensor_msgs::msg::Image::SharedPtr &msg,
                          const std::string &image_format) {
  if (!msg || msg->data.empty()) {
    return false;
  }

  if (image_format == "yuv") {
    std::ofstream file(path, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      return false;
    }

    file.write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
    return true;
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

  return cv::imwrite(path.string(), bgr);
}

bool DataSaver::SavePcd(const std::filesystem::path &path, const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                        bool save_binary) {
  if (!msg) {
    return false;
  }

  const int point_count = static_cast<int>(msg->width * msg->height);

  bool has_offset_time = false;
  bool has_timestamp = false;

  for (const auto &field : msg->fields) {
    if (field.name == "offset_time") {
      has_offset_time = true;
    } else if (field.name == "timestamp") {
      has_timestamp = true;
    }
  }

  if (!has_offset_time && !has_timestamp) {
    return false;
  }

  std::ofstream file;
  if (save_binary) {
    file.open(path, std::ios::out | std::ios::binary);
  } else {
    file.open(path, std::ios::out);
  }

  if (!file.is_open()) {
    return false;
  }

  file << "# .PCD v0.7 - Point Cloud Data file format\n";
  file << "VERSION 0.7\n";
  if (has_offset_time) {
    file << "FIELDS x y z intensity tag line offset_time\n";
    file << "SIZE 4 4 4 4 1 1 4\n";
    file << "TYPE F F F F U U U\n";
  } else {
    file << "FIELDS x y z intensity tag line timestamp\n";
    file << "SIZE 4 4 4 4 1 1 8\n";
    file << "TYPE F F F F U U F\n";
  }
  file << "COUNT 1 1 1 1 1 1 1\n";
  file << "WIDTH " << point_count << "\n";
  file << "HEIGHT 1\n";
  file << "VIEWPOINT 0 0 0 1 0 0 0\n";
  file << "POINTS " << point_count << "\n";

  if (save_binary) {
    file << "DATA binary\n";
    file.write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
    return true;
  }

  file << "DATA ascii\n";

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_tag(*msg, "tag");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_line(*msg, "line");

  if (has_offset_time) {
    sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_offset_time(*msg, "offset_time");
    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag, ++iter_line, ++iter_offset_time) {
      file << *iter_x << " " << *iter_y << " " << *iter_z << " " << *iter_intensity << " "
           << static_cast<int>(*iter_tag) << " " << static_cast<int>(*iter_line) << " " << *iter_offset_time << "\n";
    }
  } else {
    sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp(*msg, "timestamp");
    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag, ++iter_line, ++iter_timestamp) {
      file << *iter_x << " " << *iter_y << " " << *iter_z << " " << *iter_intensity << " "
           << static_cast<int>(*iter_tag) << " " << static_cast<int>(*iter_line) << " " << std::fixed
           << std::setprecision(9) << *iter_timestamp << "\n";
    }
  }

  return true;
}

std::string DataSaver::GenerateSessionName() {
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now;

#ifdef _WIN32
  localtime_s(&tm_now, &now_time_t);
#else
  localtime_r(&now_time_t, &tm_now);
#endif

  std::stringstream ss;
  ss << std::setfill('0') << std::setw(4) << (tm_now.tm_year + 1900) << "_" << std::setw(2) << (tm_now.tm_mon + 1)
     << "_" << std::setw(2) << tm_now.tm_mday << "_" << std::setw(2) << tm_now.tm_hour << "_" << std::setw(2)
     << tm_now.tm_min << "_" << std::setw(2) << tm_now.tm_sec;
  return ss.str();
}

std::string DataSaver::RunCommand(const std::string &cmd) {
  std::array<char, 256> buffer{};
  std::string result;

  FILE *pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    return "";
  }

  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }

  pclose(pipe);

  while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
    result.pop_back();
  }

  return result;
}

std::string DataSaver::GetEth0Ip() {
  return RunCommand("ifconfig eth0 | grep 'inet ' | awk '{print $2}'");
}

bool DataSaver::SaveMotionImage(const sensor_msgs::msg::Image::SharedPtr &image_msg, const std::string &image_format) {
  if (!image_msg) {
    return false;
  }

  const auto motion_dir = session_dir_ / "motion";
  std::filesystem::create_directories(motion_dir);

  const uint64_t image_ts = GetTimestampNs(image_msg->header.stamp);
  const auto image_path = motion_dir / (std::to_string(image_ts) + "." + image_format);

  return SaveImage(image_path, image_msg, image_format);
}

} // namespace auto_collect