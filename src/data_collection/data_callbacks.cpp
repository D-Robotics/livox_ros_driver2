#include "data_collection_node.h"

#include "utils.h"

#include <cstdint>
#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>

void ROS2DataCollection::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!imu_file_.is_open()) return;

  uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

  // Write IMU data to file
  imu_file_ << timestamp << ","
            << msg->angular_velocity.x<< "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
            << msg->linear_acceleration.x * saver_confg_.gravity
            << "," << msg->linear_acceleration.y * saver_confg_.gravity
            << "," << msg->linear_acceleration.z * saver_confg_.gravity
      << std::endl;

  // Feed IMU to DataSaver for motion detection
  ImuData imu_data(timestamp);
  imu_data.wx = msg->angular_velocity.x;
  imu_data.wy = msg->angular_velocity.y;
  imu_data.wz = msg->angular_velocity.z;
  imu_data.ax = msg->linear_acceleration.x * saver_confg_.gravity ;
  imu_data.ay = msg->linear_acceleration.y * saver_confg_.gravity ;
  imu_data.az = msg->linear_acceleration.z * saver_confg_.gravity ;
  data_saver_->FeedImu(timestamp, imu_data);

  imu_file_.flush();
}

void ROS2DataCollection::image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
  static uint64_t last_timestamp = 0, last_save_timestamp = 0;
  static uint64_t lost_cnt = 0;
  uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

  if (!rclcpp::ok()) return;

  RCLCPP_INFO(this->get_logger(),
              "get image at %fs, msg ts is: %fs, diff: %fms",
              this->now().seconds(), timestamp * 1e-9,
              this->now().seconds() * 1e3 - timestamp * 1e-6);

  std::cout << "\rwe have saved image: "
            << data_saver_->GetImageSaveCount() << ", pcd: " << data_saver_->GetPcdSaveCount()
            << ", imu: " << data_saver_->GetImuSaveCount() << std::flush;

  // Check camera sync
  bool camera_sync = !check_camera_sync_ || (msg->header.stamp.nanosec / 1000000) % 100 == 0;

  // Render status overlay
  double a_var, w_var;
  auto motion_status = data_saver_->UpdateMotionStatus(timestamp, a_var, w_var);
  render_status_overlay(msg, camera_sync, motion_status, a_var, w_var);

  // Log timestamp anomalies
  if (last_timestamp != 0) {
    double diff = (timestamp - last_timestamp) * 1e-9;
    std::stringstream ss;
    if (diff < 0) {
      ss << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
         << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
      log_file_ << ss.str() << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), ss.str());
    } else if (diff > 0.18 && !saver_confg_.image_gap_mode) {
      lost_cnt++;
      ss << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
         << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff
         << "s, larger than 0.18s, the data get lost! lost count: " << lost_cnt;
      log_file_ << ss.str() << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), ss.str());
    }
  }
  last_timestamp = timestamp;

  // Delegate to DataSaver
  data_saver_->ProcessImage(timestamp, msg, camera_sync);
}

void ROS2DataCollection::pcd_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  static uint64_t last_timestamp = 0, last_save_timestamp = 0;
  static uint64_t lost_cnt = 0;
  uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  last_get_pcd_time_ = msg->header.stamp.sec;

  RCLCPP_INFO(this->get_logger(),
              "get pcd at %fs, msg ts is: %fs, diff: %fms",
              this->now().seconds(), timestamp * 1e-9,
              this->now().seconds() * 1e3 - timestamp * 1e-6);

  if (!rclcpp::ok()) return;

  // Log timestamp anomalies
  if (last_timestamp != 0) {
    double diff = (timestamp - last_timestamp) * 1e-9;
    std::stringstream ss;
    if (diff < 0) {
      ss << std::fixed << "[pcd] last timestamp is: " << last_timestamp * 1e-9
         << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
      log_file_ << ss.str() << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), ss.str());
    } else if (diff > 0.18 && !saver_confg_.lidar_gap_mode) {
      lost_cnt++;
      ss << std::fixed << "[pcd] last timestamp is: " << last_timestamp * 1e-9
         << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff
         << "s, larger than 0.18s, the data get lost! lost count: " << lost_cnt;
      log_file_ << ss.str() << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), ss.str());
    }
  }
  last_timestamp = timestamp;

  // Delegate to DataSaver
  data_saver_->ProcessPcd(timestamp, msg);
}

void ROS2DataCollection::render_status_overlay(
    sensor_msgs::msg::Image::SharedPtr msg,
    bool camera_sync,
    MotionDetector::Status motion_status,
    double a_var, double w_var) {
  if (status_image_pub_->get_subscription_count() <= 0) return;

  std::vector<std::string> lines;
  auto dst = std::make_shared<sensor_msgs::msg::Image>(*msg);
  cv::Mat nv12_img = cv::Mat(dst->height, dst->width, CV_8UC1, dst->data.data());

  if (check_lidar_exist_ &&
      std::abs(dst->header.stamp.sec - last_get_pcd_time_.load()) > 3) {
    lines.push_back("STATE: NO LeiDa");
  } else if (!camera_sync) {
    lines.push_back("STATE: NO Tongbu");
  } else if (motion_status == MotionDetector::Status::IMU_NOT_ENOUGH) {
    lines.push_back("STATE: NO IMU");
  } else {
    lines.push_back("STATE: OK");
  }

  auto saved_img = data_saver_->GetImageSaveCount();
  auto saved_pcd = data_saver_->GetPcdSaveCount();
  lines.push_back("SAVED IMG: " + std::to_string(saved_img) +
                  ", PCD: " + std::to_string(saved_pcd));

  if (motion_status == MotionDetector::MOTION) {
    lines.push_back("CMD: COLLECTING MOTION IMG");
  } else if (motion_status == MotionDetector::ENTERING_STATIC ||
             motion_status == MotionDetector::STATIC) {
    int img_count = data_saver_->GetStaticBufferedImageCount();
    int pcd_count = data_saver_->GetStaticBufferedPcdCount();
    int img_target = data_saver_->GetIMGStaticCollectTarget();
    int pcd_target = data_saver_->GetPCDStaticCollectTarget();
    DataSaver::SaveState save_state = data_saver_->GetSaveStatus();
    if (save_state == DataSaver::SaveState::COLLECTING) {
      std::stringstream pcd_info;
      pcd_info << "PCD: " << pcd_count << "/" << pcd_target;
      lines.push_back(pcd_info.str());
      std::stringstream img_info;
      img_info << "IMG BUF: " << img_count << "/" << img_target;
      lines.push_back(img_info.str());
    } else if (save_state == DataSaver::SaveState::SAVING_STATIC) {
      std::stringstream pcd_info;
      pcd_info << "PCD: " << pcd_target << "/" << pcd_target;
      lines.push_back(pcd_info.str());
      std::stringstream img_info;
      img_info << "IMG BUF: " << img_target << "/" << img_target;
      lines.push_back(img_info.str());
    }
    lines.push_back("CMD: " + data_saver_->GetSaveStatusStr());
  }

  lines.push_back("MOTION: " + MotionDetector::State2String(motion_status)
  + ", a: " + std::to_string(a_var) + ", w: " + std::to_string(w_var));
  lines.push_back("DISK USE: " + data_collection::get_disk_info(data_dir_));

  const int x = 8;
  const int y = 20;
  const double font_scale = 2;
  const int thickness = 3;
  const int padding = 14;
  int baseline = 0;
  int max_text_width = 0;
  int total_text_height = 0;
  for (const auto& line : lines) {
    auto text_size = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
    max_text_width = std::max(max_text_width, text_size.width);
    total_text_height += text_size.height + 12;
  }
  int box_w = max_text_width + padding * 2;
  int box_h = total_text_height + padding * 3;
  cv::rectangle(nv12_img, cv::Rect(x, y, box_w, box_h), cv::Scalar(0, 0, 0), cv::FILLED);
  int text_y = y + padding * 4;
  for (const auto& line : lines) {
    cv::putText(nv12_img, line, cv::Point(x + padding, text_y),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), thickness);
    text_y += 60;
  }
  status_image_pub_->publish(*dst);
}
