#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "data_saver.h"

class ROS2DataCollection : public rclcpp::Node {
 public:
  ROS2DataCollection();
  ~ROS2DataCollection();

 private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void image_callback(sensor_msgs::msg::Image::SharedPtr msg);
  void pcd_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void render_status_overlay(sensor_msgs::msg::Image::SharedPtr msg,
                             bool camera_sync,
                             MotionDetector::Status motion_status,
                             double a_var, double w_var);

  // Core save logic
  DataSaver::Config saver_confg_;
  std::unique_ptr<DataSaver> data_saver_;

  // Paths & files
  std::string data_dir_;
  std::string imu_filename_;
  std::ofstream imu_file_;
  std::ofstream log_file_;

  // Config kept for status overlay and snap_shot
  bool snap_shot_ = false;
  bool check_camera_sync_ = true;
  bool check_lidar_exist_ = true;
  std::atomic<int32_t> last_get_pcd_time_{0};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr status_image_pub_;

  // Threads (snap_shot / pause control)
  std::vector<std::shared_ptr<std::thread>> control_threads_;
};