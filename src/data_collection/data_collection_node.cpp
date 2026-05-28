#include "data_collection_node.h"

#include "kbhit.h"
#include "utils.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

ROS2DataCollection::ROS2DataCollection() : Node("ROS2DataCollection") {
  // Declare ROS parameters
  std::string output_dir = this->declare_parameter("output_dir", "/media/sda2/");
  std::string imu_topic = this->declare_parameter("imu_topic", "/livox/imu");
  std::string lidar_topic = this->declare_parameter("lidar_topic", "/livox/lidar");
  std::string image_topic = this->declare_parameter("image_topic", "/image_combine_raw");
  std::string image_format = this->declare_parameter("image_format", "png");
  int save_thread_num = this->declare_parameter("save_thread_num", 4);
  snap_shot_ = this->declare_parameter("snap_shot", snap_shot_);
  float gravity = this->declare_parameter("gravity", 9.81f);
  int image_gap_mode = this->declare_parameter("image_gap_mode", 0);
  int lidar_gap_mode = this->declare_parameter("lidar_gap_mode", 0);
  bool motion_detect = this->declare_parameter("motion_detect", false);
  check_camera_sync_ = this->declare_parameter("check_camera_sync", check_camera_sync_);
  check_lidar_exist_ = this->declare_parameter("check_lidar_exist", check_lidar_exist_);
  bool save_pcd_bin = this->declare_parameter("save_pcd_bin", false);
  bool is_ir = this->declare_parameter("is_ir", false);
  int motion_imu_window_size = this->declare_parameter("motion_imu_window_size", 200);
  bool check_is_external_driver = this->declare_parameter("check_ext_driver", false);
  bool enable_pause = this->declare_parameter("enable_pause", true);
  double a_th = this->declare_parameter("motion_accel_th", 1.1);
  double w_th = this->declare_parameter("motion_gyro_th", 0.0);
  int pcd_static_collect_count = this->declare_parameter("pcd_static_collect_count", 50);
  int img_static_collect_count = this->declare_parameter("img_static_collect_count", 2);
  std::string calib_file = this->declare_parameter("calib_file", "");

  // Setup directories
  data_dir_ = output_dir + data_collection::generate_timestamp_folder();
  if (is_ir) {
    data_dir_ += "_image_ir";
  }
  std::string image_dir = data_dir_ + "/image/";
  std::string pcd_dir = data_dir_ + "/pcd/";
  std::string imu_dir = data_dir_ + "/imu/";

  std::filesystem::create_directories(data_dir_);
  std::filesystem::create_directories(image_dir);
  std::filesystem::create_directories(pcd_dir);
  std::filesystem::create_directories(imu_dir);

  // Validate image format
  if (image_format != "yuv" && image_format != "png" && image_format != "jpg" && image_format != "jpeg") {
    std::cout << "[ERROR] Please check the image_format is 'yuv' or 'png' or 'jpg'"
                 ", rather than " << image_format << std::endl;
    std::exit(-1);
  }

  // Log parameters
  char log_buffer[4096];
  snprintf(log_buffer, sizeof(log_buffer),
           "data_dir: %s\n"
           "imu_topic: %s, lidar_topic: %s, image_topic: %s\n"
           "snap_shot: %d, gravity: %f, image_gap_mode: %d, lidar_gap_mode: %d, check_ext_driver: %d\n"
           "enable_pause: %d, motion_detect: %d, motion_window_size: %d, motion_accel_th: %f, motion_gyro_th: %f\n"
           "img_static_collect_count: %d, pcd_static_collect_count: %d\n"
           "save_pcd_bin: %d, is_ir: %d\n"
           "check_camera_sync: %d, check_lidar_exist_: %d\n"
           "image_format: %s, calib_file: %s\n",
           data_dir_.c_str(), imu_topic.c_str(), lidar_topic.c_str(), image_topic.c_str(),
           snap_shot_, gravity, image_gap_mode, lidar_gap_mode, check_is_external_driver,
           enable_pause, motion_detect, motion_imu_window_size, a_th, w_th,
           img_static_collect_count, pcd_static_collect_count,
           save_pcd_bin, is_ir,
           check_camera_sync_, check_lidar_exist_,
           image_format.c_str(), calib_file.c_str());
  std::string log_str(log_buffer);
  data_collection::generate_device_info(data_dir_, calib_file, log_str);
  RCLCPP_WARN(this->get_logger(), "%s .", log_buffer);

  // Check disk space
  float capacity;
  float space_ratio = 1 - data_collection::get_free_space(output_dir, capacity);
  if (check_is_external_driver && capacity < 100) {
    std::cout << "[ERROR] Please check the Mobile Hard Disk is plugged in!!!!!!" << std::endl;
    std::exit(-1);
  }
  if (space_ratio > 0.9f) {
    std::cout << std::fixed << std::setprecision(1)
              << "[ERROR] The disk of dir: '" << output_dir << "' is now at: "
              << space_ratio * 100 << "% usage, which is nearly full!" << std::endl;
  } else {
    std::cout << std::fixed << std::setprecision(1)
              << "The disk of dir: '" << output_dir << "' is now at: "
              << space_ratio * 100 << "% usage" << std::endl;
  }

  // Open IMU log file
  imu_filename_ = imu_dir + "/imu_data.txt";
  imu_file_.open(imu_filename_, std::ios::out | std::ios::app);
  if (!imu_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open IMU file: %s", imu_filename_.c_str());
  }

  log_file_.open(data_dir_ + "/log.txt", std::ios::out);
  if (!log_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", (data_dir_ + "/log.log").c_str());
  }

  // Create DataSaver
  saver_confg_.data_dir = data_dir_;
  saver_confg_.image_dir = image_dir;
  saver_confg_.pcd_dir = pcd_dir;
  saver_confg_.image_format = image_format;
  saver_confg_.save_pcd_bin = save_pcd_bin;
  saver_confg_.gravity = gravity;
  saver_confg_.image_gap_mode = image_gap_mode;
  saver_confg_.lidar_gap_mode = lidar_gap_mode;
  saver_confg_.check_camera_sync = check_camera_sync_;
  saver_confg_.motion_detect = motion_detect;
  saver_confg_.pcd_static_collect_count = pcd_static_collect_count;
  saver_confg_.img_static_collect_count = img_static_collect_count;
  saver_confg_.motion_imu_window_size = motion_imu_window_size;
  saver_confg_.motion_accel_th = a_th;
  saver_confg_.motion_gyro_th = w_th;

  data_saver_ = std::make_unique<DataSaver>(saver_confg_);

  // Start worker threads
  if (!snap_shot_) {
    data_saver_->StartWorkerThreads(save_thread_num);
  }

  // ROS subscriptions
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 1000, std::bind(&ROS2DataCollection::imu_callback, this, std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(), std::bind(&ROS2DataCollection::image_callback, this, std::placeholders::_1));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, 30, std::bind(&ROS2DataCollection::pcd_callback, this, std::placeholders::_1));

  status_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "~/status_image_combine", 10);

  // Snap-shot or pause control thread
  if (snap_shot_) {
    auto snap_func = [this]() {
      RCLCPP_WARN(this->get_logger(), "snap_shot start.");
      RCLCPP_WARN(this->get_logger(), "waiting for snap shot cmd, please enter ENTER.");
      while (rclcpp::ok()) {
        if (!kbhit()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }
        int c = getchar();
        std::cout << "user input: " << c << std::endl;
        if (c != '\n') continue;

        bool saved = false;
        auto& image_que = data_saver_->GetImageQueue();
        auto& pcd_que = data_saver_->GetPcdQueue();

        while (!saved && rclcpp::ok()) {
          sensor_msgs::msg::Image::SharedPtr img_msg = nullptr;
          sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg = nullptr;
          if (image_que.get(img_msg, 300)) {
            int64_t image_ts = img_msg->header.stamp.sec * 1e9 + img_msg->header.stamp.nanosec;
            while (!saved && rclcpp::ok()) {
              if (pcd_que.get(pcd_msg, 300)) {
                int64_t pcd_ts = pcd_msg->header.stamp.sec * 1e9 + pcd_msg->header.stamp.nanosec;
                if (check_camera_sync_ && std::abs(pcd_ts - image_ts) >= 1e6) {
                  if (pcd_ts > image_ts) {
                    pcd_que.put_front(pcd_msg);
                    break;
                  }
                  continue;
                }
                data_saver_->SaveImageDirect(image_ts, img_msg);
                data_saver_->SavePcdDirect(image_ts, pcd_msg);
                RCLCPP_WARN(this->get_logger(),
                            "save snap shot data succeed. image ts: %f, pcd ts: %f, diff: %f",
                            image_ts * 1e-9, pcd_ts * 1e-9, (image_ts - pcd_ts) * 1e-9);
                saved = true;
              } else {
                RCLCPP_ERROR(this->get_logger(),
                             "=====failed to get pcd, so we only save image, ts: %f ====",
                             image_ts * 1e-9);
                data_saver_->SaveImageDirect(image_ts, img_msg);
                saved = true;
                break;
              }
            }
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "=====failed to get image, image size: %d ====",
                         image_que.size());
            break;
          }
        }
        RCLCPP_WARN(this->get_logger(), "we have saved %d images and %d pcds, and %d imus.",
                    data_saver_->GetImageSaveCount(), data_saver_->GetPcdSaveCount(),
                    data_saver_->GetImuSaveCount());
      }
      RCLCPP_WARN(this->get_logger(), "snap_shot exit.");
    };
    control_threads_.emplace_back(std::make_shared<std::thread>(snap_func));
  } else {
    auto get_pause_func = [this]() {
      RCLCPP_WARN(this->get_logger(), "get_pause_func start.");
      while (rclcpp::ok()) {
        if (!kbhit()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }
        int c = getchar();
        std::cout << "user input: " << c << std::endl;
        if (c == '\n') {
          data_saver_->SetPaused(!data_saver_->IsPaused());
        }
        if (data_saver_->IsPaused()) {
          std::cout << "\rdata saving stop!!!                        " << std::flush;
        } else {
          std::cout << "\rdata saving starts!!!                        " << std::flush;
        }
      }
      RCLCPP_WARN(this->get_logger(), "get_pause_func exit.");
    };
    if (enable_pause) {
      control_threads_.emplace_back(std::make_shared<std::thread>(get_pause_func));
    }
  }
}

ROS2DataCollection::~ROS2DataCollection() {
  for (auto& t : control_threads_) {
    if (t && t->joinable()) {
      t->join();
    }
  }
  control_threads_.clear();

  data_saver_->StopWorkerThreads();

  if (imu_file_.is_open()) {
    imu_file_.close();
  }
  if (log_file_.is_open()) {
    log_file_.close();
  }
  RCLCPP_WARN(this->get_logger(), "we have saved %d images and %d pcds, and %d imus.",
              data_saver_->GetImageSaveCount(), data_saver_->GetPcdSaveCount(),
              data_saver_->GetImuSaveCount());
}
