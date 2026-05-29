#include "data_saver.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

DataSaver::DataSaver(const Config& config) : config_(config) {
  if (config_.motion_detect) {
    motion_detector_ = std::make_shared<MotionDetector>(
        config_.motion_imu_window_size, config_.motion_accel_th, config_.motion_gyro_th);
    motion_record_file_.open(config_.data_dir + "/motion_datas.txt", std::ios::out);
    static_record_file_.open(config_.data_dir + "/static_datas.txt", std::ios::out);
  }
}

DataSaver::~DataSaver() {
  StopWorkerThreads();
  WriteYamlRecord();
}

void DataSaver::FeedImu(uint64_t timestamp, const ImuData& imu_data) {
  if (motion_detector_) {
    motion_detector_->FeedImu(imu_data);
  }
  ++imu_save_cnt_;
}

MotionDetector::Status DataSaver::UpdateMotionStatus(uint64_t timestamp,
                                                     double &a_var, double &w_var) {
  if (!config_.motion_detect) {
    return MotionDetector::Status::DISABLE;
  }
  // Compute motion status at image timestamp

  uint64_t nearest_imu_ts = 0;
  auto status = motion_detector_->GetStatus(timestamp, nearest_imu_ts, a_var, w_var);
  motion_status_.store(status, std::memory_order_relaxed);
  return status;
}

bool DataSaver::ProcessImage(uint64_t timestamp,
                             const sensor_msgs::msg::Image::SharedPtr msg,
                             bool camera_sync) {
  if (!config_.motion_detect) {
    if (!camera_sync) return false;
    if (config_.image_gap_mode > 0) {
      auto last_ts = last_image_save_ts_.load(std::memory_order_relaxed);
      double diff = (timestamp - last_ts) * 1e-9;
      if (diff < config_.image_gap_mode) return false;
    }
    last_image_save_ts_.store(timestamp, std::memory_order_relaxed);
    if (!is_paused_) {
      image_que_.put(msg);
    }
    if (image_que_.size() > 10) {
      image_que_.pop_front();
    }
    return true;
  }

  switch (motion_status_) {
    case MotionDetector::MOTION:
      // Motion: save image directly with sync and gap filtering
      if (!camera_sync) return false;
      if (config_.image_gap_mode > 0) {
        auto last_ts = last_image_save_ts_.load(std::memory_order_relaxed);
        double diff = (timestamp - last_ts) * 1e-9;
        if (diff < config_.image_gap_mode) return false;
      }
      last_image_save_ts_.store(timestamp, std::memory_order_relaxed);
      if (!is_paused_) {
        image_que_.put(msg);
        if (motion_record_file_.is_open()) {
          motion_record_file_ << "image/" << timestamp << "." << config_.image_format << std::endl;
        }
      }
      saver_status_.store(SaveState::WAIT_STATIC);
      return true;

    case MotionDetector::ENTERING_STATIC:
      // Transition: clear buffers, start collecting static data
      static_flushing_.store(false, std::memory_order_relaxed);
      static_image_buf_.clear();
      static_pcd_buf_.clear();
      // No sync/gap filtering for the entering-static image
      static_image_buf_.put(msg);
      saver_status_.store(SaveState::COLLECTING);
      break;

    case MotionDetector::STATIC:
      // Skip buffering while flush is in progress
      if (static_flushing_.load(std::memory_order_relaxed)) {
        return false;
      }
      if (saver_status_ == SaveState::COLLECTING) {
        if (static_image_buf_.size() < config_.img_static_collect_count) {
          static_image_buf_.put(msg);
        }
      } else if (saver_status_ == SaveState::WAIT_STATIC) {
        saver_status_.store(SaveState::COLLECTING);
      }
      break;

    case MotionDetector::ENTERING_MOTION:
      // Motion interrupted — discard all buffered data
      static_image_buf_.clear();
      static_pcd_buf_.clear();
      static_flushing_.store(false, std::memory_order_relaxed);
      return false;

    default:
      return false;
  }

  // Mark flushing when both targets reached
  if (static_image_buf_.size() >= config_.img_static_collect_count &&
      static_pcd_buf_.size() >= config_.pcd_static_collect_count) {
    static_flushing_.store(true, std::memory_order_relaxed);
    saver_status_.store(SaveState::SAVING_STATIC);
  }
  return true;
}

bool DataSaver::ProcessPcd(uint64_t timestamp,
                           const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!config_.motion_detect) {
    if (config_.lidar_gap_mode > 0) {
      auto last_ts = last_pcd_save_ts_.load(std::memory_order_relaxed);
      double diff = (timestamp - last_ts) * 1e-9;
      if (diff < config_.lidar_gap_mode) return false;
    }
    last_pcd_save_ts_.store(timestamp, std::memory_order_relaxed);
    if (!is_paused_) {
      pcd_que_.put(msg);
    }
    if (pcd_que_.size() > 10) {
      pcd_que_.pop_front();
    }
    return true;
  }

  auto status = static_cast<MotionDetector::Status>(
      motion_status_.load(std::memory_order_relaxed));

  switch (status) {
    case MotionDetector::ENTERING_STATIC:
    case MotionDetector::STATIC:
      // Skip buffering while flush is in progress
      if (static_flushing_.load(std::memory_order_relaxed)) {
        return false;
      }
      if (saver_status_ == SaveState::COLLECTING) {
        if (static_pcd_buf_.size() < config_.pcd_static_collect_count) {
          static_pcd_buf_.put(msg);
        }
      }
      break;
  }

  // Mark flushing when target reached
  if (static_image_buf_.size() >= config_.img_static_collect_count &&
      static_pcd_buf_.size() >= config_.pcd_static_collect_count) {
    static_flushing_.store(true, std::memory_order_relaxed);
    saver_status_.store(SaveState::SAVING_STATIC);
  }
  return true;
}

void DataSaver::FlushThreadFunc() {
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (static_flushing_.load(std::memory_order_relaxed)) {
      DrainAndFlush(static_image_buf_, static_pcd_buf_);
      saver_status_.store(SaveState::WAIT_MOTION);
    }
  }
}

void DataSaver::DrainAndFlush(
    blockqueue<sensor_msgs::msg::Image::SharedPtr>& img_buf,
    blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr>& pcd_buf) {
  int target_image_count = config_.img_static_collect_count;
  int target_pcd_count = config_.pcd_static_collect_count - config_.delay_confirm_count;
  sensor_msgs::msg::Image::SharedPtr img_msg;
  while (img_buf.get(img_msg, 1) && --target_image_count >= 0) {
    int64_t ts = img_msg->header.stamp.sec * 1e9 + img_msg->header.stamp.nanosec;
    SaveImageToFile(ts, img_msg);
    if (static_record_file_.is_open()) {
      static_record_file_ << "image/" << ts << "." << config_.image_format << std::endl;
    }
    last_image_save_ts_.store(ts, std::memory_order_relaxed);
  }
  img_buf.clear();

  sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg;
  while (pcd_buf.get(pcd_msg, 1) && --target_pcd_count >= 0) {
    int64_t ts = pcd_msg->header.stamp.sec * 1e9 + pcd_msg->header.stamp.nanosec;
    SavePcdToFile(ts, pcd_msg);
    if (static_record_file_.is_open()) {
      static_record_file_ << "pcd/" << ts << ".pcd" << std::endl;
    }
    last_pcd_save_ts_.store(ts, std::memory_order_relaxed);
  }
  pcd_buf.clear();
}

MotionDetector::Status DataSaver::GetMotionStatus() const {
  return static_cast<MotionDetector::Status>(motion_status_.load(std::memory_order_relaxed));
}

int DataSaver::GetStaticBufferedImageCount() {
  return static_image_buf_.size();
}

int DataSaver::GetStaticBufferedPcdCount() {
  return static_pcd_buf_.size();
}

int DataSaver::GetIMGStaticCollectTarget() const {
  return config_.img_static_collect_count;
}

int DataSaver::GetPCDStaticCollectTarget() const {
  return config_.pcd_static_collect_count;
}

uint32_t DataSaver::GetImageSaveCount() const {
  return image_save_cnt_.load();
}

uint32_t DataSaver::GetPcdSaveCount() const {
  return pcd_save_cnt_.load();
}

uint32_t DataSaver::GetImuSaveCount() const {
  return imu_save_cnt_.load();
}

void DataSaver::SetPaused(bool paused) {
  is_paused_.store(paused);
}

bool DataSaver::IsPaused() const {
  return is_paused_.load();
}

void DataSaver::SaveImageDirect(int64_t timestamp,
                                const sensor_msgs::msg::Image::SharedPtr msg) {
  SaveImageToFile(timestamp, msg);
}

void DataSaver::SavePcdDirect(int64_t timestamp,
                              const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  SavePcdToFile(timestamp, msg);
}

blockqueue<sensor_msgs::msg::Image::SharedPtr>& DataSaver::GetImageQueue() {
  return image_que_;
}

blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr>& DataSaver::GetPcdQueue() {
  return pcd_que_;
}

void DataSaver::StartWorkerThreads(int thread_count) {
  for (int i = 0; i < thread_count; ++i) {
    worker_threads_.emplace_back(std::make_shared<std::thread>(
        &DataSaver::ImageWorkerThread, this));
    worker_threads_.emplace_back(std::make_shared<std::thread>(
        &DataSaver::PcdWorkerThread, this));
  }
  flush_thread_ = std::make_shared<std::thread>(&DataSaver::FlushThreadFunc, this);
}

void DataSaver::StopWorkerThreads() {
  if (flush_thread_ && flush_thread_->joinable()) {
    flush_thread_->join();
  }
  flush_thread_.reset();

  for (auto& t : worker_threads_) {
    if (t && t->joinable()) {
      t->join();
    }
  }
  worker_threads_.clear();
}

void DataSaver::ImageWorkerThread() {
  while (rclcpp::ok() || image_que_.size() > 0) {
    sensor_msgs::msg::Image::SharedPtr msg;
    if (image_que_.get(msg)) {
      if (!is_paused_) {
        int64_t ts = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        SaveImageToFile(ts, msg);
      }
    }
  }
}

void DataSaver::PcdWorkerThread() {
  while (rclcpp::ok() || pcd_que_.size() > 0) {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    if (pcd_que_.get(msg)) {
      if (!is_paused_) {
        int64_t ts = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        SavePcdToFile(ts, msg);
      }
    }
  }
}

void DataSaver::SaveImageToFile(int64_t timestamp,
                                const sensor_msgs::msg::Image::SharedPtr msg) {
  std::string filename;
  if (config_.image_format == "yuv") {
    filename = std::to_string(timestamp) + ".yuv";
    std::ofstream file(config_.image_dir + "/" + filename, std::ios::out | std::ios::binary);
    if (file.is_open()) {
      file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
    }
  } else {
    filename = std::to_string(timestamp) + "." + config_.image_format;
    cv::Mat nv12(msg->height * 3 / 2, msg->width, CV_8UC1, msg->data.data());
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    cv::imwrite(config_.image_dir + "/" + filename, bgr);
  }
  ++image_save_cnt_;
}

void DataSaver::SavePcdToFile(int64_t timestamp,
                              const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const int N = msg->width;
  std::string filename = config_.pcd_dir + "/" + std::to_string(timestamp) + ".pcd";
  std::ofstream file;
  if (config_.save_pcd_bin) {
    file = std::ofstream(filename, std::ios::binary);
  } else {
    file = std::ofstream(filename);
  }
  if (!file.is_open()) return;

  file << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z intensity offset_time tag line\n"
       << "SIZE 4 4 4 4 4 1 1\n"
       << "TYPE F F F F U U U\n"
       << "COUNT 1 1 1 1 1 1 1\n"
       << "WIDTH " << N << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << N << "\n";

  bool use_binary = config_.save_pcd_bin;
  if (msg->point_step != 22) {
    std::cout << "point_step of pointcloud2 is " << msg->point_step
              << ", rather than 22, so save pcd as ascii" << std::endl;
    use_binary = false;
  }

  if (use_binary) {
    file << "DATA binary\n";
    file.write(reinterpret_cast<const char*>(msg->data.data()), N * 22);
  } else {
    file << "DATA ascii\n";
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_tag(*msg, "tag");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_line(*msg, "line");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_offset_time(*msg, "offset_time");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z,
        ++iter_intensity, ++iter_tag, ++iter_line, ++iter_offset_time) {
      file << *iter_x << " " << *iter_y << " " << *iter_z << " "
           << *iter_intensity << " " << *iter_offset_time << " "
           << static_cast<int>(*iter_tag) << " " << static_cast<int>(*iter_line) << "\n";
    }
  }

  ++pcd_save_cnt_;
}

void DataSaver::WriteYamlRecord() {
  if (!config_.motion_detect) return;
  if (motion_record_file_.is_open()) motion_record_file_.close();
  if (static_record_file_.is_open()) static_record_file_.close();
}