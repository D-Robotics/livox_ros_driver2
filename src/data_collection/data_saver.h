#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include "block_queue.h"
#include "motion_detector.h"

class DataSaver {
 public:

  enum SaveState {
    WAIT_STATIC,
    COLLECTING,
    SAVING_STATIC,
    WAIT_MOTION
  };

  static std::string State2String(SaveState status) {
    switch (status) {
      case SaveState::WAIT_STATIC:
        return "WAIT_STATIC";
      case SaveState::COLLECTING:
        return "COLLECTING";
      case SaveState::WAIT_MOTION:
        return "WAIT_MOTION";
      case SaveState::SAVING_STATIC:
        return "SAVING_STATIC";
    }
  }

  struct Config {
    std::string data_dir;
    std::string image_dir;
    std::string pcd_dir;
    std::string image_format = "png";
    bool save_pcd_bin = false;
    float gravity = 9.81f;
    int image_gap_mode = 0;
    int lidar_gap_mode = 0;
    bool check_camera_sync = true;
    bool motion_detect = false;
    int pcd_static_collect_count = 65;
    int delay_confirm_count = 15;
    int img_static_collect_count = 2;
    int motion_imu_window_size = 200;
    double motion_accel_th = 1.1;
    double motion_gyro_th = 0.0;
  };

  explicit DataSaver(const Config& config);
  ~DataSaver();
  MotionDetector::Status UpdateMotionStatus(uint64_t timestamp,
                                            double &a_var, double &w_var);
  void FeedImu(uint64_t timestamp, const ImuData& imu_data);

  bool ProcessImage(uint64_t timestamp,
                    const sensor_msgs::msg::Image::SharedPtr msg,
                    bool camera_sync);

  bool ProcessPcd(uint64_t timestamp,
                  const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Accessors (thread-safe via atomic or blockqueue)
  MotionDetector::Status GetMotionStatus() const;
  std::string GetSaveStatusStr() const {
    return State2String((SaveState)saver_status_.load());
  }
  SaveState GetSaveStatus() const {
    return (SaveState)saver_status_.load();
  }
  int GetStaticBufferedImageCount();
  int GetStaticBufferedPcdCount();
  int GetIMGStaticCollectTarget() const;
  int GetPCDStaticCollectTarget() const;
  uint32_t GetImageSaveCount() const;
  uint32_t GetPcdSaveCount() const;
  uint32_t GetImuSaveCount() const;

  void StartWorkerThreads(int thread_count);
  void StopWorkerThreads();

  void SetPaused(bool paused);
  bool IsPaused() const;

  void SaveImageDirect(int64_t timestamp,
                       const sensor_msgs::msg::Image::SharedPtr msg);
  void SavePcdDirect(int64_t timestamp,
                     const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  blockqueue<sensor_msgs::msg::Image::SharedPtr>& GetImageQueue();
  blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr>& GetPcdQueue();

 private:
  void FlushThreadFunc();
  void DrainAndFlush(blockqueue<sensor_msgs::msg::Image::SharedPtr>& img_buf,
                     blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr>& pcd_buf);
  void WriteYamlRecord();
  void SaveImageToFile(int64_t timestamp,
                       const sensor_msgs::msg::Image::SharedPtr msg);
  void SavePcdToFile(int64_t timestamp,
                     const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImageWorkerThread();
  void PcdWorkerThread();

  Config config_;
  std::shared_ptr<MotionDetector> motion_detector_;

  // Motion status — atomic, no mutex needed
  std::atomic<int> motion_status_{MotionDetector::DISABLE};
  std::atomic<int> saver_status_{SaveState::WAIT_STATIC};

  // True when static buffers have reached collect count and are being flushed.
  std::atomic<bool> static_flushing_{false};

  // Gap mode timestamps
  std::atomic<uint64_t> last_image_save_ts_{0};
  std::atomic<uint64_t> last_pcd_save_ts_{0};

  // Record files: motion image filenames and static image+pcd pair filenames
  std::ofstream motion_record_file_;
  std::ofstream static_record_file_;

  // Static-mode buffers — thread-safe via blockqueue
  blockqueue<sensor_msgs::msg::Image::SharedPtr> static_image_buf_;
  blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr> static_pcd_buf_;

  // Motion-mode queues (direct save via worker threads)
  blockqueue<sensor_msgs::msg::Image::SharedPtr> image_que_;
  blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr> pcd_que_;

  // Counts
  std::atomic<uint32_t> image_save_cnt_{0};
  std::atomic<uint32_t> pcd_save_cnt_{0};
  std::atomic<uint32_t> imu_save_cnt_{0};

  // State
  std::atomic<bool> is_paused_{false};
  std::vector<std::shared_ptr<std::thread>> worker_threads_;
  std::shared_ptr<std::thread> flush_thread_;
};