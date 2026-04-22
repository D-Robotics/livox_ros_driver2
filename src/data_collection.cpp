#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <deque>
#include <filesystem>

#include <sys/ioctl.h>
#include <termios.h>

#include <opencv2/opencv.hpp>
#include "motion_detector.h"

template<class T>
struct blockqueue {
  int put(T &&t) {
    {
      std::lock_guard<std::mutex>lck (mtx);
      que.emplace_back(t);
      cv.notify_one();
      return que.size();
    }
  }
  int put(T &t) {
    {
      std::lock_guard<std::mutex>lck (mtx);
      que.push_back(t);
      cv.notify_one();
      return que.size();
    }
  }

  int put_front(T &&t) {
    {
      std::lock_guard<std::mutex>lck (mtx);
      que.emplace_back(t);
      cv.notify_one();
      return que.size();
    }
  }
  int put_front(T &t) {
    {
      std::lock_guard<std::mutex>lck (mtx);
      que.push_back(t);
      cv.notify_one();
      return que.size();
    }
  }

  bool get(T &t, uint32_t timeout_ms = 300) {
    {
      std::unique_lock<std::mutex>lck (mtx);
      if (!que.empty() || cv.wait_for(
          lck, std::chrono::milliseconds(timeout_ms),
          [&]() {auto sz = que.size();
            //printf("sz:%d\n", sz);
            return sz > 0;})) {
        t = que.front();
        que.pop_front();
        return true;
      }
      return false;
    }
  }

  void pop_front() {
    std::lock_guard<std::mutex>lck (mtx);
    que.pop_front();
  }

  void clear() {
    std::lock_guard<std::mutex>lck (mtx);
    que.clear();
  }

  uint size() {
    std::lock_guard<std::mutex>lck (mtx);
    return que.size();
  }

 private:
  std::condition_variable cv;
  std::mutex mtx;
  std::deque<T> que;
};

bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

class ROS2DataCollection : public rclcpp::Node {
 public:
  ROS2DataCollection() : Node("ROS2DataCollection") {

    std::string home_dir = this->declare_parameter("home_dir", "/media/sda2/");;
    std::string imu_topic = this->declare_parameter("imu_topic", "/livox/imu");;
    std::string lidar_topic = this->declare_parameter("lidar_topic", "/livox/lidar");;
    std::string image_topic = this->declare_parameter("image_topic", "/image_combine_raw");;
    image_format_ = this->declare_parameter("image_format", "png");;
    int save_thread_num = this->declare_parameter("save_thread_num", 4);;
    snap_shot_ = this->declare_parameter("snap_shot", snap_shot_);
    gravity_ = this->declare_parameter("gravity", gravity_);
    image_gap_mode_ = this->declare_parameter("image_gap_mode", image_gap_mode_);
    lidar_gap_mode_ = this->declare_parameter("lidar_gap_mode", lidar_gap_mode_);
    motion_detect_ = this->declare_parameter("motion_detect", motion_detect_);
    check_camera_sync_ = this->declare_parameter("check_camera_sync", check_camera_sync_);
    check_lidar_exist_ = this->declare_parameter("check_lidar_exist", check_lidar_exist_);
    save_pcd_bin_ = this->declare_parameter("save_pcd_bin", save_pcd_bin_.load());
    bool is_ir = this->declare_parameter("is_ir", false);
    int motion_window_size = this->declare_parameter("motion_window_size", 200);
    bool check_is_external_driver = this->declare_parameter("check_ext_driver", false);
    bool enable_pause = this->declare_parameter("enable_pause", true);
    double a_th = this->declare_parameter("motion_accel_th", 1.1);
    double w_th = this->declare_parameter("motion_gyro_th", 0.0);
    std::string calib_file = this->declare_parameter("calib_file", "");

    if (motion_detect_) {
      motion_detector_ = std::make_shared<MotionDetector>(motion_window_size, a_th, w_th);
    }
    data_dir_ = home_dir + generate_timestamp_folder();
    if (is_ir) {
      data_dir_ += "_image_ir";
    }
    image_dir_ = data_dir_ + "/image/";
    pcd_dir_ = data_dir_ + "/pcd/";
    imu_dir_ = data_dir_ + "/imu/";
    system(("mkdir -p " + data_dir_).c_str());
    system(("mkdir -p " + image_dir_).c_str());
    system(("mkdir -p " + pcd_dir_).c_str());
    system(("mkdir -p " + imu_dir_).c_str());

    char log_buffer[4096];
    snprintf(log_buffer, sizeof(log_buffer),
             "data_dir: %s\n"
             "imu_topic: %s, lidar_topic: %s, image_topic: %s\n"
             "snap_shot: %d, gravity_: %f, image_gap_mode: %d, lidar_gap_mode: %d, check_ext_driver: %d\n"
             "enable_pause: %d, motion_detect: %d, motion_window_size: %d, motion_accel_th: %f, motion_gyro_th: %f\n"
             "save_pcd_bin: %d, is_ir: %d\n"
             "check_camera_sync: %d, image_format: %s, calib_file: %s ",
             data_dir_.c_str(), imu_topic.c_str(), lidar_topic.c_str(), image_topic.c_str(),
             snap_shot_, gravity_, image_gap_mode_, lidar_gap_mode_, check_is_external_driver,
             enable_pause, motion_detect_, motion_window_size, a_th, w_th,
             save_pcd_bin_.load(), is_ir,
             check_camera_sync_, image_format_.c_str(), calib_file.c_str()
    );
    std::string log_str(log_buffer);
    generate_device_info( calib_file, log_str);
    RCLCPP_WARN(this->get_logger(), "%s .", log_buffer);

    float capacity;
    float space_ratio = 1 - get_free_space(home_dir, capacity);
    if (check_is_external_driver) {
      if (capacity < 100) {
        std::cout << "[ERROR] Please check the Mobile Hard Disk is plugged in!!!!!!" << std::endl;
        std::exit(-1);
      }
    }
    if (space_ratio > 0.9) {
      std::cout << std::fixed << std::setprecision(1)
                << "[ERROR] The disk of dir: '" << home_dir << "' is now at: "
                << space_ratio * 100 << "% usage, which is nearly full!" << std::endl;
      std::cout << "[ERROR] Please chose another directory or port over to another disk !!" << std::endl;
      //std::exit(-1);
    } else {
      std::cout << std::fixed << std::setprecision(1)
                << "The disk of dir: '" << home_dir << "' is now at: "
                << space_ratio * 100 << "% usage" << std::endl;
    }

    imu_filename_ = imu_dir_ + "/imu_data.txt";
    imu_file_.open(imu_filename_, std::ios::out | std::ios::app);
    if (!imu_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open IMU file: %s", imu_filename_.c_str());
    }

    log_file_.open(data_dir_ + "/log.txt", std::ios::out);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", (data_dir_ + "/log.log").c_str());
    }

    if (!snap_shot_) {
      for (int i = 0; i < save_thread_num; ++i) {
        save_threads_.emplace_back(std::make_shared<std::thread>(
            std::bind(&ROS2DataCollection::save_pcd_thread, this)));
        save_threads_.emplace_back(std::make_shared<std::thread>(
            std::bind(&ROS2DataCollection::save_image_thread, this)));
      }
    }

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 1000, std::bind(&ROS2DataCollection::imu_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 30, std::bind(&ROS2DataCollection::image_callback, this, std::placeholders::_1));

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 30, std::bind(&ROS2DataCollection::pcd_callback, this, std::placeholders::_1));

    status_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "~/status_image_combine", 10
    );

    if (snap_shot_) {
      auto snap_func = [this]() {
        int64_t image_ts, pcd_ts;
        RCLCPP_WARN(this->get_logger(), "snap_shot start.");
        while (rclcpp::ok()) {
          bool saved = false;
          sensor_msgs::msg::Image::SharedPtr img_msg = nullptr;
          sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg = nullptr;
          RCLCPP_WARN(this->get_logger(), "waiting for snap shot cmd, please enter ENTER.");
          if (!kbhit()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
          }
          int c;
          c = getchar();
          std::cout << "user input: " << c << std::endl;
          if (c == '\n') {
            get_shot_time_ = this->now().seconds();
            RCLCPP_WARN(this->get_logger(), "get snap shot at: %f.", get_shot_time_);
            while (!saved && rclcpp::ok()) {
              if (image_que_.get(img_msg, 300)) {
                image_ts = img_msg->header.stamp.sec * 1e9 + img_msg->header.stamp.nanosec;
                while (!saved && rclcpp::ok()) {
                  if (pcd_que_.get(pcd_msg, 300)) {
                    pcd_ts = pcd_msg->header.stamp.sec * 1e9 + pcd_msg->header.stamp.nanosec;
                    if (std::abs(pcd_ts - image_ts) < 1e6) {
                      save_image(image_ts, img_msg);
                      save_pcd(image_ts, pcd_msg);
                      RCLCPP_WARN(this->get_logger(),
                                  "save snap shot data succeed. image ts: %f, pcd ts: %f, diff: %f",
                                  image_ts * 1e-9, pcd_ts * 1e-9, (image_ts - pcd_ts) * 1e-9);
                      saved = true;
                    } else if (pcd_ts > image_ts) {
                      pcd_que_.put_front(pcd_msg);
                      break;
                    }
                  } else {
                    RCLCPP_ERROR(this->get_logger(), "=====failed to get pcd, so we only save image, ts: %f ====", image_ts * 1e-9);
                    save_image(image_ts, img_msg);
                    saved = true;
                    break;
                  }
                }
              } else {
                RCLCPP_ERROR(this->get_logger(), "=====failed to get image====");
                break;
              }
            }
            RCLCPP_WARN(this->get_logger(), "we have saved %d images and %d pcds, and %d imus.",
                        image_save_cnt_.load(), pcd_save_cnt_.load(), imu_save_cnt_.load());
          }
        }
        RCLCPP_WARN(this->get_logger(), "snap_shot exit.");
      };
      save_threads_.emplace_back(std::make_shared<std::thread>(snap_func));
    } else {
      auto get_pause_func = [this]() {
        RCLCPP_WARN(this->get_logger(), "get_pause_func start.");
        while (rclcpp::ok()) {
          sensor_msgs::msg::Image::SharedPtr img_msg = nullptr;
          sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg = nullptr;
          if (!kbhit()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
          }
          int c;
          c = getchar();
          std::cout << "user input: " << c << std::endl;
          if (c == '\n') {
            is_paused_ = !is_paused_;
          }
          if (is_paused_) {
            std::cout << "\rdata saving stop!!!                        " << std::flush;
          } else {
            std::cout << "\rdata saving starts!!!                        " << std::flush;
          }
        }
        RCLCPP_WARN(this->get_logger(), "get_pause_func exit.");
      };
      if (enable_pause)
        save_threads_.emplace_back(std::make_shared<std::thread>(get_pause_func));
    }
  }

  ~ROS2DataCollection() {
    for (auto &t : save_threads_) {
      t->join();
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "quit pcd queue save thread, left: " << pcd_que_.size());
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "quit image queue save thread, left: " << image_que_.size());
    save_threads_.clear();
    if (imu_file_.is_open()) {
      imu_file_.close();
    }
    if (log_file_.is_open()) {
      log_file_.close();
    }
    RCLCPP_WARN(this->get_logger(), "we have saved %d images and %d pcds, and %d imus.",
                image_save_cnt_.load(), pcd_save_cnt_.load(), imu_save_cnt_.load());
  }

 private:
  std::string data_dir_, imu_dir_, image_dir_, pcd_dir_;
  std::string imu_filename_;
  std::string image_format_;
  std::ofstream imu_file_;
  std::ofstream log_file_;
  std::atomic_uint32_t pcd_save_cnt_{0}, image_save_cnt_ {0}, imu_save_cnt_ {0};
  float gravity_ = 9.81;
  bool snap_shot_{false}, motion_detect_ {false}, check_camera_sync_{true}, check_lidar_exist_{true};
  std::atomic_bool save_pcd_bin_ {false};
  int image_gap_mode_ = 0;
  int lidar_gap_mode_ = 0;
  double get_shot_time_;
  std::atomic_bool is_paused_{false};

  std::atomic_int32_t last_get_pcd_time_{0};

  std::shared_ptr<MotionDetector> motion_detector_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr status_image_pub_;

  blockqueue<sensor_msgs::msg::Image::SharedPtr> image_que_;
  blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr> pcd_que_;

  std::vector<std::shared_ptr<std::thread>> save_threads_;

  float get_free_space(const std::string &path, float &capacity) {
    try {
      std::filesystem::space_info si = std::filesystem::space(path);
      if (si.capacity == 0) {
        return 0.0;
      }
      std::cout << "Path:  '" << path << "' space situation: " << std::endl;
      std::cout << "Capacity:  " << si.capacity / 1024 / 1024 / 1024 << " GB\n";
      capacity = si.capacity / 1024 / 1024 / 1024;
      std::cout << "Free:      " << si.free / 1024 / 1024 / 1024 << " GB\n";
      std::cout << "Available: " << si.available / 1024 / 1024 / 1024 << " GB\n";
      return static_cast<float>(si.available) / static_cast<float>(si.capacity);
    } catch (const std::exception& e) {
      std::cerr << "[ERROR] : " << e.what() << std::endl;
      return 0.0;
    }
  }

  std::string generate_timestamp_folder() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H.%M.%S");

    return std::string( + "/ros2_data/" + oss.str());
  }

  void generate_device_info(const std::string &calib_file, const std::string &log_str) {
    const std::string device_info_file = data_dir_ + "/device.info";
    const std::string generate_cmd =
        R"SH(printf "ip: %s\nuname: %s\n" "$(ifconfig eth0 | grep 'inet ' | awk '{print $2}')" "$(uname -a)" > )SH"
        + device_info_file + "; sync;";
    std::cout << "generate device info cmd: \n" << generate_cmd << std::endl;
    system(generate_cmd.c_str());
    std::ofstream device_info_file_strem(device_info_file, std::ios::app);
    device_info_file_strem << "parameters:" << std::endl << log_str << std::endl;
    device_info_file_strem.close();

    if (!calib_file.empty() && std::filesystem::is_regular_file(calib_file)) {
      std::filesystem::path dst_path = std::filesystem::path(data_dir_) / std::filesystem::path(calib_file).filename();
      std::filesystem::copy_file(calib_file, dst_path, std::filesystem::copy_options::overwrite_existing);
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!imu_file_.is_open()) return;

    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    imu_file_ << timestamp << ","
              << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
              << msg->linear_acceleration.x * gravity_ << "," << msg->linear_acceleration.y * gravity_ << "," << msg->linear_acceleration.z * gravity_
              << std::endl;
    if (motion_detect_) {
      ImuData imu_data(timestamp);
      imu_data.wx = msg->angular_velocity.x, imu_data.wy = msg->angular_velocity.y, imu_data.wz = msg->angular_velocity.z;
      imu_data.ax = msg->linear_acceleration.x, imu_data.ay = msg->linear_acceleration.y, imu_data.az = msg->linear_acceleration.z;
      motion_detector_->FeedImu(imu_data);
    }
    imu_file_.flush();
    ++imu_save_cnt_;
  }

  static bool is_gap(const builtin_interfaces::msg::Time &time, int gap_mode,
                     uint64_t last_timestamp, bool check_sync = false) {
    if (gap_mode <= 0) {
      return false;
    }
    double current_time = time.sec + time.nanosec * 1e-9;
    double last_time = last_timestamp * 1e-9;
    if (current_time - last_time < gap_mode) {
      return true;
    }
    return check_sync && ((time.nanosec / 1000000) % 100 != 0);
  }

  void image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    static uint64_t last_timestamp, last_save_timestamp;
    static uint64_t lost_cnt;
    auto now = this->now().seconds();
    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

    std::stringstream stringstream;
    std::string log_string;
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "get image at %fs, msg ts is: %fs, diff: %fms",
                now, timestamp * 1e-9, now * 1e3 - timestamp * 1e-6);

    if (status_image_pub_->get_subscription_count() > 0) {
      int baseline = 0;
      cv::Point org = cv::Point(40, 120);
      //std::string show_text = "OK,image:" + std::to_string(image_save_cnt_) + ",pcd:" + std::to_string(pcd_save_cnt_);
      std::string show_text = "OK";
      auto dst = std::make_shared<sensor_msgs::msg::Image>(*msg);
      cv::Mat nv12_img = cv::Mat(dst->height, dst->width, CV_8UC1, dst->data.data());
      if (check_lidar_exist_ &&
          std::abs(dst->header.stamp.sec - last_get_pcd_time_.load()) > 3) {
        //  no Lidar msg received
        show_text = "NO LeiDa";
      } else if (check_camera_sync_ &&
                (dst->header.stamp.nanosec / 1000000) % 100 != 0) {
        //  camera is not sync
        show_text = "NO Tongbu";
      } else if (motion_detect_) {
        double a_var, w_var;
        uint64_t nearest_imu_timestamp;
        auto s = std::chrono::high_resolution_clock::now();
        MotionDetector::Status motion_status = motion_detector_->GetStatus(
            timestamp, nearest_imu_timestamp, a_var, w_var);
        auto e = std::chrono::high_resolution_clock::now();
        std::cout << "Motion GetStatus consume: " <<
                      std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count()
                  << std::endl;
        if (motion_status != MotionDetector::STATIC) {
          //  body is in motion, not in static
          show_text = "NO JingZhi";
          if (motion_status == MotionDetector::INIT) {

          }
          if (motion_status == MotionDetector::IMU_TOO_FAR) {

          }
          if (motion_status == MotionDetector::ENTERING_MOTION) {

          }
          if (motion_status == MotionDetector::ENTERING_STATIC) {

          }
        }
      }

      //std::cout << "\rshow_text: " << show_text << std::endl;
      //std::cout << "msg->header.stamp.nanosec: " << msg->header.stamp.nanosec << std::endl;
      //std::cout << "msg->header.stamp.nanosec / 1000000: " << msg->header.stamp.nanosec / 1000000 << std::endl;
      cv::Size textSize = cv::getTextSize(show_text, cv::FONT_HERSHEY_TRIPLEX, 3.5, 4, &baseline);

      cv::Point bl = cv::Point(org.x - 4, org.y + 4);
      cv::Point tr = cv::Point(org.x + textSize.width + 4, org.y - textSize.height - 4);
      cv::rectangle(nv12_img, bl, tr, CV_RGB(0, 0, 0), cv::FILLED);
      cv::putText(nv12_img, show_text, org,
                  cv::FONT_HERSHEY_TRIPLEX, 3.5, CV_RGB(255, 255, 255), 4);
      status_image_pub_->publish(*dst);
    }

    if (last_timestamp != 0) {
      double diff = (timestamp - last_timestamp) * 1e-9;
      if (diff < 0) {
        stringstream << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
        log_string = stringstream.str();
        log_file_ << log_string << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
      } else if (diff > 0.18 && !image_gap_mode_) {
        lost_cnt++;
        stringstream << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff
                     << "s, larger than 0.18s, the data get lost! lost count: " << lost_cnt;
        log_string = stringstream.str();
        log_file_ << log_string << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
      }
    }
    last_timestamp = timestamp;

    if (is_gap(msg->header.stamp, image_gap_mode_, last_save_timestamp, check_camera_sync_)) {
      return;
    }

    last_save_timestamp = timestamp;

    if (!is_paused_) {
      int sz = image_que_.put(msg);
      if (sz > 10) {
        if (!snap_shot_) {
          stringstream.clear();
          stringstream << std::fixed << "[image] que is larger than 10: " << sz;
          log_string = stringstream.str();
          log_file_ << log_string << std::endl;
          RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
        } else {
          image_que_.pop_front();
        }
      }
    } else {
      std::cout << "\rdata saving is stop!!!                        " << std::flush;
    }
  }

  void pcd_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static uint64_t last_timestamp, last_save_timestamp;
    static uint64_t lost_cnt;
    auto now = this->now().seconds();
    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    last_get_pcd_time_ = msg->header.stamp.sec;
    std::stringstream stringstream;
    std::string log_string;

    RCLCPP_INFO(this->get_logger(),
                "get pcd at %fs, msg ts is: %fs, diff: %fms",
                now, timestamp * 1e-9, now * 1e3 - timestamp * 1e-6);

    if (!rclcpp::ok()) {
      return;
    }
    if (last_timestamp != 0) {
      double diff = (timestamp - last_timestamp) * 1e-9;
      if (diff < 0) {
        stringstream << std::fixed << "[pcd] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
        log_string = stringstream.str();
        log_file_ << log_string << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
      } else if (diff > 0.18  && !lidar_gap_mode_) {
        lost_cnt++;
        stringstream << std::fixed << "[pcd] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff
                     << "s, larger than 0.18s, the data get lost! lost count: " << lost_cnt;
        log_string = stringstream.str();
        log_file_ << log_string << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
      }
    }
    last_timestamp = timestamp;

    if (is_gap(msg->header.stamp, lidar_gap_mode_, last_save_timestamp)) {
      return;
    }
    last_save_timestamp = timestamp;

    if (!is_paused_) {
      int sz = pcd_que_.put(msg);
      if (sz > 10) {
        if (!snap_shot_) {
          stringstream.clear();
          stringstream << std::fixed << "[pcd] que is larger than 10: " << sz;
          log_string = stringstream.str();
          log_file_ << log_string << std::endl;
          RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
        } else {
          pcd_que_.pop_front();
        }
      }
    }
  }

  void save_image(int64_t timestamp, const sensor_msgs::msg::Image::SharedPtr msg) {
    if (image_format_ == "yuv") {
      std::string filename = image_dir_ + "/" + std::to_string(timestamp) + ".yuv";
      std::ofstream file(filename, std::ios::out | std::ios::binary);
      if (file.is_open()) {
        file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
        file.close();
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "cannot save: " << filename);
      }
    } else {
      cv::Mat nv12(msg->height * 3 / 2, msg->width, CV_8UC1, msg->data.data());
      cv::Mat bgr;
      cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
      cv::imwrite(image_dir_ + "/" + std::to_string(timestamp) + ".png", bgr);
    }
    image_save_cnt_++;
  }

  void save_pcd(int64_t timestamp, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const int N = msg->width;
    std::string filename = pcd_dir_ + "/" + std::to_string(timestamp) + ".pcd";
    std::ofstream file;
    if (save_pcd_bin_) {
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

    if (msg->point_step != 22) {
      std::cout << "point_step of pointcloud2 is " << msg->point_step
                << ", rather than 22, so save pcd as asiic" << std::endl;
      save_pcd_bin_ = false;
    }

    if (save_pcd_bin_) {
      file << "DATA binary\n";
      file.write(reinterpret_cast<const char *>(msg->data.data()), N * 22);
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
             << (int)*iter_tag << " " << (int)*iter_line << std::endl;
      }
    }

    file.close();
    pcd_save_cnt_++;
  }

  void save_pcd_thread() {
    while (rclcpp::ok() || pcd_que_.size() > 0) {
      sensor_msgs::msg::PointCloud2::SharedPtr msg;
      if (pcd_que_.get(msg)) {
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        if (!is_paused_) {
          save_pcd(timestamp, msg);
        }
      }
    }
  }

  void save_image_thread() {
    while (rclcpp::ok() || image_que_.size() > 0) {
      sensor_msgs::msg::Image ::SharedPtr msg;
      if (image_que_.get(msg)) {
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        if (is_paused_) {
          std::cout << "\rdata saving is stop!!!                        " << std::flush;
        } else {
          save_image(timestamp, msg);
          std::cout << "\rwe have saved image: "
                    << image_save_cnt_.load() << ", pcd: " << pcd_save_cnt_.load()
                    << ", imu: " << imu_save_cnt_.load() << std::flush;
        }
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3);

  auto node = std::make_shared<ROS2DataCollection>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
