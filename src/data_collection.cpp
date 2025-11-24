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

class ROS2DataCollection : public rclcpp::Node {
 public:
  ROS2DataCollection() : Node("ROS2DataCollection") {

    std::string home_dir = this->declare_parameter("home_dir", "/media/sda2/");;
    std::string imu_topic = this->declare_parameter("imu_topic", "/livox/imu");;
    std::string lidar_topic = this->declare_parameter("lidar_topic", "/livox/lidar");;
    std::string image_topic = this->declare_parameter("image_topic", "/image_combine_raw");;
    int save_thread_num = this->declare_parameter("save_thread_num", 4);;
    snap_shot_ = this->declare_parameter("snap_shot", false);
    gravity_ = this->declare_parameter("gravity", gravity_);
    gap_mode_ = this->declare_parameter("gap_mode", gap_mode_);

    data_dir_ = home_dir + generate_timestamp_folder();
    image_dir_ = data_dir_ + "/image/";
    pcd_dir_ = data_dir_ + "/pcd/";
    imu_dir_ = data_dir_ + "/imu/";
    system(("mkdir -p " + data_dir_).c_str());
    system(("mkdir -p " + image_dir_).c_str());
    system(("mkdir -p " + pcd_dir_).c_str());
    system(("mkdir -p " + imu_dir_).c_str());

    float space_ratio = 1 - get_free_space(home_dir);
    if (space_ratio > 0.9) {
      std::cout << std::fixed << std::setprecision(1)
                << "The disk of dir: '" << home_dir << "' is now at: "
                << space_ratio * 100 << "% usage, which is nearly full!" << std::endl;
      std::cout << "Please chose another directory or port over to another disk !!" << std::endl;
      //std::exit(-1);
    } else {
      std::cout << std::fixed << std::setprecision(1)
                << "The disk of dir: '" << home_dir << "' is now at: "
                << space_ratio * 100 << "% usage" << std::endl;
    }

    RCLCPP_WARN(this->get_logger(),
                "data_dir: %s\n"
                "imu_topic: %s, lidar_topic: %s, image_topic: %s\n"
                "snap_shot: %d, gravity_: %f, gap_mode: %d",
                data_dir_.c_str(), imu_topic.c_str(), lidar_topic.c_str(), image_topic.c_str(),
                snap_shot_, gravity_, gap_mode_);

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
            RCLCPP_WARN(this->get_logger(), "we have saved %d images and %d pcds.",
                        image_save_cnt_.load(), pcd_save_cnt_.load());
          }
        }
        RCLCPP_WARN(this->get_logger(), "snap_shot exit.");
      };
      save_threads_.emplace_back(std::make_shared<std::thread>(snap_func));
    }
  }

  ~ROS2DataCollection() {
    for (auto &t : save_threads_) {
      t->join();
    }
    save_threads_.clear();
    if (imu_file_.is_open()) {
      imu_file_.close();
    }
    if (log_file_.is_open()) {
      log_file_.close();
    }
    RCLCPP_WARN(this->get_logger(), "we have saved %d images and %d pcds.",
                image_save_cnt_.load(), pcd_save_cnt_.load());
  }

 private:
  std::string data_dir_, imu_dir_, image_dir_, pcd_dir_;
  std::string imu_filename_;
  std::ofstream imu_file_;
  std::ofstream log_file_;
  std::atomic_uint32_t pcd_save_cnt_{0}, image_save_cnt_ {0};
  float gravity_ = 9.81;
  bool snap_shot_;
  int gap_mode_ = 0;
  double get_shot_time_;

  std::atomic_int32_t last_get_pcd_time_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr status_image_pub_;

  blockqueue<sensor_msgs::msg::Image::SharedPtr> image_que_;
  blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr> pcd_que_;

  std::vector<std::shared_ptr<std::thread>> save_threads_;

  float get_free_space(const std::string &path) {
    try {
      std::filesystem::space_info si = std::filesystem::space(path);
      if (si.capacity == 0) {
        return 0.0;
      }
      std::cout << "Path:  " << path << " space situation: " << std::endl;
      std::cout << "Capacity:  " << si.capacity / 1024 / 1024 << " MB\n";
      std::cout << "Free:      " << si.free / 1024 / 1024 << " MB\n";
      std::cout << "Available: " << si.available / 1024 / 1024  << " MB\n";
      return static_cast<float>(si.available) / static_cast<float>(si.capacity);
    } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
      return 0.0;
    }
  }

  std::string generate_timestamp_folder() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H:%M:%S");

    return std::string( + "/ros2_data/" + oss.str());
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!imu_file_.is_open()) return;

    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    imu_file_ << timestamp << ","
              << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
              << msg->linear_acceleration.x * gravity_ << "," << msg->linear_acceleration.y * gravity_ << "," << msg->linear_acceleration.z * gravity_
              << std::endl;
    imu_file_.flush();
  }

  bool is_gap(const builtin_interfaces::msg::Time &time) const {
    std::vector<char> last_sec_v_1 {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    std::vector<char> last_sec_v_2 {'0', '2', '4', '6', '8'};
    std::vector<char> last_sec_v_5 {'0', '5'};
    std::vector<char> last_sec_v;
    if (gap_mode_ <= 0) {
      return false;
    }
    if (gap_mode_ == 1) {
      last_sec_v = last_sec_v_1;
    } else if (gap_mode_ == 2) {
      last_sec_v = last_sec_v_2;
    } else if (gap_mode_ == 5) {
      last_sec_v = last_sec_v_5;
    }
    char last_sec = std::to_string(time.sec).back();
    if ((int)(time.nanosec / 1e8) != 0) {
      return true;
    }
    return std::find(last_sec_v.begin(), last_sec_v.end(), last_sec) == last_sec_v.end();
  }

  void image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    static uint64_t last_timestamp;
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
      std::string show_text = "OK";
      cv::Mat nv12_img = cv::Mat(msg->height, msg->width, CV_8UC1, msg->data.data());
      if (std::abs(msg->header.stamp.sec - last_get_pcd_time_.load()) > 3
          ||  (msg->header.stamp.nanosec / 10000) % 100 != 0) {
        show_text = "NO";
      }
      cv::putText(nv12_img, show_text, cv::Point(40, 120),
                  cv::FONT_HERSHEY_TRIPLEX, 4.0, CV_RGB(255, 255, 255), 5);
      status_image_pub_->publish(*msg);
    }

    if (is_gap(msg->header.stamp)) {
      return;
    }

    if (last_timestamp != 0) {
      double diff = (timestamp - last_timestamp) * 1e-9;
      if (diff < 0) {
        stringstream << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
        log_string = stringstream.str();
        log_file_ << log_string << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), log_string);
      } else if (diff > 0.18 && !gap_mode_) {
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
  }

  void pcd_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static uint64_t last_timestamp;
    static uint64_t lost_cnt;
    auto now = this->now().seconds();
    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    last_get_pcd_time_ = msg->header.stamp.sec;
    std::stringstream stringstream;
    std::string log_string;

    if (is_gap(msg->header.stamp)) {
      return;
    }

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
      } else if (diff > 0.18  && !gap_mode_) {
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

  void save_image(int64_t timestamp, const sensor_msgs::msg::Image::SharedPtr msg) {
    std::string filename = image_dir_ + "/" + std::to_string(timestamp) + ".yuv";
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if (file.is_open()) {
      file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
      file.close();
      image_save_cnt_++;
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "cannot save: " << filename);
    }
    if (snap_shot_) {
      cv::Mat nv12(msg->height * 3 / 2, msg->width, CV_8UC1, msg->data.data());
      cv::Mat bgr;
      cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
      cv::imwrite(image_dir_ + "/" + std::to_string(timestamp) + ".png", bgr);
    }
  }

  void save_pcd(int64_t timestamp, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::string filename = pcd_dir_ + "/" + std::to_string(timestamp) + ".pcd";
    std::ofstream file(filename);
    if (!file.is_open()) return;

    file << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity offset_time tag line\n"
         << "SIZE 4 4 4 4 4 1 1\n"
         << "TYPE F F F F U U U\n"
         << "COUNT 1 1 1 1 1 1 1\n"
         << "WIDTH " << msg->width << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << msg->width << "\n"
         << "DATA ascii\n";

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
    file.close();
    pcd_save_cnt_++;
  }

  void save_pcd_thread() {
    while (rclcpp::ok() || pcd_que_.size() > 0) {
      sensor_msgs::msg::PointCloud2::SharedPtr msg;
      if (pcd_que_.get(msg)) {
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        save_pcd(timestamp, msg);
      }
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
        "quit pcd queue save thread, left: " << pcd_que_.size());
  }

  void save_image_thread() {
    while (rclcpp::ok() || image_que_.size() > 0) {
      sensor_msgs::msg::Image ::SharedPtr msg;
      if (image_que_.get(msg)) {
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        save_image(timestamp, msg);
        if (gap_mode_) {
          printf("\rwe have saved image: %d, pcd: %d",
              image_save_cnt_.load(), pcd_save_cnt_.load());
          std::cout << std::flush;
        }
      }
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "quit image queue save thread, left: " << image_que_.size());
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
