#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <deque>

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
    int save_thread_num = this->declare_parameter("save_thread_num", 2);;

    // 生成当前时间文件夹
    data_dir_ = home_dir + generateTimestampFolder();
    image_dir_ = data_dir_ + "/image/";
    pcd_dir_ = data_dir_ + "/pcd/";
    imu_dir_ = data_dir_ + "/imu/";
    system(("mkdir -p " + data_dir_).c_str());
    system(("mkdir -p " + image_dir_).c_str());
    system(("mkdir -p " + pcd_dir_).c_str());
    system(("mkdir -p " + imu_dir_).c_str());

    RCLCPP_INFO(this->get_logger(),
                "data_dir: %s\n"
                "imu_topic: %s, lidar_topic:%s, image_topic: %s",
                data_dir_.c_str(), imu_topic.c_str(), lidar_topic.c_str(), image_topic.c_str());

    // 创建 IMU 数据文件
    imu_filename_ = imu_dir_ + "/imu_data.txt";
    imu_file_.open(imu_filename_, std::ios::out | std::ios::app);  // 追加模式
    if (!imu_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open IMU file: %s", imu_filename_.c_str());
    }

    log_file_.open(data_dir_ + "/log.txt", std::ios::out);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", (data_dir_ + "/log.log").c_str());
    }

    for (int i = 0; i < save_thread_num; ++i) {
      save_threads_.emplace_back(std::make_shared<std::thread>(
          std::bind(&ROS2DataCollection::save_pcd_thread, this)));
      save_threads_.emplace_back(std::make_shared<std::thread>(
          std::bind(&ROS2DataCollection::save_image_thread, this)));
    }

    // 订阅 IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 1000, std::bind(&ROS2DataCollection::imu_callback, this, std::placeholders::_1));

    // 订阅 NV12 图像
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 30, std::bind(&ROS2DataCollection::image_callback, this, std::placeholders::_1));

    // 订阅 PointCloud2
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 30, std::bind(&ROS2DataCollection::pcd_callback, this, std::placeholders::_1));
  }

  ~ROS2DataCollection() {
    save_threads_.clear();
    if (imu_file_.is_open()) {
      imu_file_.close();
    }
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

 private:
  std::string data_dir_, imu_dir_, image_dir_, pcd_dir_;
  std::string imu_filename_;
  std::ofstream imu_file_;
  std::ofstream log_file_;
  float gravity_ = 9.81;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

  blockqueue<sensor_msgs::msg::Image::SharedPtr> image_que_;
  blockqueue<sensor_msgs::msg::PointCloud2::SharedPtr> pcd_que_;

  std::vector<std::shared_ptr<std::thread>> save_threads_;

  std::string generateTimestampFolder() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");

    return std::string( + "/ros2_data/" + oss.str());
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!imu_file_.is_open()) return;

    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    imu_file_ << timestamp << ","
              << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
              << msg->linear_acceleration.x * gravity_ << "," << msg->linear_acceleration.y * gravity_ << "," << msg->linear_acceleration.z * gravity_
              << "\n";
    imu_file_.flush();  // 确保数据及时写入
  }

  void image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    static uint64_t last_timestamp;
    static uint64_t lost_cnt;
    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    std::stringstream stringstream;
    if (last_timestamp != 0) {
      double diff = (timestamp - last_timestamp) * 1e-9;
      if (diff < 0) {
        stringstream << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
        << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
        log_file_ << stringstream.rdbuf() << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), stringstream.rdbuf());
      } else if (diff > 0.18) {
        lost_cnt++;
        stringstream << std::fixed << "[image] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff
                     << "s, larger than 0.18s, the data get lost! lost count: " << lost_cnt;
        log_file_ << stringstream.rdbuf() << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), stringstream.rdbuf());
      }
    }
    last_timestamp = timestamp;
    int sz = image_que_.put(msg);
    if (sz > 10) {
      stringstream.clear();
      stringstream << std::fixed << "[image] que is larger than 10: " << sz;
      log_file_ << stringstream.rdbuf() << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), stringstream.rdbuf());
    }
  }

  void pcd_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static uint64_t last_timestamp;
    static uint64_t lost_cnt;
    uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    std::stringstream stringstream;
    if (last_timestamp != 0) {
      double diff = (timestamp - last_timestamp) * 1e-9;
      if (diff < 0) {
        stringstream << std::fixed << "[pcd] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff << "s";
        log_file_ << stringstream.rdbuf() << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), stringstream.rdbuf());
      } else if (diff > 0.18) {
        lost_cnt++;
        stringstream << std::fixed << "[pcd] last timestamp is: " << last_timestamp * 1e-9
                     << ", current timestamp is: " << timestamp * 1e-9 << ", diff is: " << diff
                     << "s, larger than 0.18s, the data get lost! lost count: " << lost_cnt;
        log_file_ << stringstream.rdbuf() << std::endl;
        RCLCPP_ERROR_STREAM(this->get_logger(), stringstream.rdbuf());
      }
    }
    last_timestamp = timestamp;
    int sz = pcd_que_.put(msg);
    if (sz > 10) {
      stringstream.clear();
      stringstream << std::fixed << "[pcd] que is larger than 10: " << sz;
      log_file_ << stringstream.rdbuf() << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), stringstream.rdbuf());
    }
  }

  void save_pcd(const std::string& filename, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    file << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity offset_time\n"
         << "SIZE 4 4 4 4 4\n"
         << "TYPE F F F F U\n"
         << "COUNT 1 1 1 1 1\n"
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
           << *iter_intensity << " " << *iter_offset_time << "\n";
    }
    file.close();
  }

  void save_pcd_thread() {
    while (rclcpp::ok()) {
      sensor_msgs::msg::PointCloud2::SharedPtr msg;
      if (pcd_que_.get(msg)) {
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        std::string filename = pcd_dir_ + "/" + std::to_string(timestamp) + ".pcd";
        save_pcd(filename, msg);
      }
    }
  }

  void save_image_thread() {
    while (rclcpp::ok()) {
      sensor_msgs::msg::Image ::SharedPtr msg;
      if (image_que_.get(msg)) {
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        std::string filename = image_dir_ + "/" + std::to_string(timestamp) + ".bin";
        std::ofstream file(filename, std::ios::out | std::ios::binary);
        if (file.is_open()) {
          file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
          file.close();
        } else {
          std::cout << "cannot save: " << filename << std::endl;
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