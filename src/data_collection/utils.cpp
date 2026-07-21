#include "utils.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace data_collection {

float get_free_space(const std::string& path, float& capacity_gb) {
  try {
    auto si = std::filesystem::space(path);
    if (si.capacity == 0) {
      return 0.0f;
    }
    auto gb = [](auto bytes) { return bytes / 1024.0 / 1024.0 / 1024.0; };
    std::cout << "Path:  '" << path << "' space situation: " << std::endl;
    std::cout << "Capacity:  " << gb(si.capacity) << " GB\n";
    std::cout << "Free:      " << gb(si.free) << " GB\n";
    std::cout << "Available: " << gb(si.available) << " GB\n";
    capacity_gb = static_cast<float>(gb(si.capacity));
    return static_cast<float>(si.available) / static_cast<float>(si.capacity);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] : " << e.what() << std::endl;
    return 0.0f;
  }
}

std::string get_disk_info(const std::string& path) {
  std::stringstream info;
  try {
    auto si = std::filesystem::space(path);
    auto gb = [](auto bytes) { return bytes / 1024.0 / 1024.0 / 1024.0; };
    info << std::fixed << std::setprecision(1) << gb(si.capacity) - gb(si.available) << "G/" << gb(si.capacity) << "G ("
         << (1 - gb(si.available)/gb(si.capacity)) * 100 << "%)";
    return info.str();
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] : " << e.what() << std::endl;
    return "0/0";
  }
}

std::string generate_timestamp_folder() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d-%H.%M.%S");
  return "/ros2_data/" + oss.str();
}

void generate_device_info(const std::string& data_dir,
                          const std::string& calib_file,
                          const std::string& log_str) {
  const std::string device_info_file = data_dir + "/device.info";
  const std::string generate_cmd =
      R"SH({ printf "ip: %s\nuname: %s\n" "$(ifconfig eth0 | grep 'inet ' | awk '{print $2}')" "$(uname -a)"; echo -n "isp tuning file md5: "; md5sum /usr/hobot/lib/sensor/ox02c1s_tuning.json; } > )SH"
      + device_info_file + "; sync;";
  std::cout << "generate device info cmd: \n" << generate_cmd << std::endl;
  system(generate_cmd.c_str());
  std::ofstream ofs(device_info_file, std::ios::app);
  ofs << "parameters:" << std::endl << log_str << std::endl;
  ofs.close();

  if (!calib_file.empty() && std::filesystem::is_regular_file(calib_file)) {
    auto dst_path = std::filesystem::path(data_dir) / std::filesystem::path(calib_file).filename();
    std::filesystem::copy_file(calib_file, dst_path, std::filesystem::copy_options::overwrite_existing);
  }
}

bool is_gap(const builtin_interfaces::msg::Time& time, int gap_mode,
            uint64_t last_timestamp, bool check_sync) {
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

}  // namespace data_collection
