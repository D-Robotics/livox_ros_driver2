#pragma once

#include <string>
#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>

namespace data_collection {

float get_free_space(const std::string& path, float& capacity_gb);
std::string get_disk_info(const std::string& path);
std::string generate_timestamp_folder();

void generate_device_info(const std::string& data_dir,
                          const std::string& calib_file,
                          const std::string& log_str);

bool is_gap(const builtin_interfaces::msg::Time& time, int gap_mode,
            uint64_t last_timestamp, bool check_sync = false);

}  // namespace data_collection
