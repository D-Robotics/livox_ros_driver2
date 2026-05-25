// Copyright (c) 2026，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

namespace auto_collect {

class ImageMotionDetector {
public:
  /**
   * @brief The status of the image motion detector
   * @param diff_ratio The difference ratio between the current image and the reference image
   * @param diff_gray The difference gray value between the current image and the reference image
   */
  ImageMotionDetector(double diff_pixel_ratio_th, int diff_gray_th);

  /**
   * @brief Set the reference image
   * @param msg The reference image
   * @return True if the reference image is set successfully, false otherwise
   */
  bool SetReference(const sensor_msgs::msg::Image::SharedPtr &msg);

  /**
   * @brief Check if the image is moving
   * @param msg The current image
   * @param diff_ratio The difference ratio between the current image and the reference image
   * @return True if the image is moving, false otherwise
   */
  bool IsMoving(const sensor_msgs::msg::Image::SharedPtr &msg, double *diff_ratio = nullptr);

  /**
   * @brief Reset the image motion detector
   */
  void Reset();

private:
  /**
   * @brief Convert the image to gray
   * @param msg The image
   * @param gray The gray image
   * @return True if the conversion is successful, false otherwise
   */
  bool ConvertToGray(const sensor_msgs::msg::Image::SharedPtr &msg, cv::Mat *gray);

private:
  double diff_pixel_ratio_th_{0.02};
  int diff_gray_th_{25};
  bool has_reference_{false};
  cv::Mat reference_gray_;
};

} // namespace auto_collect