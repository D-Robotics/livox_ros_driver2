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

#include "image_motion_detector.h"

namespace auto_collect {

ImageMotionDetector::ImageMotionDetector(double diff_pixel_ratio_th, int diff_gray_th)
    : diff_pixel_ratio_th_(diff_pixel_ratio_th), diff_gray_th_(diff_gray_th) {
}

void ImageMotionDetector::Reset() {
  has_reference_ = false;
  reference_gray_.release();
}

bool ImageMotionDetector::ConvertToGray(const sensor_msgs::msg::Image::SharedPtr &msg, cv::Mat *gray) {
  if (!msg || gray == nullptr || msg->data.empty()) {
    return false;
  }

  if (msg->encoding == "mono8") {
    *gray = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t *>(msg->data.data())).clone();
    return true;
  }

  if (msg->encoding == "bgr8") {
    cv::Mat bgr(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    cv::cvtColor(bgr, *gray, cv::COLOR_BGR2GRAY);
    return true;
  }

  if (msg->encoding == "rgb8") {
    cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    cv::cvtColor(rgb, *gray, cv::COLOR_RGB2GRAY);
    return true;
  }

  cv::Mat nv12(msg->height * 3 / 2, msg->width, CV_8UC1, const_cast<uint8_t *>(msg->data.data()));
  cv::Mat bgr;
  cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
  cv::cvtColor(bgr, *gray, cv::COLOR_BGR2GRAY);

  return true;
}

bool ImageMotionDetector::SetReference(const sensor_msgs::msg::Image::SharedPtr &msg) {
  Reset();

  cv::Mat gray;
  if (!ConvertToGray(msg, &gray)) {
    return false;
  }

  reference_gray_ = gray;
  has_reference_ = true;
  return true;
}

bool ImageMotionDetector::IsMoving(const sensor_msgs::msg::Image::SharedPtr &msg, double *diff_ratio) {
  if (!has_reference_) {
    return false;
  }

  cv::Mat gray;
  if (!ConvertToGray(msg, &gray)) {
    return true;
  }

  if (gray.size() != reference_gray_.size()) {
    return true;
  }

  cv::Mat diff;
  cv::absdiff(reference_gray_, gray, diff);

  cv::Mat mask;
  cv::threshold(diff, mask, diff_gray_th_, 255, cv::THRESH_BINARY);

  const int changed_pixels = cv::countNonZero(mask);
  const int total_pixels = mask.rows * mask.cols;
  const double ratio = static_cast<double>(changed_pixels) / static_cast<double>(total_pixels);

  if (diff_ratio != nullptr) {
    *diff_ratio = ratio;
  }

  return ratio > diff_pixel_ratio_th_;
}

} // namespace auto_collect