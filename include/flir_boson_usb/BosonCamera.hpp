/*
 * Copyright (c) 2018 FLIR Systems, INC
 * Copyright (c) 2018-2019 AutonomouStuff, LLC
 * * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef FLIR_BOSON_USB_BOSONCAMERA_HPP
#define FLIR_BOSON_USB_BOSONCAMERA_HPP

#include <string>
#include <cmath>
#include <memory>
#include <vector>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace flir_boson_usb
{

enum Encoding {
  YUV = 0,
  RAW16 = 1,
};

enum SensorTypes { Boson320, Boson640 };

class BosonCamera : public rclcpp::Node
{
public:
  explicit BosonCamera(const rclcpp::NodeOptions & options);
  ~BosonCamera() override;

private:
  void init();
  bool openCamera();
  bool closeCamera();
  void captureAndPublish();

  // Custom processing utilities
  void agc(const cv::Mat& input_16, cv::Mat& output_8, cv::Mat& output_16, double clip_low_pct, double clip_high_pct);

  // ROS Node variables
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_;
  image_transport::CameraPublisher image_pub_;
  image_transport::Publisher image_pub_8_, image_pub_color_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  // Hardware V4L2 variables
  int32_t width_, height_, fd_;
  struct v4l2_capability cap_;

  struct V4L2Buffer {
      void* start;
      size_t length;
  };
  std::vector<V4L2Buffer> buffers_; // 4-buffer Ring Queue
  int expected_height_;
  size_t bytesperline_;

  // OpenCV Mats (Pre-allocated to prevent memory churn)
  cv::Mat thermal16_, thermal16_linear_, thermal16_linear_zoom_, thermal8_linear_, thermal8_linear_zoom_, thermal8_color_, thermal_rgb_, hist_, thermal_rgb_zoom_, thermal_luma_;

  // Parameters
  std::string frame_id_, dev_path_, camera_info_url_, video_mode_str_, sensor_type_str_;
  double frame_rate_;
  Encoding video_mode_;
  bool zoom_enable_;
  bool publish_color_;
  bool is_yv12_;
  double raw16_agc_low_pct_;
  double raw16_agc_high_pct_;
  SensorTypes sensor_type_;
};

}  // namespace flir_boson_usb

#endif  // FLIR_BOSON_USB_BOSONCAMERA_HPP