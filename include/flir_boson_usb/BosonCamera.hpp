// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2018 FLIR Systems, INC
// SPDX-FileCopyrightText: 2018-2019 AutonomouStuff, LLC
// SPDX-FileCopyrightText: Czech Technical University in Prague

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

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>

#ifdef ROS2

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>

#else

#include <camera_info_manager/camera_info_manager.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <dynamic_reconfigure/server.h>
#include <flir_boson_usb/BosonCameraConfig.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Header.h>

#endif

namespace flir_boson_usb {

#ifdef ROS2
using Header = std_msgs::msg::Header;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Temperature = sensor_msgs::msg::Temperature;
#else
using Header = std_msgs::Header;
using CameraInfo = sensor_msgs::CameraInfo;
using Temperature = sensor_msgs::Temperature;
#endif

enum class Encoding {
  YUV = 0,
  RAW16 = 1,
};

enum class SensorTypes {
  Boson320,
  Boson640,
};

class BosonCamera :
#ifdef ROS2
  public rclcpp::Node
#else
  public cras::Nodelet
#endif
{
public:
#ifdef ROS2
  explicit BosonCamera(const rclcpp::NodeOptions& options);
#else
  BosonCamera();
#endif

  ~BosonCamera() override;

private:
  void validateParams();
  void init();
  bool openCamera();
  bool closeCamera();
  void captureAndPublish();
  std::string detectSerial() const;

  // Custom processing utilitiesC
  void agc(
      const cv::Mat& input_16, cv::Mat& output_8, cv::Mat& output_16, double clip_low_pct, double clip_high_pct,
      double* max_temp, double* min_temp);

#ifdef ROS2
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  PostSetParametersCallbackHandle::SharedPtr params_cb_;
#else
  // Nodelet entry point: reads the parameters and calls init().
  void onInit() override;

  // ROS 1 handles have to be declared before the publishers so that the
  // publishers (and the image transport using them) are destroyed first.
  ros::NodeHandle nh_, pnh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ros::Timer capture_timer_;

  // Dynamically reconfigurable parameters
  std::shared_ptr<dynamic_reconfigure::Server<flir_boson_usb::BosonCameraConfig>> reconfigure_server_;
  void reconfigureCallback(flir_boson_usb::BosonCameraConfig& config, uint32_t level);
#endif

  // ROS Node variables
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_;
  image_transport::CameraPublisher image_pub_;
  image_transport::Publisher image_pub_8_, image_pub_heatmap_, image_pub_temp_, image_pub_8_norm_;
#ifndef ROS2
  ros::Publisher max_temp_pub_, min_temp_pub_, ptr_temp_pub_;
#else
  rclcpp::Publisher<Temperature>::SharedPtr max_temp_pub_, min_temp_pub_, ptr_temp_pub_;
#endif

  // Hardware V4L2 variables
  int32_t width_, height_, fd_;
  v4l2_capability cap_;

  struct V4L2Buffer {
    void* start;
    size_t length;
  };
  std::vector<V4L2Buffer> buffers_;  // 4-buffer Ring Queue
  int expected_height_;
  size_t bytesperline_;
  double max_temp_, min_temp_, ptr_temp_;

  cv::Point temp_ptr_;

  // OpenCV Mats (Pre-allocated to prevent memory churn)
  cv::Mat thermal16_, thermal16_linear_, thermal16_linear_zoom_, thermal8_linear_, thermal8_linear_zoom_,
    thermal8_heatmap_, thermal8_temp_, thermal8_norm_, thermal_rgb_, hist_, thermal_rgb_zoom_, thermal_luma_;

  Temperature max_temp_msg_, min_temp_msg_, ptr_temp_msg_;

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

  // Dynamic parameters
  int point_x_, point_y_;
  int max_temp_limit_, min_temp_limit_;
  double norm_margin_;
  std::mutex mutex_;
};

}  // namespace flir_boson_usb
