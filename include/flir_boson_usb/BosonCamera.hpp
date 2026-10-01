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

#include <flir_boson_usb/pipeline.hpp>

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

//! \brief A feature the driver can either detect by itself or be told about explicitly.
enum class TriState {
  Auto,
  Yes,
  No,
};

class BosonCamera
#ifdef ROS2
    : public rclcpp::Node
#else
: public cras::Nodelet
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
  //! \brief Turn the parameter strings into the typed state. Returns false when a value cannot be used.
  bool validateParams();
  /**
   * \brief Turn a palette name into the colormap the heatmap is painted with.
   *
   * The name is matched case-insensitively. The name and its typed form are updated together, and both are
   * left untouched when the name is not a palette of the built OpenCV.
   * \note The caller has to hold mutex_ when other threads can already read the parameters.
   * \return False when the name is not a known palette.
   */
  bool setColormap(const std::string& name);
  /**
   * \brief Turn a name into the content stamped on the heatmap.
   *
   * The name is matched case-insensitively. The name and its typed form are updated together, and both are
   * left untouched when the name is not a supported overlay content.
   * \note The caller has to hold mutex_ when other threads can already read the parameters.
   * \return False when the name is not a supported overlay content.
   */
  bool setOverlayMode(const std::string& name);
  /**
   * \brief Turn a unit name into the unit the absolute temperature image is carried in.
   *
   * The name is matched case-insensitively. The name, its typed form and the image encoding that goes with
   * the unit are updated together, and all three are left untouched when the name is not a supported unit.
   * \note The caller has to hold mutex_ when other threads can already read the parameters.
   * \return False when the name is not a known unit.
   */
  bool setTempMode(const std::string& name);
  void init();
  bool openCamera();
  bool closeCamera();
  void captureAndPublish();
  bool detectDevice();
  void resolveRadiometric();
  void createPublishers();

  //! \brief Collect the current parameters into the configuration handed to the processing stage.
  PipelineConfig pipelineConfig() const;
  //! \brief Push the current parameters into the processing stage.
  void applyPipelineConfig();
  //! \brief The size of the published image, which is not the sensor size when zoom is enabled.
  cv::Size publishedSize() const;
  //! \brief Whether the temperature probe lies inside the published image.
  bool probeInBounds() const;

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
  image_transport::Publisher image_pub_visual_, image_pub_heatmap_, image_pub_temp_;
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
  bool is_yv12_;

  // OpenCV Mats (Pre-allocated to prevent memory churn)
  cv::Mat thermal16_, thermal16_zoom_, thermal8_, thermal_rgb_;

  // The image processing itself, which does not know anything about ROS.
  Pipeline pipeline_;

  Temperature max_temp_msg_, min_temp_msg_, ptr_temp_msg_;

  // Parameters
  std::string frame_id_, dev_path_, camera_info_url_, video_mode_str_, sensor_type_str_;
  std::string radiometric_str_, agc_mode_str_, heatmap_mode_str_, temp_mode_str_;
  std::vector<std::string> radiometric_patterns_;
  double frame_rate_;
  Encoding video_mode_;
  bool zoom_enable_;
  bool yuv_color_;
  SensorTypes sensor_type_;
  TriState radiometric_request_;
  // Resolved from radiometric_request_ and the device identification once the camera is open.
  bool radiometric_;
  std::string device_id_, device_serial_;

  // The typed form of the preset parameters above.
  AgcMode agc_mode_;
  HeatmapMode heatmap_mode_;
  TempMode temp_mode_;
  //! \brief The image encoding that goes with temp_mode_; the publishers have no access to the stage.
  std::string temp_encoding_;

  // Dynamic parameters
  //! \brief The temperature probe point, in pixels of the published image; the point of the ptr reading.
  int temp_ptr_x_;
  int temp_ptr_y_;
  //! \brief The stretch bounds of the fixed-range AGC, in degrees Celsius.
  int agc_fixed_max_temp_, agc_fixed_min_temp_;
  bool agc_norm_;
  //! \brief The clip percentiles of the auto-range AGC.
  double agc_auto_low_pct_, agc_auto_high_pct_;
  //! \brief How much wider the observed bounds are taken when agc_norm is on.
  double agc_norm_margin_;
  //! \brief The palette name and its typed form; setColormap() keeps them in sync.
  std::string colormap_str_;
  Colormap colormap_;
  //! \brief The overlay content name and its typed form; setOverlayMode() keeps them in sync.
  std::string overlay_mode_str_;
  OverlayMode overlay_mode_;
  mutable std::mutex mutex_;
};

}  // namespace flir_boson_usb
