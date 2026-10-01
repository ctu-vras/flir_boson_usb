// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2018 FLIR Systems, INC
// SPDX-FileCopyrightText: 2018-2019 AutonomouStuff, LLC
// SPDX-FileCopyrightText: Czech Technical University in Prague

/*
 * Copyright (c) 2018 FLIR Systems, INC
 * Copyright (c) 2018-2019 AutonomouStuff, LLC
 * * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <flir_boson_usb/BosonCamera.hpp>

#ifdef ROS2
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <flir_boson_usb/BosonCameraConfig.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.hpp>
#endif

// To ease logging messages in ROS 1 and ROS 2, these defines work in ROS 2. ROS 1 is handled by the provided CRAS_*.
#ifdef ROS2
#define CRAS_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__);
#define CRAS_WARN(...) RCLCPP_WARN(this->get_logger(), __VA_ARGS__);
#define CRAS_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__);
#endif

namespace flir_boson_usb {

using namespace cv;

#ifdef ROS2

inline rcl_interfaces::msg::ParameterDescriptor paramDesc(
    const std::string& description, const std::string& additional_constraints = "") {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  desc.additional_constraints = additional_constraints;
  return desc;
}

inline rcl_interfaces::msg::ParameterDescriptor paramDescRangeI(
    const std::string& description, const int64_t start, const int64_t stop, const int64_t step = 0,
    const std::string& additional_constraints = "") {
  auto desc = paramDesc(description, additional_constraints);

  desc.integer_range.resize(1);
  auto& range = desc.integer_range[0];
  range.from_value = start;
  range.to_value = stop;
  range.step = step;

  return desc;
}

inline rcl_interfaces::msg::ParameterDescriptor paramDescRangeF(
    const std::string& description, const double start, const double stop, const double step = 0.0,
    const std::string& additional_constraints = "") {
  auto desc = paramDesc(description, additional_constraints);

  desc.floating_point_range.resize(1);
  auto& range = desc.floating_point_range[0];
  range.from_value = start;
  range.to_value = stop;
  range.step = step;

  return desc;
}

BosonCamera::BosonCamera(const rclcpp::NodeOptions & options)
    : Node("boson_camera", options), fd_(-1) {
  frame_id_ = this->declare_parameter("frame_id", "boson_camera", paramDesc("Frame used in header.frame_id"));
  dev_path_ = this->declare_parameter(
    "dev", "/dev/video0", paramDesc("the linux file descriptor location for the camera"));
  sensor_type_str_ = this->declare_parameter(
    "sensor_type", "Boson_640",
    paramDesc(
      "Physical sensor array size. Boson_320 or Boson_640. The driver cross-checks this against the V4L2-negotiated "
      "width and refuses to start on a mismatch.",
      "Boson_640|Boson_320"));
  const auto is640 = sensor_type_str_ == "Boson_640";
  frame_rate_ = this->declare_parameter(
    "frame_rate", 30.0, paramDescRangeF("Frame rate of the camera", 9.0, 60.0, 3.0, "Only 9.0/30.0/60.0 supported."));
  if (!is640) {
    zoom_enable_ = this->declare_parameter(
      "zoom_enable", false,
      paramDesc("Digital 2x upscale (320x256 -> 640x512) of the published image. Only available on Boson320 cameras."));
  }
  video_mode_str_ = this->declare_parameter(
    "video_mode", "YUV",
    paramDesc(
      "Camera image format. "
      "YUV: camera-side AGC (mono8/bgr8, low CPU). "
      "RAW16: raw 16-bit thermal counts (mono16, no camera processing).",
      "YUV|RAW16"));
  publish_color_ = this->declare_parameter("publish_color", false, paramDesc("Publish color images."));
  raw16_agc_low_pct_ = this->declare_parameter(
    "raw16_agc_low_pct", 1.0,
    paramDescRangeF(
      "Bottom-tail clip percentage for RAW16 driver-side AGC (e.g. 1.0 discards the darkest 1% of pixels before "
      "linear stretch). Valid range [0, 50); invalid values revert to 1.0.",
      0.0, 50.0));
  raw16_agc_high_pct_ = this->declare_parameter(
    "raw16_agc_high_pct", 1.0,
    paramDescRangeF(
      "Top-tail clip percentage for RAW16 driver-side AGC (e.g. 1.0 discards the brightest 1% of pixels before "
      "linear stretch). Valid range [0, 50); invalid values revert to 1.0.",
      0.0, 50.0));
  camera_info_url_ = this->declare_parameter(
    "camera_info_url", "",
    paramDesc("Camera calibration file URL (file:// or package://). Empty publishes uncalibrated CameraInfo."));
  point_x_ = this->declare_parameter(
    "point_x", is640 ? 319 : 159, paramDescRangeI("X coord of the temperature probe point", 0, is640 ? 639 : 319));
  point_y_ = this->declare_parameter(
    "point_y", is640 ? 255 : 127, paramDescRangeI("Y coord of the temperature probe point", 0, is640 ? 511 : 255));
  max_temp_limit_ = this->declare_parameter(
    "max_temp_limit", 50, paramDescRangeI("Maximum temperature for constant AGC.", -273, 655 - 273 - 1));
  min_temp_limit_ = this->declare_parameter(
    "min_temp_limit", 20, paramDescRangeI("Minimum temperature for constant AGC.", -273, max_temp_limit_));
  norm_margin_ = this->declare_parameter(
    "norm_margin", 20.0,
    paramDescRangeF(
      "Constant AGC margin (in 8-bit image units). Non-zero values means the given number of lowest and highest "
      "values will not be used in the linear stretch.",
      0.0, 120.0));

  this->validateParams();

  params_cb_ = this->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& parameters) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& param : parameters) {
        if (param.get_name() == "point_x") {
          point_x_ = param.as_int();
        } else if (param.get_name() == "point_y") {
          point_y_ = param.as_int();
        } else if (param.get_name() == "max_temp_limit") {
          max_temp_limit_ = param.as_int();
        } else if (param.get_name() == "min_temp_limit") {
          min_temp_limit_ = param.as_int();
        } else if (param.get_name() == "norm_margin") {
          norm_margin_ = param.as_double();
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() {
      this->init_timer_->cancel(); // Prevent it from looping
      this->init();
    });
}

#else

BosonCamera::BosonCamera() : fd_(-1) {}

void BosonCamera::onInit() {
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  pnh_.param<std::string>("frame_id", frame_id_, "boson_camera");
  pnh_.param<std::string>("dev", dev_path_, "/dev/video0");
  pnh_.param<double>("frame_rate", frame_rate_, 30.0);
  pnh_.param<std::string>("video_mode", video_mode_str_, "YUV");
  pnh_.param<bool>("zoom_enable", zoom_enable_, false);
  pnh_.param<bool>("publish_color", publish_color_, false);
  pnh_.param<std::string>("sensor_type", sensor_type_str_, "Boson_640");
  pnh_.param<std::string>("camera_info_url", camera_info_url_, "");
  pnh_.param<int>("point_x", point_x_, 319);
  pnh_.param<int>("point_y", point_y_, 255);
  pnh_.param<int>("max_temp_limit", max_temp_limit_, 50);
  pnh_.param<int>("min_temp_limit", min_temp_limit_, 20);
  pnh_.param<double>("norm_margin", norm_margin_, 20.0);
  pnh_.param<double>("raw16_agc_low_pct", raw16_agc_low_pct_, 1.0);
  pnh_.param<double>("raw16_agc_high_pct", raw16_agc_high_pct_, 1.0);

  this->validateParams();

  reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<flir_boson_usb::BosonCameraConfig>>(pnh_);
  // The callback has to be spelled out as a boost::function to disambiguate
  // the two setCallback() overloads of the reconfigure server.
  reconfigure_server_->setCallback(
    boost::function<void (flir_boson_usb::BosonCameraConfig&, uint32_t)>(
      [this](flir_boson_usb::BosonCameraConfig& config, uint32_t level) {
        this->reconfigureCallback(config, level);
      }));

  this->init();
}

void BosonCamera::reconfigureCallback(flir_boson_usb::BosonCameraConfig& config, uint32_t /* level */) {
  std::lock_guard<std::mutex> lock(mutex_);
  point_x_ = config.point_x;
  point_y_ = config.point_y;
  max_temp_limit_ = config.max_temp_limit;
  min_temp_limit_ = config.min_temp_limit;
  norm_margin_ = config.norm_margin;
}

#endif

BosonCamera::~BosonCamera() {
  closeCamera();
}

void BosonCamera::validateParams() {
  if (!std::isfinite(frame_rate_) || frame_rate_ <= 0.0) {
    CRAS_WARN("Invalid frame_rate parameter (%.3f). Clamping to 1.0 Hz.", frame_rate_);
    frame_rate_ = 1.0;
  }

  const auto clip_valid = [](const double p) {
    return std::isfinite(p) && p >= 0.0 && p < 50.0;
  };
  if (!clip_valid(raw16_agc_low_pct_) || !clip_valid(raw16_agc_high_pct_)) {
    CRAS_WARN(
      "Invalid AGC clip percentages (%.2f / %.2f). Reverting to 1.0 / 1.0.", raw16_agc_low_pct_, raw16_agc_high_pct_);
    raw16_agc_low_pct_ = 1.0;
    raw16_agc_high_pct_ = 1.0;
  }
}

void BosonCamera::init() {
  CRAS_INFO("Initializing FLIR Boson on %s", dev_path_.c_str());
#ifdef ROS2
  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

  const auto cam_pub_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  // Set explicit Best-Effort / SensorData QoS for 60Hz camera streams
  image_pub_ = image_transport::create_camera_publisher(this, "image_raw", cam_pub_qos);
  image_pub_8_ = image_transport::create_publisher(this, "image8", cam_pub_qos);
  image_pub_8_norm_ = image_transport::create_publisher(this, "image8_norm", cam_pub_qos);
  if (publish_color_) {
    image_pub_heatmap_ = image_transport::create_publisher(this, "image_heatmap", cam_pub_qos);
    image_pub_temp_ = image_transport::create_publisher(this, "image_temp", cam_pub_qos);
  }

  max_temp_pub_ = this->create_publisher<Temperature>("max_temp", 1);
  min_temp_pub_ = this->create_publisher<Temperature>("min_temp", 1);
  ptr_temp_pub_ = this->create_publisher<Temperature>("ptr_temp", 1);
#else
  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_);
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);
  image_pub_ = it_->advertiseCamera("image_raw", 1);
  image_pub_8_ = it_->advertise("image8", 1);
  image_pub_8_norm_ = it_->advertise("image8_norm", 1);
  if (publish_color_) {
    image_pub_heatmap_ = it_->advertise("image_heatmap", 1);
    image_pub_temp_ = it_->advertise("image_temp", 1);
  }

  max_temp_pub_ = nh_.advertise<Temperature>("max_temp", 1);
  min_temp_pub_ = nh_.advertise<Temperature>("min_temp", 1);
  ptr_temp_pub_ = nh_.advertise<Temperature>("ptr_temp", 1);
#endif

  if (video_mode_str_ == "RAW16") {
    video_mode_ = Encoding::RAW16;
  } else if (video_mode_str_ == "YUV") {
    video_mode_ = Encoding::YUV;
  } else {
    CRAS_ERROR("Invalid video_mode. Use YUV or RAW16.");
#ifndef ROS2
    ros::shutdown();
#else
    rclcpp::shutdown();
#endif
    return;
  }

  std::string cam_name;
  if (sensor_type_str_ == "Boson_320" || sensor_type_str_ == "boson_320") {
    sensor_type_ = SensorTypes::Boson320;
    cam_name = "Boson320";
  } else if (sensor_type_str_ == "Boson_640" || sensor_type_str_ == "boson_640") {
    sensor_type_ = SensorTypes::Boson640;
    cam_name = "Boson640";
  } else {
    CRAS_ERROR("Invalid sensor_type value provided.");
#ifdef ROS2
    rclcpp::shutdown();
#else
    ros::shutdown();
#endif
    return;
  }

  if (camera_info_url_.empty()) {
    CRAS_WARN(
      "No camera_info_url set; publishing uncalibrated CameraInfo. Set camera_info_url to a "
      "file:// or package:// URL to load a calibration.");
  } else {
    // A URL was provided, so set the name and load it
    camera_info_->setCameraName(cam_name);

    if (camera_info_->validateURL(camera_info_url_)) {
      camera_info_->loadCameraInfo(camera_info_url_);
      CRAS_INFO("Loaded camera calibration from %s", camera_info_url_.c_str());
    } else {
      CRAS_WARN(
        "camera_info_url '%s' could not be validated; publishing uncalibrated CameraInfo.", camera_info_url_.c_str());
    }
  }

  if (video_mode_ != Encoding::RAW16 && (raw16_agc_low_pct_ != 1.0 || raw16_agc_high_pct_ != 1.0)) {
    CRAS_WARN(
      "AGC clip percentages (raw16_agc_low_pct / raw16_agc_high_pct) are only supported in RAW16 mode and will be "
      "ignored.");
  }

  if (zoom_enable_ && sensor_type_ == SensorTypes::Boson640) {
    CRAS_WARN("zoom_enable is only for Boson320.");
  }

  if (zoom_enable_ && sensor_type_ == SensorTypes::Boson320 && video_mode_ != Encoding::RAW16) {
    CRAS_WARN(
      "zoom_enable is only honored in RAW16 mode (got %s). Image will be published at native sensor resolution.",
      video_mode_str_.c_str());
  }

  if (!openCamera()) {
#ifdef ROS2
    rclcpp::shutdown();
#else
    ros::shutdown();
#endif
    return;
  }

  const double period = 1.0 / frame_rate_;
#ifdef ROS2
  capture_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
    std::bind(&BosonCamera::captureAndPublish, this));
#else
  capture_timer_ = nh_.createTimer(
    ros::Duration(period),
    [this](const ros::TimerEvent& /* event */) {
      this->captureAndPublish();
    });
#endif
}

void BosonCamera::agc(
    const cv::Mat& input_16, cv::Mat& output_8, cv::Mat& output_16,
    const double clip_low_pct, const double clip_high_pct,
    double* max_temp, double* min_temp) {
  CV_Assert(input_16.type() == CV_16UC1);

  output_16 = input_16.clone();

  int histSize = 65536;
  float range[] = { 0, 65536 };
  const float* histRange = { range };

  // Reuses the pre-allocated hist_ member variable
  cv::calcHist(&input_16, 1, 0, cv::Mat(), hist_, 1, &histSize, &histRange, true, false);

  double total_pixels = input_16.rows * input_16.cols;
  double clip_low_count = (clip_low_pct / 100.0) * total_pixels;
  double clip_high_count = (clip_high_pct / 100.0) * total_pixels;

  int min_val = 0, max_val = 65535;
  double current_count = 0.0; // double prevents truncation on large sensors

  // Find bottom percentile
  for (int i = 0; i < histSize; i++) {
    current_count += hist_.at<float>(i);
    if (current_count > clip_low_count) {
      min_val = i;
      break;
    }
  }

  // Find top percentile
  current_count = 0.0;
  for (int i = histSize - 1; i >= 0; i--) {
    current_count += hist_.at<float>(i);
    if (current_count > clip_high_count) {
      max_val = i;
      break;
    }
  }

  if (max_val <= min_val) {
    max_val = min_val + 1; // Prevent division by zero
  }

  *max_temp = max_val / 100. - 273.15;
  *min_temp = min_val / 100. - 273.15;

  if (max_temp_limit_ < min_temp_limit_) {
    std::stringstream err_msg_ss;
    err_msg_ss << "max_temp_limit should be larger than min_temp_limit ";
    err_msg_ss << "(max_temp_limit: " << max_temp_limit_ << ", min_temp_limit: " << min_temp_limit_ << ")";
    throw std::range_error(err_msg_ss.str());
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    max_val = (max_temp_limit_ + 273.15) * 100;
    min_val = (min_temp_limit_ + 273.15) * 100;
  }

  // Scale using SIMD-optimized convertTo
  double scale = 255.0 / (max_val - min_val);
  double shift = -min_val * scale;
  input_16.convertTo(output_8, CV_8UC1, scale, shift);
}

bool BosonCamera::openCamera() {
  if ((fd_ = open(dev_path_.c_str(), O_RDWR)) < 0) {
    CRAS_ERROR("ERROR: Invalid Video Device.");
    return false;
  }

  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap_) < 0) {
    CRAS_ERROR("ERROR: Video Capture is not available.");
    return false;
  }

  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // 1. Determine baseline sizes from parameter state
  int requested_width = (sensor_type_ == SensorTypes::Boson640) ? 640 : 320;
  int requested_height = (sensor_type_ == SensorTypes::Boson640) ? 512 : 256;

  if (zoom_enable_ && sensor_type_ == SensorTypes::Boson320 && video_mode_ == Encoding::RAW16) {
    thermal16_linear_zoom_ = Mat(512, 640, CV_8UC1);
  }

  // 2. Set format parameters
  format.fmt.pix.pixelformat = video_mode_ == Encoding::RAW16 ? V4L2_PIX_FMT_Y16 : V4L2_PIX_FMT_YVU420;
  format.fmt.pix.width = requested_width;
  format.fmt.pix.height = requested_height;

  // 3. Negotiate with the hardware
  if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
    CRAS_ERROR("VIDIOC_S_FMT error. Format not supported.");
    return false;
  }

  // 4. Validate that the hardware width matches what we requested
  // (Prevents someone from specifying Boson_640 parameter on a physical 320 camera)
  if (static_cast<int>(format.fmt.pix.width) != requested_width) {
    CRAS_ERROR("Hardware mismatch! Configured for %s (width %d) but V4L2 negotiated width %d.",
      sensor_type_str_.c_str(), requested_width, format.fmt.pix.width);
    return false;
  }

  // 5. Hard assignment: Store the TRUE negotiated hardware dimensions
  // (This absorbs the telemetry offset seamlessly if it is turned on)
  width_ = format.fmt.pix.width;
  height_ = format.fmt.pix.height;

  // YUV unpack path assumes tightly packed planes. Assert that here.
  if (video_mode_ == Encoding::YUV && format.fmt.pix.bytesperline != 0 &&
      format.fmt.pix.bytesperline != format.fmt.pix.width) {
    CRAS_ERROR(
      "Driver reports YUV bytesperline=%u but width=%u. Strided YUV buffers are not supported by this node.",
      format.fmt.pix.bytesperline, format.fmt.pix.width);
    return false;
  }

  // 6. Check that the driver accepted a format we actually know how to unpack
  if (video_mode_ == Encoding::RAW16 && format.fmt.pix.pixelformat != V4L2_PIX_FMT_Y16) {
    CRAS_ERROR("Driver did not negotiate Y16 in RAW16 mode.");
    return false;
  }

  bool is_i420 = (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420);
  bool is_yv12 = (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420);
  is_yv12_ = is_yv12;

  if (video_mode_ == Encoding::YUV && !is_i420 && !is_yv12) {
    CRAS_ERROR("Driver did not negotiate a supported 8-bit 4:2:0 format.");
    return false;
  }

  // --- Hardware Framerate Validation ---
  struct v4l2_streamparm streamparm;
  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (ioctl(fd_, VIDIOC_G_PARM, &streamparm) == 0) {
    if (streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
      double hw_fps =
        (double)streamparm.parm.capture.timeperframe.denominator /
          (double)streamparm.parm.capture.timeperframe.numerator;

      if (frame_rate_ > hw_fps + 1.0) {  // +1.0 for floating point margin
        CRAS_WARN(
          "Requested ROS frame_rate (%.1f Hz) exceeds actual hardware rate (%.1f Hz). "
          "The node will automatically throttle to the hardware limit.", frame_rate_, hw_fps);
      }
    }
  }

  // Calculate and lock the exact dimensions
  expected_height_ = (sensor_type_ == SensorTypes::Boson640) ? 512 : 256;
  bytesperline_ = format.fmt.pix.bytesperline;

  // Request 4 buffers to prevent pipeline stalls
  struct v4l2_requestbuffers bufrequest;
  memset(&bufrequest, 0, sizeof(bufrequest));
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 4;

  if (ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0) {
    return false;
  }

  buffers_.resize(bufrequest.count);

  for (size_t i = 0; i < buffers_.size(); ++i) {
    struct v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = i;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo) < 0) {
      return false;
    }

    buffers_[i].length = bufferinfo.length;
    buffers_[i].start = mmap(nullptr, bufferinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, bufferinfo.m.offset);

    if (buffers_[i].start == MAP_FAILED) {
      return false;
    }
    memset(buffers_[i].start, 0, bufferinfo.length);

    if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0) {
      return false;
    }
  }

  // Pre-allocate output Mats to expected_height_
  thermal16_linear_ = cv::Mat(expected_height_, width_, CV_8UC1);
  thermal8_linear_ = cv::Mat(expected_height_, width_, CV_16UC1);
  thermal8_norm_ = Mat(expected_height_, width_, CV_8U, 1);
  if (video_mode_ == Encoding::YUV && publish_color_) {
    thermal_rgb_ = cv::Mat(height_, width_, CV_8UC3);
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    return false;
  }

  return true;
}

bool BosonCamera::closeCamera() {
  if (fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
      CRAS_WARN("Failed to stop V4L2 stream.");
    }

    // Loop through and unmap all 4 buffers
    for (auto& buffer : buffers_) {
      if (buffer.start != nullptr && buffer.start != MAP_FAILED) {
        munmap(buffer.start, buffer.length);
      }
    }
    buffers_.clear();

    close(fd_);
    fd_ = -1;
  }
  return true;
}

void BosonCamera::captureAndPublish() {
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  if (poll(&pfd, 1, 0) <= 0) {
    return;
  }

  struct v4l2_buffer bufferinfo;
  memset(&bufferinfo, 0, sizeof(bufferinfo));
  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd_, VIDIOC_DQBUF, &bufferinfo) < 0) {
    CRAS_ERROR("VIDIOC_DQBUF error.");
    return;
  }

  void* current_buffer = buffers_[bufferinfo.index].start;

  Header header;
#ifdef ROS2
  header.stamp = this->now();
#else
  header.stamp = ros::Time::now();
#endif
  header.frame_id = frame_id_;

  cv_bridge::CvImage cv_img;
  cv_img.header = header;

  double max_temp, min_temp;
  // ---------- Phase A: copy data out of the V4L2 buffer ----------
  if (video_mode_ == Encoding::RAW16) {
    cv::Mat thermal16(height_, width_, CV_16UC1, current_buffer, bytesperline_);
    cv::Mat thermal16_cropped = thermal16(cv::Rect(0, 0, width_, expected_height_));
    try {
      agc(
        thermal16_cropped, thermal8_linear_, thermal16_linear_, raw16_agc_low_pct_, raw16_agc_high_pct_,
        &max_temp, &min_temp);
    } catch (const std::range_error& e) {
#ifndef ROS2
      CRAS_ERROR_THROTTLE(1.0, "AGC error: %s", e.what());
#else
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "AGC error: %s", e.what());
#endif
      return;
    }
  } else { // YUV
    cv::Mat thermal_luma(height_ + height_ / 2, width_, CV_8UC1, current_buffer);
    if (publish_color_) {
      if (is_yv12_) {
        cv::cvtColor(thermal_luma, thermal_rgb_, cv::COLOR_YUV2BGR_YV12);
      } else {
        cv::cvtColor(thermal_luma, thermal_rgb_, cv::COLOR_YUV2BGR_I420);
      }
    } else {
      cv_img.image = thermal_luma(cv::Rect(0, 0, width_, expected_height_)).clone();
    }
  }

  // ---------- Phase B: hand the buffer back NOW ----------
  if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0) {
    CRAS_ERROR("VIDIOC_QBUF error during recycle.");
  }

  // ---------- Phase C: set encodings, zoom, publish ----------
  if (video_mode_ == Encoding::RAW16) {
    if (zoom_enable_ && sensor_type_ == SensorTypes::Boson320) {
      cv::resize(thermal16_linear_, thermal16_linear_zoom_, cv::Size(640, 512));
      cv_img.image = thermal16_linear_zoom_;
    } else {
      cv_img.image = thermal16_linear_;
    }
    cv_img.encoding = "mono16";
  } else { // YUV
    if (publish_color_) {
      cv_img.image = thermal_rgb_(cv::Rect(0, 0, width_, expected_height_));
      cv_img.encoding = "bgr8";
    } else {
      cv_img.encoding = "mono8"; // image already populated in Phase A
    }
  }

#ifndef ROS2
  sensor_msgs::CameraInfoPtr ci(new CameraInfo());
#else
  auto ci = std::make_shared<CameraInfo>();
#endif

  if (!camera_info_url_.empty()) {
    // If a URL was provided, get the calibrated info from the manager
    *ci = camera_info_->getCameraInfo();
  } else {
    // Otherwise, bypass the manager and populate a basic uncalibrated message
    ci->width = width_;
    ci->height = expected_height_;
    ci->distortion_model = "plumb_bob";
    // (Other arrays like D, K, R, P will remain empty/0, which is standard for uncalibrated)
  }

  ci->header = header;
#ifndef ROS2
  image_pub_.publish(cv_img.toImageMsg(), ci);
#else
  image_pub_.publish(*cv_img.toImageMsg(), *ci);
#endif

  if (video_mode_ == Encoding::RAW16) {
    // 8bit image
    if (zoom_enable_ && sensor_type_ == SensorTypes::Boson320) {
      cv::resize(thermal8_linear_, thermal8_linear_zoom_, cv::Size(640, 512));
      cv_img.image = thermal8_linear_zoom_;
    } else {
      cv_img.image = thermal8_linear_;
    }
    cv_img.encoding = "mono8";
    image_pub_8_.publish(cv_img.toImageMsg());

    // 8bit image (auto range)
    double min, max;
    minMaxLoc(thermal8_linear_, &min, &max);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double min_threshold = std::max(min - norm_margin_, 0.0);
      double max_threshold = std::min(max + norm_margin_, 255.0);
      if ((max_threshold - min_threshold) != 0) {
        thermal8_norm_ = (thermal8_linear_ - min_threshold) * (255 - 0) / (max_threshold - min_threshold);
      } else {
        thermal8_norm_ = thermal8_linear_;
      }
    }
    cv_img.image = thermal8_norm_;
    cv_img.encoding = "mono8";
    image_pub_8_norm_.publish(cv_img.toImageMsg());

    if (publish_color_) {
      cv::applyColorMap(cv_img.image, thermal8_heatmap_, cv::COLORMAP_JET);
      // 8bit heatmap image
      cv_img.image = thermal8_heatmap_;
      cv_img.encoding = "bgr8";
      image_pub_heatmap_.publish(cv_img.toImageMsg());

      // put temperature info
      thermal8_temp_ = thermal8_heatmap_.clone();
      std::stringstream max_temp_ss, min_temp_ss, ptr_temp_ss;
      max_temp_ss << std::fixed << std::setprecision(2) << max_temp;
      min_temp_ss << std::fixed << std::setprecision(2) << min_temp;

      std::string disp_max_temp = "Max: " + max_temp_ss.str() + " deg";
      std::string disp_min_temp = "Min: " + min_temp_ss.str() + " deg";
      cv::putText(
        thermal8_temp_, disp_max_temp, cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
      cv::putText(
        thermal8_temp_, disp_min_temp, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
      // pointer temperature
      {
        std::lock_guard<std::mutex> lock(mutex_);
        temp_ptr_ = cv::Point(point_x_, point_y_);
        ptr_temp_ = thermal16_linear_.at<uint16_t>(point_y_, point_x_) / 100.0 - 273.15;
      }
      ptr_temp_ss << std::fixed << std::setprecision(2) << ptr_temp_;
      std::string disp_ptr_temp = "Ptr: " + ptr_temp_ss.str() + " deg";
      cv::putText(
        thermal8_temp_, disp_ptr_temp, cv::Point(15, 45), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
      cv::circle(thermal8_temp_, temp_ptr_, 3, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      cv::circle(thermal8_temp_, temp_ptr_, 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);

      max_temp_msg_.header = header;
      min_temp_msg_.header = header;
      ptr_temp_msg_.header = header;
      max_temp_msg_.temperature = max_temp;
      min_temp_msg_.temperature = min_temp;
      ptr_temp_msg_.temperature = ptr_temp_;

#ifndef ROS2
      max_temp_pub_.publish(max_temp_msg_);
      min_temp_pub_.publish(min_temp_msg_);
      ptr_temp_pub_.publish(ptr_temp_msg_);
#else
      max_temp_pub_->publish(max_temp_msg_);
      min_temp_pub_->publish(min_temp_msg_);
      ptr_temp_pub_->publish(ptr_temp_msg_);
#endif

      // 24bit image
      cv_img.image = thermal8_temp_;
      cv_img.encoding = "bgr8";
      image_pub_temp_.publish(cv_img.toImageMsg());
    }
  }
}

}  // namespace flir_boson_usb

#ifndef ROS2
PLUGINLIB_EXPORT_CLASS(flir_boson_usb::BosonCamera, nodelet::Nodelet)
#else
RCLCPP_COMPONENTS_REGISTER_NODE(flir_boson_usb::BosonCamera)
#endif
