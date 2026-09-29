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

#include "flir_boson_usb/BosonCamera.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace flir_boson_usb
{

using namespace cv;

BosonCamera::BosonCamera(const rclcpp::NodeOptions & options)
: Node("boson_camera", options), fd_(-1)
{
  frame_id_ = this->declare_parameter("frame_id", "boson_camera");
  dev_path_ = this->declare_parameter("dev", "/dev/video0");
  frame_rate_ = this->declare_parameter("frame_rate", 30.0);
  if (!std::isfinite(frame_rate_) || frame_rate_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
      "Invalid frame_rate parameter (%.3f). Clamping to 1.0 Hz.", frame_rate_);
    frame_rate_ = 1.0;
  }
  video_mode_str_ = this->declare_parameter("video_mode", "YUV");
  zoom_enable_ = this->declare_parameter("zoom_enable", false);
  publish_color_ = this->declare_parameter("publish_color", false);
  sensor_type_str_ = this->declare_parameter("sensor_type", "Boson_640");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");

  raw16_agc_low_pct_  = this->declare_parameter("raw16_agc_low_pct", 1.0);
  raw16_agc_high_pct_ = this->declare_parameter("raw16_agc_high_pct", 1.0);

  auto clip_valid = [](double p) { return std::isfinite(p) && p >= 0.0 && p < 50.0; };
  if (!clip_valid(raw16_agc_low_pct_) || !clip_valid(raw16_agc_high_pct_)) {
    RCLCPP_WARN(this->get_logger(), "Invalid AGC clip percentages (%.2f / %.2f). Reverting to 1.0 / 1.0.", 
                raw16_agc_low_pct_, raw16_agc_high_pct_);
    raw16_agc_low_pct_ = 1.0;
    raw16_agc_high_pct_ = 1.0;
  }

  RCLCPP_INFO(this->get_logger(), "Initializing FLIR Boson on %s", dev_path_.c_str());

  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() {
      this->init_timer_->cancel(); // Prevent it from looping
      this->init();
    });
}

void BosonCamera::init()
{
  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  
  // Set explicit Best-Effort / SensorData QoS for 60Hz camera streams
  image_pub_ = image_transport::create_camera_publisher(
    this, "image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  if (video_mode_str_ == "RAW16") video_mode_ = RAW16_PURE;
  else if (video_mode_str_ == "RAW16_AGC") video_mode_ = RAW16_AGC;
  else if (video_mode_str_ == "YUV") video_mode_ = YUV;
  else {
    RCLCPP_ERROR(this->get_logger(), "Invalid video_mode. Use YUV, RAW16, or RAW16_AGC.");
    rclcpp::shutdown();
    return;
  }

  std::string cam_name;
  if (sensor_type_str_ == "Boson_320" || sensor_type_str_ == "boson_320") {
    sensor_type_ = Boson320;
    cam_name = "Boson320";
  } else if (sensor_type_str_ == "Boson_640" || sensor_type_str_ == "boson_640") {
    sensor_type_ = Boson640;
    cam_name = "Boson640";
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid sensor_type value provided.");
    rclcpp::shutdown();
    return;
  }

  if (camera_info_url_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "No camera_info_url set; publishing uncalibrated CameraInfo. "
      "Set camera_info_url to a file:// or package:// URL to load a calibration.");
  } else {
    // A URL was provided, so set the name and load it
    camera_info_->setCameraName(cam_name);
    
    if (camera_info_->validateURL(camera_info_url_)) {
      camera_info_->loadCameraInfo(camera_info_url_);
      RCLCPP_INFO(this->get_logger(),
        "Loaded camera calibration from %s", camera_info_url_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "camera_info_url '%s' could not be validated; "
        "publishing uncalibrated CameraInfo.", camera_info_url_.c_str());
    }
  }

  if (publish_color_ && isRaw16()) {
    RCLCPP_WARN(this->get_logger(), 
        "publish_color is only supported in YUV mode and will be ignored.");
  }

  if (video_mode_ != RAW16_AGC && (raw16_agc_low_pct_ != 1.0 || raw16_agc_high_pct_ != 1.0)) {
    RCLCPP_WARN(this->get_logger(),
        "AGC clip percentages (raw16_agc_low_pct / raw16_agc_high_pct) are only "
        "supported in RAW16_AGC mode and will be ignored.");
  }
  
  if (zoom_enable_ && sensor_type_ == Boson640) {
    RCLCPP_WARN(this->get_logger(), "zoom_enable is only for Boson320.");
  }

  if (zoom_enable_ && sensor_type_ == Boson320 && video_mode_ != RAW16_AGC) {
    RCLCPP_WARN(this->get_logger(),
      "zoom_enable is only honored in RAW16_AGC mode (got %s). Image will be published at native sensor resolution.",
      video_mode_str_.c_str());
  }

  if (!openCamera()) {
    rclcpp::shutdown();
    return;
  }

  auto period = std::chrono::duration<double>(1.0 / frame_rate_);
  capture_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&BosonCamera::captureAndPublish, this));
}

BosonCamera::~BosonCamera()
{
  closeCamera();
}

void BosonCamera::agc(const cv::Mat& input_16, cv::Mat& output_8, double clip_low_pct, double clip_high_pct)
{
    CV_Assert(input_16.type() == CV_16UC1);

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

    if (max_val <= min_val) max_val = min_val + 1; // Prevent division by zero

    // Scale using SIMD-optimized convertTo
    double scale = 255.0 / (max_val - min_val);
    double shift = -min_val * scale;
    input_16.convertTo(output_8, CV_8UC1, scale, shift);
}

bool BosonCamera::openCamera()
{
  if ((fd_ = open(dev_path_.c_str(), O_RDWR)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: Invalid Video Device.");
    return false;
  }

  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: Video Capture is not available.");
    return false;
  }

  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // 1. Determine baseline sizes from parameter state
  int requested_width = (sensor_type_ == Boson640) ? 640 : 320;
  int requested_height = (sensor_type_ == Boson640) ? 512 : 256;

  if (zoom_enable_ && sensor_type_ == Boson320 && video_mode_ == RAW16_AGC) {
    thermal16_linear_zoom_ = Mat(512, 640, CV_8UC1);
  }

  // 2. Set format parameters
  if (isRaw16()) {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
  } else {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420;
  }
  format.fmt.pix.width = requested_width;
  format.fmt.pix.height = requested_height;

  // 3. Negotiate with the hardware
  if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_S_FMT error. Format not supported.");
    return false;
  }
  
  // 4. Validate that the hardware width matches what we requested
  // (Prevents someone from specifying Boson_640 parameter on a physical 320 camera)
  if (static_cast<int>(format.fmt.pix.width) != requested_width) {
    RCLCPP_ERROR(this->get_logger(), 
      "Hardware mismatch! Configured for %s (width %d) but V4L2 negotiated width %d.",
      sensor_type_str_.c_str(), requested_width, format.fmt.pix.width);
    return false;
  }

  // 5. Hard assignment: Store the TRUE negotiated hardware dimensions 
  // (This absorbs the telemetry offset seamlessly if it is turned on)
  width_ = format.fmt.pix.width;
  height_ = format.fmt.pix.height;

  // YUV unpack path assumes tightly packed planes. Assert that here.
  if (video_mode_ == YUV &&
      format.fmt.pix.bytesperline != 0 &&
      format.fmt.pix.bytesperline != format.fmt.pix.width)
  {
    RCLCPP_ERROR(this->get_logger(),
      "Driver reports YUV bytesperline=%u but width=%u. Strided YUV buffers "
      "are not supported by this node.",
      format.fmt.pix.bytesperline, format.fmt.pix.width);
    return false;
  }

  // 6. Check that the driver accepted a format we actually know how to unpack
  if (isRaw16() && format.fmt.pix.pixelformat != V4L2_PIX_FMT_Y16) {
    RCLCPP_ERROR(this->get_logger(), "Driver did not negotiate Y16 in RAW16 mode.");
    return false;
  }

  bool is_i420 = (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420);
  bool is_yv12 = (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420);
  is_yv12_ = is_yv12;

  if (video_mode_ == YUV && !is_i420 && !is_yv12) {
    RCLCPP_ERROR(this->get_logger(), "Driver did not negotiate a supported 8-bit 4:2:0 format.");
    return false;
  }

  // --- Hardware Framerate Validation ---
  struct v4l2_streamparm streamparm;
  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if (ioctl(fd_, VIDIOC_G_PARM, &streamparm) == 0) {
    if (streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
      double hw_fps = (double)streamparm.parm.capture.timeperframe.denominator /
                      (double)streamparm.parm.capture.timeperframe.numerator;
                      
      if (frame_rate_ > hw_fps + 1.0) { // +1.0 for floating point margin
        RCLCPP_WARN(this->get_logger(),
          "Requested ROS frame_rate (%.1f Hz) exceeds actual hardware rate (%.1f Hz). "
          "The node will automatically throttle to the hardware limit.",
          frame_rate_, hw_fps);
      }
    }
  }

  // Calculate and lock the exact dimensions
  expected_height_ = (sensor_type_ == Boson640) ? 512 : 256;
  bytesperline_ = format.fmt.pix.bytesperline;

  // Request 4 buffers to prevent pipeline stalls
  struct v4l2_requestbuffers bufrequest;
  memset(&bufrequest, 0, sizeof(bufrequest));
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 4; 

  if (ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0) return false;

  buffers_.resize(bufrequest.count);

  for (size_t i = 0; i < buffers_.size(); ++i) {
    struct v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = i;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo) < 0) return false;

    buffers_[i].length = bufferinfo.length;
    buffers_[i].start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, bufferinfo.m.offset);
    
    if (buffers_[i].start == MAP_FAILED) return false;
    memset(buffers_[i].start, 0, bufferinfo.length);

    if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0) return false;
  }

  // Pre-allocate output Mats to expected_height_
  thermal16_linear_ = cv::Mat(expected_height_, width_, CV_8UC1);
  if (video_mode_ == YUV && publish_color_) {
    thermal_rgb_ = cv::Mat(height_, width_, CV_8UC3);
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) return false;

  return true;
}

bool BosonCamera::closeCamera()
{
  if (fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to stop V4L2 stream.");
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

void BosonCamera::captureAndPublish()
{
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  if (poll(&pfd, 1, 0) <= 0) return;

  struct v4l2_buffer bufferinfo;
  memset(&bufferinfo, 0, sizeof(bufferinfo));
  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd_, VIDIOC_DQBUF, &bufferinfo) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_DQBUF error.");
    return;
  }

  void* current_buffer = buffers_[bufferinfo.index].start;

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = frame_id_;

  cv_bridge::CvImage cv_img;
  cv_img.header = header;

  // ---------- Phase A: copy data out of the V4L2 buffer ----------
  if (video_mode_ == RAW16_PURE) {
    cv::Mat thermal16(height_, width_, CV_16UC1, current_buffer, bytesperline_);
    cv_img.image = thermal16(cv::Rect(0, 0, width_, expected_height_)).clone();
  }
  else if (video_mode_ == RAW16_AGC) {
    cv::Mat thermal16(height_, width_, CV_16UC1, current_buffer, bytesperline_);
    cv::Mat thermal16_cropped = thermal16(cv::Rect(0, 0, width_, expected_height_));
    agc(thermal16_cropped, thermal16_linear_, raw16_agc_low_pct_, raw16_agc_high_pct_);
  }
  else { // YUV
    cv::Mat thermal_luma(height_ + height_ / 2, width_, CV_8UC1, current_buffer);
    if (publish_color_) {
      if (is_yv12_) cv::cvtColor(thermal_luma, thermal_rgb_, cv::COLOR_YUV2BGR_YV12);
      else          cv::cvtColor(thermal_luma, thermal_rgb_, cv::COLOR_YUV2BGR_I420);
    } else {
      cv_img.image = thermal_luma(cv::Rect(0, 0, width_, expected_height_)).clone();
    }
  }

  // ---------- Phase B: hand the buffer back NOW ----------
  if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF error during recycle.");
  }

  // ---------- Phase C: set encodings, zoom, publish ----------
  if (video_mode_ == RAW16_PURE) {
    cv_img.encoding = "mono16";
  }
  else if (video_mode_ == RAW16_AGC) {
    if (zoom_enable_ && sensor_type_ == Boson320) {
      cv::resize(thermal16_linear_, thermal16_linear_zoom_, cv::Size(640, 512));
      cv_img.image = thermal16_linear_zoom_;
    } else {
      cv_img.image = thermal16_linear_;
    }
    cv_img.encoding = "mono8";
  }
  else { // YUV
    if (publish_color_) {
      cv_img.image = thermal_rgb_(cv::Rect(0, 0, width_, expected_height_));
      cv_img.encoding = "bgr8";
    } else {
      cv_img.encoding = "mono8"; // image already populated in Phase A
    }
  }

  auto ci = std::make_shared<sensor_msgs::msg::CameraInfo>();

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
  image_pub_.publish(*cv_img.toImageMsg(), *ci);
}

}  // namespace flir_boson_usb

RCLCPP_COMPONENTS_REGISTER_NODE(flir_boson_usb::BosonCamera)
