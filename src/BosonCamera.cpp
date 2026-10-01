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
#include <glob.h>
#include <memory>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cras_cpp_common/string_utils.hpp>
#include <flir_boson_usb/BosonCamera.hpp>
#include <flir_boson_usb/pipeline.hpp>

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

namespace {

//! \brief Turn a fixed-size V4L2 string field into a std::string.
std::string v4l2String(const __u8* field, const size_t max_len) {
  const auto* text = reinterpret_cast<const char*>(field);
  return std::string(text, strnlen(text, max_len));
}

/**
 * \brief Extract the serial number out of a /dev/v4l/by-id identifier.
 *
 * The identifier looks like usb-Manufacturer_Camera_Name_SN123456-video-index0 and the serial is the
 * last underscore-separated chunk of the name.
 * \return Empty string when the identifier does not have the expected shape.
 */
std::string parseSerial(const std::string& identifier) {
  const auto parts = cras::split(identifier, "-");
  if (parts.size() < 3) {
    return "";
  }
  if (!cras::startsWith(parts.back(), "index")) {
    return "";
  }
  if (parts[parts.size() - 2] != "video") {
    return "";
  }

  const auto name_parts = cras::split(parts[parts.size() - 3], "_");
  if (name_parts.size() < 2) {
    return "";
  }

  return name_parts.back();
}

/**
 * \brief Join the accepted values of a string parameter into a single human-readable list.
 * \param[in] names The accepted values.
 * \param[in] separator Put between the names.
 */
std::string valueList(const std::vector<std::string>& names, const std::string& separator) {
  std::string list;
  for (const auto& name : names) {
    if (!list.empty()) {
      list += separator;
    }
    list += name;
  }
  return list;
}

//! \brief The colour palettes the heatmap can be painted with.
std::string colormapList(const std::string& separator) {
  return valueList(colormapNames(), separator);
}

//! \brief The units the temperature image can be carried in.
std::string tempList(const std::string& separator) {
  return valueList(tempModeNames(), separator);
}

//! \brief The contents the heatmap can be stamped with.
std::string overlayList(const std::string& separator) {
  return valueList(overlayModeNames(), separator);
}

}  // namespace

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

BosonCamera::BosonCamera(const rclcpp::NodeOptions& options)
    : Node("boson_camera", options), width_(-1), height_(-1), fd_(-1), cap_({}), expected_height_(-1), bytesperline_(0),
      is_yv12_(false), frame_rate_(0.0), video_mode_(Encoding::YUV), zoom_enable_(false), yuv_color_(false),
      sensor_type_(SensorTypes::Boson640), radiometric_request_(TriState::Auto), radiometric_(false),
      agc_mode_(AgcMode::AutoRange), heatmap_mode_(HeatmapMode::None), temp_mode_(TempMode::None), point_x_(0),
      point_y_(0), max_temp_limit_(50), min_temp_limit_(20), agc_norm_(false), agc_low_pct_(1.0),
      agc_high_pct_(1.0), agc_norm_margin_(20.0), colormap_(Colormap::Jet), overlay_mode_(OverlayMode::MinMaxPtr) {
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
  zoom_enable_ = this->declare_parameter(
    "zoom_enable", false,
    paramDesc("Digital 2x upscale (320x256 -> 640x512) of the published image. Only available on Boson320 cameras."));
  video_mode_str_ = this->declare_parameter(
    "video_mode", "YUV",
    paramDesc(
      "Camera image format. "
      "YUV: camera-side AGC (mono8/bgr8, low CPU). "
      "RAW16: raw 16-bit thermal counts (mono16, no camera processing). The visual topics only exist in this mode.",
      "YUV|RAW16"));
  yuv_color_ = this->declare_parameter(
    "yuv_color", false,
    paramDesc("Publish image_raw as bgr8 instead of mono8. Only honoured in the YUV video mode."));
  agc_mode_str_ = this->declare_parameter(
    "agc_mode", "auto_range",
    paramDesc(
      "Content of the image_visual topic. "
      "none: no visual topic. "
      "fixed_range: stretch [min_temp_limit, max_temp_limit] onto the grey scale. "
      "auto_range: stretch the percentiles of the current frame given by agc_low_pct and agc_high_pct.",
      "none|fixed_range|auto_range"));
  agc_low_pct_ = this->declare_parameter(
    "agc_low_pct", 1.0,
    paramDescRangeF(
      "Bottom-tail clip percentage of the frame AGC (1.0 discards the darkest 1% of pixels before the linear "
      "stretch). Only honoured in agc_mode:=auto_range. Valid range [0, 50).",
      0.0, 50.0));
  agc_high_pct_ = this->declare_parameter(
    "agc_high_pct", 1.0,
    paramDescRangeF(
      "Top-tail clip percentage of the frame AGC (1.0 discards the brightest 1% of pixels before the linear "
      "stretch). Only honoured in agc_mode:=auto_range. Valid range [0, 50).",
      0.0, 50.0));
  agc_norm_ = this->declare_parameter(
    "agc_norm", false,
    paramDesc(
      "Renormalise image_visual to the observed frame bounds widened by agc_norm_margin. Applied to the raw counts, "
      "on top of whichever agc_mode produced the bounds."));
  agc_norm_margin_ = this->declare_parameter(
    "agc_norm_margin", 20.0,
    paramDescRangeF(
      "How much wider the observed bounds are taken when agc_norm is on, in grey levels of the active stretch.",
      0.0, 120.0));
  heatmap_mode_str_ = this->declare_parameter(
    "heatmap_mode", "none",
    paramDesc(
      "Content of the image_heatmap topic. none: no heatmap topic. visual: colourise image_visual. Requires an "
      "agc_mode other than none.",
      "none|visual"));
  overlay_mode_str_ = this->declare_parameter(
    "overlay_mode", "min_max_ptr",
    paramDesc(
      "What is stamped on image_heatmap. none: nothing. min_max_ptr: the minimum, maximum and probe reading text and "
      "the probe marker, printed in the unit of temp_mode (in raw counts for a non-radiometric camera). Only honoured "
      "when a heatmap is published.",
      overlayList("|")));
  colormap_str_ = this->declare_parameter(
    "colormap", "jet",
    paramDesc(
      "Colour palette image_heatmap is painted with. Only honoured when a heatmap is published.",
      colormapList("|")));
  temp_mode_str_ = this->declare_parameter(
    "temp_mode", "none",
    paramDesc(
      "Unit of the image_temp topic. none: no temperature topic. c, k, f: degrees Celsius, Kelvin and Fahrenheit "
      "as 32-bit floats. centi_c, centi_k, centi_f: the same units in hundredths as 16-bit integers. Only offered "
      "for radiometric cameras; also decides whether max_temp, min_temp and ptr_temp exist.",
      tempList("|")));
  camera_info_url_ = this->declare_parameter(
    "camera_info_url", "",
    paramDesc("Camera calibration file URL (file:// or package://). Empty publishes uncalibrated CameraInfo."));
  point_x_ = this->declare_parameter(
    "point_x", is640 ? 319 : 159, paramDescRangeI("X coord of the temperature probe point", 0, is640 ? 639 : 319));
  point_y_ = this->declare_parameter(
    "point_y", is640 ? 255 : 127, paramDescRangeI("Y coord of the temperature probe point", 0, is640 ? 511 : 255));
  max_temp_limit_ = this->declare_parameter(
    "max_temp_limit", 50, paramDescRangeI("Upper bound of the fixed_range stretch in degrees Celsius.", -273, 382));
  min_temp_limit_ = this->declare_parameter(
    "min_temp_limit", 20, paramDescRangeI("Lower bound of the fixed_range stretch in degrees Celsius.", -273, 382));
  radiometric_str_ = this->declare_parameter(
    "radiometric", "auto",
    paramDesc(
      "Whether the camera can map the raw counts to absolute temperatures. "
      "auto: match the device identification against radiometric_patterns. "
      "true/false: override the detection. Only radiometric cameras publish image_temp and the temperature topics.",
      "auto|true|false"));
  radiometric_patterns_ = this->declare_parameter(
    "radiometric_patterns", std::vector<std::string>({"[Rr]adiometric"}),
    paramDesc(
      "Regular expressions matched against the /dev/v4l/by-id device identifier and the V4L2 card, driver and bus "
      "strings. An empty list, or no match, means the camera is not radiometric."));

  if (!this->validateParams()) {
    rclcpp::shutdown();
    return;
  }

  params_cb_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // The values the parameters had before this call, so that a rejected change leaves them alone.
          const int point_x = point_x_;
          const int point_y = point_y_;
          const int max_temp_limit = max_temp_limit_;
          const int min_temp_limit = min_temp_limit_;
          const bool agc_norm = agc_norm_;
          const double agc_low_pct = agc_low_pct_;
          const double agc_high_pct = agc_high_pct_;
          const double agc_norm_margin = agc_norm_margin_;
          const std::string colormap = colormap_str_;
          const Colormap palette = colormap_;
          const std::string overlay_mode = overlay_mode_str_;
          const OverlayMode overlay = overlay_mode_;
          //! \brief The reason for the first value this call refused; empty when everything was accepted.
          std::string rejected;
          for (const auto& param : parameters) {
            if (param.get_name() == "point_x") {
              point_x_ = param.as_int();
            } else if (param.get_name() == "point_y") {
              point_y_ = param.as_int();
            } else if (param.get_name() == "max_temp_limit") {
              max_temp_limit_ = param.as_int();
            } else if (param.get_name() == "min_temp_limit") {
              min_temp_limit_ = param.as_int();
            } else if (param.get_name() == "agc_norm") {
              agc_norm_ = param.as_bool();
            } else if (param.get_name() == "agc_norm_margin") {
              agc_norm_margin_ = param.as_double();
            } else if (param.get_name() == "agc_low_pct") {
              agc_low_pct_ = param.as_double();
            } else if (param.get_name() == "agc_high_pct") {
              agc_high_pct_ = param.as_double();
            } else if (param.get_name() == "colormap") {
              // rclcpp calls the post-set callback after the value reached the parameter storage and ignores
              // its result, so the rejection is reported here and the previous palette is put back below.
              const std::string name = param.as_string();
              if (!setColormap(name)) {
                CRAS_WARN("Unknown colormap '%s', keeping '%s'.", name.c_str(), colormap_str_.c_str());
                if (rejected.empty()) {
                  rejected = "colormap has to be one of " + colormapList(", ");
                }
              }
            } else if (param.get_name() == "overlay_mode") {
              // The same situation as with the palette: the rejected value is already in the parameter
              // storage, so the previous content is put back below.
              const std::string name = param.as_string();
              if (!setOverlayMode(name)) {
                CRAS_WARN("Unknown overlay_mode '%s', keeping '%s'.", name.c_str(), overlay_mode_str_.c_str());
                if (rejected.empty()) {
                  rejected = "overlay_mode has to be one of " + overlayList(", ");
                }
              }
            }
          }
          result.successful = true;
          if (!rejected.empty()) {
            result.successful = false;
            result.reason = rejected;
          } else if (min_temp_limit_ >= max_temp_limit_) {
            result.successful = false;
            result.reason = "min_temp_limit has to be lower than max_temp_limit";
          } else if (!probeInBounds()) {
            result.successful = false;
            result.reason = "the probe point has to lie inside the published image";
          }
          if (!result.successful) {
            point_x_ = point_x;
            point_y_ = point_y;
            max_temp_limit_ = max_temp_limit;
            min_temp_limit_ = min_temp_limit;
            agc_norm_ = agc_norm;
            agc_low_pct_ = agc_low_pct;
            agc_high_pct_ = agc_high_pct;
            agc_norm_margin_ = agc_norm_margin;
            colormap_str_ = colormap;
            colormap_ = palette;
            overlay_mode_str_ = overlay_mode;
            overlay_mode_ = overlay;
          }
        }
        if (result.successful) {
          applyPipelineConfig();
        }
        return result;
      });

  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
      [this] {
        this->init_timer_->cancel();  // Prevent it from looping
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
  pnh_.param<bool>("yuv_color", yuv_color_, false);
  pnh_.param<std::string>("sensor_type", sensor_type_str_, "Boson_640");
  pnh_.param<std::string>("camera_info_url", camera_info_url_, "");
  pnh_.param<std::string>("agc_mode", agc_mode_str_, "auto_range");
  pnh_.param<bool>("agc_norm", agc_norm_, false);
  pnh_.param<double>("agc_low_pct", agc_low_pct_, 1.0);
  pnh_.param<double>("agc_high_pct", agc_high_pct_, 1.0);
  pnh_.param<double>("agc_norm_margin", agc_norm_margin_, 20.0);
  pnh_.param<std::string>("heatmap_mode", heatmap_mode_str_, "none");
  pnh_.param<std::string>("overlay_mode", overlay_mode_str_, "min_max_ptr");
  pnh_.param<std::string>("colormap", colormap_str_, "jet");
  pnh_.param<std::string>("temp_mode", temp_mode_str_, "none");
  pnh_.param<int>("point_x", point_x_, 319);
  pnh_.param<int>("point_y", point_y_, 255);
  pnh_.param<int>("max_temp_limit", max_temp_limit_, 50);
  pnh_.param<int>("min_temp_limit", min_temp_limit_, 20);
  pnh_.param<std::string>("radiometric", radiometric_str_, "auto");
  pnh_.param<std::vector<std::string>>(
    "radiometric_patterns", radiometric_patterns_, std::vector<std::string>({"[Rr]adiometric"}));

  if (!this->validateParams()) {
    ros::shutdown();
    return;
  }

  reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<flir_boson_usb::BosonCameraConfig>>(pnh_);
  // The callback has to be spelled out as a boost::function to disambiguate
  // the two setCallback() overloads of the reconfigure server.
  reconfigure_server_->setCallback(
    boost::function<void(flir_boson_usb::BosonCameraConfig&, uint32_t)>(
          [this](flir_boson_usb::BosonCameraConfig& config, uint32_t level) {
            this->reconfigureCallback(config, level);
          }));

  this->init();
}

void BosonCamera::reconfigureCallback(flir_boson_usb::BosonCameraConfig& config, uint32_t /* level */) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // dynamic_reconfigure has no way of expressing a relation between two fields, so an invalid request
    // is corrected here instead of being rejected.
    if (config.min_temp_limit >= config.max_temp_limit) {
      CRAS_WARN(
        "min_temp_limit (%d) has to be lower than max_temp_limit (%d), keeping the previous limits.",
        config.min_temp_limit, config.max_temp_limit);
      config.min_temp_limit = min_temp_limit_;
      config.max_temp_limit = max_temp_limit_;
    }
    const auto size = publishedSize();
    if (config.point_x < 0 || config.point_x >= size.width || config.point_y < 0 || config.point_y >= size.height) {
      CRAS_WARN(
        "The probe point (%d, %d) is outside the published image (%dx%d), keeping the previous probe.",
        config.point_x, config.point_y, size.width, size.height);
      config.point_x = point_x_;
      config.point_y = point_y_;
    }
    point_x_ = config.point_x;
    point_y_ = config.point_y;
    max_temp_limit_ = config.max_temp_limit;
    min_temp_limit_ = config.min_temp_limit;
    agc_norm_ = config.agc_norm;
    agc_norm_margin_ = config.agc_norm_margin;
    // An unknown palette would silently keep the previous one, so put it back into the config to let the
    // reconfigure server (and therefore the GUI) show the palette that is really used.
    if (!setColormap(config.colormap)) {
      CRAS_WARN("Unknown colormap '%s', keeping '%s'.", config.colormap.c_str(), colormap_str_.c_str());
      config.colormap = colormap_str_;
    }
    // The same for the overlay: an unknown content is put back so that the GUI shows what is stamped.
    if (!setOverlayMode(config.overlay_mode)) {
      CRAS_WARN(
        "Unknown overlay_mode '%s', keeping '%s'.", config.overlay_mode.c_str(), overlay_mode_str_.c_str());
      config.overlay_mode = overlay_mode_str_;
    }
    agc_low_pct_ = config.agc_low_pct;
    agc_high_pct_ = config.agc_high_pct;
  }
  applyPipelineConfig();
}

#endif

BosonCamera::~BosonCamera() {
  closeCamera();
}

bool BosonCamera::setColormap(const std::string& name) {
  Colormap colormap;
  if (!colormapFromString(cras::toLower(name), colormap)) {
    return false;
  }
  colormap_str_ = name;
  colormap_ = colormap;
  return true;
}

bool BosonCamera::setOverlayMode(const std::string& name) {
  OverlayMode mode;
  if (!overlayModeFromString(cras::toLower(name), mode)) {
    return false;
  }
  overlay_mode_str_ = name;
  overlay_mode_ = mode;
  return true;
}

bool BosonCamera::setTempMode(const std::string& name) {
  TempMode mode;
  if (!tempModeFromString(cras::toLower(name), mode)) {
    return false;
  }
  temp_mode_str_ = name;
  temp_mode_ = mode;
  temp_encoding_ = tempEncoding(mode);
  return true;
}

bool BosonCamera::validateParams() {
  if (!std::isfinite(frame_rate_) || frame_rate_ <= 0.0) {
    CRAS_WARN("Invalid frame_rate parameter (%.3f). Clamping to 1.0 Hz.", frame_rate_);
    frame_rate_ = 1.0;
  }

  // The published image size is derived from the sensor type and the video mode, and the probe point is
  // validated against it, so both have to be resolved before anything is checked.
  if (video_mode_str_ == "RAW16") {
    video_mode_ = Encoding::RAW16;
  } else if (video_mode_str_ == "YUV") {
    video_mode_ = Encoding::YUV;
  } else {
    CRAS_ERROR("Invalid video_mode value '%s', expected YUV or RAW16.", video_mode_str_.c_str());
    return false;
  }

  if (sensor_type_str_ == "Boson_320" || sensor_type_str_ == "boson_320") {
    sensor_type_ = SensorTypes::Boson320;
  } else if (sensor_type_str_ == "Boson_640" || sensor_type_str_ == "boson_640") {
    sensor_type_ = SensorTypes::Boson640;
  } else {
    CRAS_ERROR(
      "Invalid sensor_type value '%s', expected Boson_320 or Boson_640.", sensor_type_str_.c_str());
    return false;
  }

  const auto clip_valid =
    [](const double p) {
      return std::isfinite(p) && p >= 0.0 && p < 50.0;
    };
  if (!clip_valid(agc_low_pct_) || !clip_valid(agc_high_pct_)) {
    CRAS_WARN(
      "Invalid AGC clip percentages (%.2f / %.2f). Reverting to 1.0 / 1.0.", agc_low_pct_, agc_high_pct_);
    agc_low_pct_ = 1.0;
    agc_high_pct_ = 1.0;
  }

  if (agc_mode_str_ == "none") {
    agc_mode_ = AgcMode::None;
  } else if (agc_mode_str_ == "fixed_range") {
    agc_mode_ = AgcMode::FixedRange;
  } else if (agc_mode_str_ == "auto_range") {
    agc_mode_ = AgcMode::AutoRange;
  } else {
    CRAS_ERROR("Invalid agc_mode value '%s', expected none, fixed_range or auto_range.", agc_mode_str_.c_str());
    return false;
  }

  if (heatmap_mode_str_ == "none") {
    heatmap_mode_ = HeatmapMode::None;
  } else if (heatmap_mode_str_ == "visual") {
    heatmap_mode_ = HeatmapMode::Visual;
  } else {
    CRAS_ERROR("Invalid heatmap_mode value '%s', expected none or visual.", heatmap_mode_str_.c_str());
    return false;
  }

  if (!setOverlayMode(overlay_mode_str_)) {
    CRAS_ERROR(
      "Invalid overlay_mode value '%s', expected one of %s.", overlay_mode_str_.c_str(),
      overlayList(", ").c_str());
    return false;
  }

  if (!setColormap(colormap_str_)) {
    CRAS_ERROR(
      "Invalid colormap value '%s', expected one of %s.", colormap_str_.c_str(), colormapList(", ").c_str());
    return false;
  }

  if (!setTempMode(temp_mode_str_)) {
    CRAS_ERROR(
      "Invalid temp_mode value '%s', expected one of %s.", temp_mode_str_.c_str(), tempList(", ").c_str());
    return false;
  }

  if (min_temp_limit_ >= max_temp_limit_) {
    CRAS_ERROR(
      "min_temp_limit (%d) has to be lower than max_temp_limit (%d).", min_temp_limit_, max_temp_limit_);
    return false;
  }

  if (!probeInBounds()) {
    const auto size = publishedSize();
    CRAS_ERROR(
      "The probe point (%d, %d) is outside the published image (%dx%d).", point_x_, point_y_, size.width,
      size.height);
    return false;
  }

  if (radiometric_str_ == "true") {
    radiometric_request_ = TriState::Yes;
  } else if (radiometric_str_ == "false") {
    radiometric_request_ = TriState::No;
  } else if (radiometric_str_.empty() || radiometric_str_ == "auto") {
    radiometric_request_ = TriState::Auto;
  } else {
    CRAS_ERROR("Invalid radiometric value '%s', expected auto, true or false.", radiometric_str_.c_str());
    return false;
  }

  return true;
}

cv::Size BosonCamera::publishedSize() const {
  const cv::Size sensor_size =
    sensor_type_ == SensorTypes::Boson640 ? cv::Size(640, 512) : cv::Size(320, 256);
  if (zoom_enable_ && sensor_type_ == SensorTypes::Boson320 && video_mode_ == Encoding::RAW16) {
    return cv::Size(2 * sensor_size.width, 2 * sensor_size.height);
  }
  return sensor_size;
}

bool BosonCamera::probeInBounds() const {
  const auto size = publishedSize();
  return point_x_ >= 0 && point_x_ < size.width && point_y_ >= 0 && point_y_ < size.height;
}

PipelineConfig BosonCamera::pipelineConfig() const {
  std::lock_guard<std::mutex> lock(mutex_);
  PipelineConfig config;
  config.agc_mode = agc_mode_;
  config.agc_norm = agc_norm_;
  config.heatmap_mode = heatmap_mode_;
  config.overlay_mode = overlay_mode_;
  config.colormap = colormap_;
  config.temp_mode = temp_mode_;
  config.radiometric = radiometric_;
  config.agc_low_pct = agc_low_pct_;
  config.agc_high_pct = agc_high_pct_;
  config.min_limit_degC = min_temp_limit_;
  config.max_limit_degC = max_temp_limit_;
  config.agc_norm_margin = agc_norm_margin_;
  config.probe_x = point_x_;
  config.probe_y = point_y_;
  config.size = publishedSize();
  return config;
}

void BosonCamera::applyPipelineConfig() {
  // pipelineConfig() takes the lock itself, so the snapshot has to be taken before it is locked again.
  const PipelineConfig config = pipelineConfig();
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_.configure(config);
}

bool BosonCamera::detectDevice() {
  device_id_.clear();
  device_serial_.clear();

  if (dev_path_.empty()) {
    return false;
  }

  auto* resolved_device_path = realpath(dev_path_.c_str(), nullptr);
  if (resolved_device_path == nullptr) {
    CRAS_WARN("Could not resolve the device path %s.", dev_path_.c_str());
    return false;
  }

  const std::string resolved_device(resolved_device_path);
  free(resolved_device_path);

  glob_t globbuf;
  if (glob("/dev/v4l/by-id/*", 0, nullptr, &globbuf) != 0) {
    CRAS_WARN("No V4L2 device identification is available under /dev/v4l/by-id.");
    return false;
  }

  char resolved_path[PATH_MAX];
  for (size_t i = 0; i < globbuf.gl_pathc; ++i) {
    // Resolve the symlink (e.g., /dev/v4l/by-id/usb-... -> ../../video0)
    if (realpath(globbuf.gl_pathv[i], resolved_path) != nullptr && resolved_device == std::string(resolved_path)) {
      // Extract the identifier string from the symlink path
      const std::string full_path(globbuf.gl_pathv[i]);
      const auto last_slash = full_path.find_last_of('/');
      if (last_slash != std::string::npos) {
        device_id_ = full_path.substr(last_slash + 1);
      }
      break;
    }
  }
  globfree(&globbuf);

  if (device_id_.empty()) {
    CRAS_WARN("Could not match %s to any device under /dev/v4l/by-id.", resolved_device.c_str());
    return false;
  }

  // device_id_ should now contain a string like "usb-Manufacturer_Camera_Name_SN123456-video-index0"
  CRAS_INFO("Identified camera device as: %s", device_id_.c_str());

  device_serial_ = parseSerial(device_id_);
  if (device_serial_.empty()) {
    CRAS_WARN("Could not parse a serial number out of the device identifier %s.", device_id_.c_str());
  } else {
    CRAS_INFO("Identified camera serial as: %s", device_serial_.c_str());
  }

  return true;
}

void BosonCamera::resolveRadiometric() {
  if (radiometric_request_ != TriState::Auto) {
    radiometric_ = radiometric_request_ == TriState::Yes;
    CRAS_INFO(
      "Camera radiometric state forced to %s by the radiometric parameter.", radiometric_ ? "true" : "false");
    return;
  }

  const std::vector<std::string> candidates{
    device_id_, v4l2String(cap_.card, sizeof(cap_.card)), v4l2String(cap_.driver, sizeof(cap_.driver)),
    v4l2String(cap_.bus_info, sizeof(cap_.bus_info))};

  std::string matched;
  std::string invalid;
  radiometric_ = matchAnyPattern(candidates, radiometric_patterns_, matched, &invalid);

  if (!invalid.empty()) {
    CRAS_WARN("radiometric_patterns contains an invalid regular expression: %s", invalid.c_str());
  }

  if (radiometric_) {
    CRAS_INFO(
      "Camera identification '%s' matches radiometric pattern '%s'; publishing absolute temperature outputs.",
      candidates[0].c_str(), matched.c_str());
  } else {
    CRAS_INFO(
      "None of radiometric_patterns matched (device '%s', card '%s', driver '%s', bus '%s'); treating the camera as "
      "non-radiometric.",
      candidates[0].c_str(), candidates[1].c_str(), candidates[2].c_str(), candidates[3].c_str());
  }
}

void BosonCamera::init() {
  CRAS_INFO("Initializing FLIR Boson on %s", dev_path_.c_str());
#ifdef ROS2
  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
#else
  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_);
#endif

  // The video mode and the sensor type were resolved by validateParams(), which also validated the probe
  // point against the published size that follows from them.
  std::string cam_name = sensor_type_ == SensorTypes::Boson640 ? "Boson640" : "Boson320";

  detectDevice();
  if (!device_serial_.empty()) {
    CRAS_INFO("Camera serial was found. Setting camera name to: %s", device_serial_.c_str());
    cam_name = device_serial_;
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

  // The radiometric state comes out of the V4L2 capability strings, so it can only be resolved once the
  // camera is open -- and it has to be resolved before any publisher is created.
  resolveRadiometric();
  // The stage needs both the frame size, which openCamera() determined, and the radiometric state.
  applyPipelineConfig();
  createPublishers();

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

void BosonCamera::createPublishers() {
  // A topic either carries data or it does not exist, so a publisher is only created for the outputs the
  // preset enables. The processing stage uses exactly the same conditions.
  const bool visual = video_mode_ == Encoding::RAW16 && agc_mode_ != AgcMode::None;
  const bool heatmap = visual && heatmap_mode_ != HeatmapMode::None;
  const bool temp = radiometric_ && temp_mode_ != TempMode::None && video_mode_ == Encoding::RAW16;

  if (video_mode_ != Encoding::RAW16) {
    if (agc_mode_ != AgcMode::None || heatmap_mode_ != HeatmapMode::None || temp_mode_ != TempMode::None) {
      CRAS_INFO(
        "The visual, heatmap and temperature outputs are only produced in RAW16 mode; publishing image_raw only.");
    }
  } else if (agc_mode_ == AgcMode::None) {
    CRAS_INFO("agc_mode is none; publishing image_raw only.");
  }

  if (temp_mode_ != TempMode::None && !radiometric_) {
    CRAS_WARN(
      "temp_mode is %s but the camera was resolved as non-radiometric; the raw counts cannot be turned into "
      "absolute temperatures, so no temperature output is published.", temp_mode_str_.c_str());
  }

#ifdef ROS2
  const auto cam_pub_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  // Set explicit Best-Effort / SensorData QoS for 60Hz camera streams
  image_pub_ = image_transport::create_camera_publisher(this, "image_raw", cam_pub_qos);
  if (visual) {
    image_pub_visual_ = image_transport::create_publisher(this, "image_visual", cam_pub_qos);
  }
  if (heatmap) {
    image_pub_heatmap_ = image_transport::create_publisher(this, "image_heatmap", cam_pub_qos);
  }
  if (temp) {
    image_pub_temp_ = image_transport::create_publisher(this, "image_temp", cam_pub_qos);
    max_temp_pub_ = this->create_publisher<Temperature>("max_temp", 1);
    min_temp_pub_ = this->create_publisher<Temperature>("min_temp", 1);
    ptr_temp_pub_ = this->create_publisher<Temperature>("ptr_temp", 1);
  }
#else
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);
  image_pub_ = it_->advertiseCamera("image_raw", 1);
  if (visual) {
    image_pub_visual_ = it_->advertise("image_visual", 1);
  }
  if (heatmap) {
    image_pub_heatmap_ = it_->advertise("image_heatmap", 1);
  }
  if (temp) {
    image_pub_temp_ = it_->advertise("image_temp", 1);
    max_temp_pub_ = nh_.advertise<Temperature>("max_temp", 1);
    min_temp_pub_ = nh_.advertise<Temperature>("min_temp", 1);
    ptr_temp_pub_ = nh_.advertise<Temperature>("ptr_temp", 1);
  }
#endif
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
      format.fmt.pix.bytesperline != format.fmt.pix.width)
  {
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
        static_cast<double>(streamparm.parm.capture.timeperframe.denominator) /
        static_cast<double>(streamparm.parm.capture.timeperframe.numerator);

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

  // Pre-allocate the copy-out Mats. The V4L2 buffer is handed back to the driver before anything is
  // published, so no published image may be a view into it.
  if (video_mode_ == Encoding::RAW16) {
    thermal16_ = cv::Mat(expected_height_, width_, CV_16UC1);
    if (zoom_enable_ && sensor_type_ == SensorTypes::Boson320) {
      thermal16_zoom_ = cv::Mat(publishedSize(), CV_16UC1);
    }
  } else if (yuv_color_) {
    thermal_rgb_ = cv::Mat(height_, width_, CV_8UC3);
  } else {
    thermal8_ = cv::Mat(expected_height_, width_, CV_8UC1);
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    return false;
  }

  // The frame size is only known once the format has been negotiated.
  applyPipelineConfig();

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
  header.frame_id = frame_id_;
#ifdef ROS2
  header.stamp = this->now();
#else
  header.stamp = ros::Time::now();
#endif

  if ((bufferinfo.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) &&
      (bufferinfo.timestamp.tv_sec > 0 || bufferinfo.timestamp.tv_usec > 0))
  {
#ifdef ROS2
    using rclcpp::Time;
#else
    using ros::Time;
#endif
    timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const auto mono = Time(ts.tv_sec, ts.tv_nsec);
    clock_gettime(CLOCK_REALTIME, &ts);
    const auto real = Time(ts.tv_sec, ts.tv_nsec);

    if (real >= mono) {
      const auto diff = real - mono;
#ifdef ROS2
      const auto stamp = Time(v4l2_timeval_to_ns(&bufferinfo.timestamp));
#else
      // Ubuntu 18.04 does not have v4l2_timeval_to_ns() so we expand it here manually
      const uint64_t ns =
        static_cast<uint64_t>(bufferinfo.timestamp.tv_sec) * 1000000000ULL + bufferinfo.timestamp.tv_usec * 1000;
      const auto stamp = Time().fromNSec(ns);
#endif
      header.stamp = stamp + diff;
    }
  }

  cv_bridge::CvImage cv_img;
  cv_img.header = header;

  const bool zoom = zoom_enable_ && sensor_type_ == SensorTypes::Boson320 && video_mode_ == Encoding::RAW16;

  // ---------- Phase A: copy the frame out of the V4L2 buffer ----------
  // The buffer goes back to the driver in Phase B, so nothing published below may be a view into it.
  cv::Mat frame;  // the CV_16UC1 frame handed to the processing stage, only used in RAW16 mode
  if (video_mode_ == Encoding::RAW16) {
    const cv::Mat raw(height_, width_, CV_16UC1, current_buffer, bytesperline_);
    raw(cv::Rect(0, 0, width_, expected_height_)).copyTo(thermal16_);
    frame = thermal16_;
    if (zoom) {
      cv::resize(thermal16_, thermal16_zoom_, publishedSize());
      frame = thermal16_zoom_;
    }
    cv_img.image = frame;
    cv_img.encoding = "mono16";
  } else {  // YUV: the camera already ran its own AGC, so the frame is published as it comes out.
    const cv::Mat thermal_luma(height_ + height_ / 2, width_, CV_8UC1, current_buffer);
    if (yuv_color_) {
      if (is_yv12_) {
        cv::cvtColor(thermal_luma, thermal_rgb_, cv::COLOR_YUV2BGR_YV12);
      } else {
        cv::cvtColor(thermal_luma, thermal_rgb_, cv::COLOR_YUV2BGR_I420);
      }
      cv_img.image = thermal_rgb_(cv::Rect(0, 0, width_, expected_height_));
      cv_img.encoding = "bgr8";
    } else {
      thermal_luma(cv::Rect(0, 0, width_, expected_height_)).copyTo(thermal8_);
      cv_img.image = thermal8_;
      cv_img.encoding = "mono8";
    }
  }

  // ---------- Phase B: hand the buffer back NOW ----------
  if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0) {
    CRAS_ERROR("VIDIOC_QBUF error during recycle.");
  }

  // ---------- Phase C: run the processing stage ----------
  // In YUV mode there is nothing to process: the camera produced the display image itself and only the
  // raw frame is published.
  PipelineOutputs outs;
  bool processed = true;
  if (video_mode_ == Encoding::RAW16) {
    std::lock_guard<std::mutex> lock(mutex_);
    processed = pipeline_.process(frame, outs);
  }
  if (!processed) {
#ifndef ROS2
    CRAS_WARN_THROTTLE(
      1.0, "The frame (%dx%d) does not match the configured image (%dx%d); skipping it.", frame.cols, frame.rows,
      pipeline_.config().size.width, pipeline_.config().size.height);
#else
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "The frame (%dx%d) does not match the configured image (%dx%d); skipping it.",
      frame.cols, frame.rows, pipeline_.config().size.width, pipeline_.config().size.height);
#endif
  }

  // ---------- Phase D: publish ----------
  // image_raw (together with camera_info) always carries data, even when the stage skipped the frame.
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

  if (video_mode_ != Encoding::RAW16 || !processed) {
    return;
  }

  // The images below are buffers owned by the processing stage, so they are published while the stage is
  // still locked against a concurrent reconfiguration.
  std::lock_guard<std::mutex> lock(mutex_);

  if (outs.visual) {
    cv_img.image = outs.visual8;
    cv_img.encoding = "mono8";
    image_pub_visual_.publish(cv_img.toImageMsg());
  }

  if (outs.heatmap) {
    cv_img.image = outs.heatmap8;
    cv_img.encoding = "bgr8";
    image_pub_heatmap_.publish(cv_img.toImageMsg());
  }

  if (outs.temp) {
    cv_img.image = outs.tempImage;
    // The unit (and therefore the depth) the stage produced the image in. Both are only changed under the
    // lock that is held here, so the encoding always matches the pixels.
    cv_img.encoding = temp_encoding_;
    image_pub_temp_.publish(cv_img.toImageMsg());
  }

  // The temperature topics exist exactly when the stage computes absolute values, and they report the
  // bounds of the stretch the visible pixels were made from.
  if (radiometric_ && temp_mode_ != TempMode::None) {
    max_temp_msg_.header = header;
    min_temp_msg_.header = header;
    max_temp_msg_.temperature = outs.max_degC;
    min_temp_msg_.temperature = outs.min_degC;
#ifndef ROS2
    max_temp_pub_.publish(max_temp_msg_);
    min_temp_pub_.publish(min_temp_msg_);
#else
    max_temp_pub_->publish(max_temp_msg_);
    min_temp_pub_->publish(min_temp_msg_);
#endif
    if (outs.probe_valid) {
      ptr_temp_msg_.header = header;
      ptr_temp_msg_.temperature = outs.probe_degC;
#ifndef ROS2
      ptr_temp_pub_.publish(ptr_temp_msg_);
#else
      ptr_temp_pub_->publish(ptr_temp_msg_);
#endif
    }
  }
}

}  // namespace flir_boson_usb

#ifndef ROS2
PLUGINLIB_EXPORT_CLASS(flir_boson_usb::BosonCamera, nodelet::Nodelet)
#else
RCLCPP_COMPONENTS_REGISTER_NODE(flir_boson_usb::BosonCamera)
#endif
