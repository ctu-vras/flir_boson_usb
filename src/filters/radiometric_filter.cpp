// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/*
 * The filter converts sensor_msgs::msg::Image and nothing else, so it is not templated and its implementation
 * lives here rather than in the header, the same way the rest of this package splits a class in two.
 */

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <flir_boson_usb/pipeline.hpp>
#include <flir_boson_usb/radiometric_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace flir_boson_usb {

RadiometricFilter::RadiometricFilter() : RadiometricFilter(GainMode::High, TempMode::DegC) {}

RadiometricFilter::RadiometricFilter(const GainMode default_gain_mode, const TempMode default_temp_mode)
    : gain_mode_(default_gain_mode), temp_unit_(default_temp_mode), scale_(tempAffine(gain_mode_, temp_unit_).scale),
      offset_(tempAffine(gain_mode_, temp_unit_).offset), output_depth_(tempDepth(temp_unit_)),
      output_encoding_(tempEncoding(temp_unit_)) {}

bool RadiometricFilter::configure() {
  if (!cras::FilterBase<sensor_msgs::msg::Image>::configure()) {
    return false;
  }

  auto gain_mode = gain_mode_;
  auto temp_unit = temp_unit_;

  const auto gain_mode_str = declareParameter("gain", gainModeToString(gain_mode)).get<std::string>();
  const auto temp_unit_str = declareParameter("output_unit", tempModeToString(temp_unit)).get<std::string>();

  if (!gainModeFromString(gain_mode_str, gain_mode)) {
    RCLCPP_ERROR(get_logger(), "gain must be one of the gain modes of the driver");
    return false;
  }
  if (!tempModeFromString(temp_unit_str, temp_unit)) {
    RCLCPP_ERROR(get_logger(), "output_unit must be one of the temperature units of the driver");
    return false;
  }

  if (gain_mode != gain_mode_ || temp_unit != temp_unit_) {
    return changeUnits(gain_mode, temp_unit);
  } else {
    changeUnitsImpl(gain_mode, temp_unit);
  }

  return true;
}

rcl_interfaces::msg::SetParametersResult RadiometricFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  auto gain_mode = gain_mode_;
  auto temp_mode = temp_unit_;

  bool unitsChanged {false};

  using rclcpp::ParameterType::PARAMETER_STRING;
  for (const auto& parameter : parameters) {
    if (parameter.get_type() == PARAMETER_STRING && parameter.get_name() == this->getFullParamName("output_unit")) {
      if (!tempModeFromString(parameter.as_string(), temp_mode)) {
        result.reason = "output_unit must be one of the temperature units of the driver";
        result.successful = false;
      } else {
        unitsChanged |= temp_mode != temp_unit_;
      }
    } else if (parameter.get_type() == PARAMETER_STRING && parameter.get_name() == this->getFullParamName("gain")) {
      if (!gainModeFromString(parameter.as_string(), gain_mode)) {
        result.reason = "gain must be one of the gain modes of the driver";
        result.successful = false;
      } else {
        unitsChanged |= gain_mode != gain_mode_;
      }
    }
  }

  if (result.successful && unitsChanged) {
    changeUnitsImpl(gain_mode, temp_mode);
  }

  return result;
}

bool RadiometricFilter::changeUnits(const GainMode gain_mode, const TempMode temp_mode) {
  // The filter applies the units itself. Whether anything in the node it runs in listens to parameter
  // events is not its business, and at configure() time the values it is handed come from that very
  // interface, so the write below only keeps the interface in sync with what the filter does.
  changeUnitsImpl(gain_mode, temp_mode);

  if (!params_interface_) {
    return true;  // Not configured yet, so there is no parameter interface to report the units to.
  }

  return params_interface_->set_parameters_atomically({
      rclcpp::Parameter(getFullParamName("gain"), gainModeToString(gain_mode)),
      rclcpp::Parameter(getFullParamName("output_unit"), tempModeToString(temp_mode)),
  }).successful;
}

void RadiometricFilter::changeUnitsImpl(const GainMode gain_mode, const TempMode temp_mode) {
  const auto affine = tempAffine(gain_mode, temp_mode);
  gain_mode_ = gain_mode;
  temp_unit_ = temp_mode;
  scale_ = affine.scale;
  offset_ = affine.offset;
  output_depth_ = tempDepth(temp_mode);
  output_encoding_ = tempEncoding(temp_mode);
}

bool RadiometricFilter::update(const sensor_msgs::msg::Image& data_in, sensor_msgs::msg::Image& data_out) {
  if (data_in.encoding != sensor_msgs::image_encodings::MONO16 || data_in.is_bigendian != 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid image encoding. Only mono16 is supported.");
    return false;
  }
  const size_t row_bytes = data_in.step;
  if (row_bytes < static_cast<size_t>(data_in.width) * sizeof(uint16_t)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Image step is smaller than expected.");
    return false;
  }
  if (data_in.data.size() < row_bytes * data_in.height) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Image data size is smaller than expected.");
    return false;
  }

  const auto counts = cv_bridge::toCvShare(data_in, nullptr, sensor_msgs::image_encodings::MONO16);
  // One saturating rescaling, the very same the temperature image of the node is made of.
  counts->image.convertTo(image_, output_depth_, scale_, offset_);
  cv_bridge::CvImage(data_in.header, output_encoding_, image_).toImageMsg(data_out);
  return true;
}

}  // namespace flir_boson_usb

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(flir_boson_usb::RadiometricFilter, filters::FilterBase<sensor_msgs::msg::Image>)
