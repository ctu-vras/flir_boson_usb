// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Turns the raw 16-bit counts of a Boson into a temperature image.
 *
 * The filter is the radiometric stage of the driver expressed as a sensor filter. The driver loads it into a
 * filter chain of its own to produce image_temp, and the same filter can also be put into a chain of its own by
 * the user. See doc/filter_chains.md for where it sits among the other stages.
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <cras_cpp_common/filter_utils/filter_base.hpp>
#include <flir_boson_usb/pipeline.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace flir_boson_usb {

/**
 * \brief A filter turning the raw counts of a Boson into a temperature image.
 *
 * The filter assumes the counts come from a camera in its temperature-linear mode, which is what the whole
 * affine map rests on. It does not care whether the camera is radiometric -- with \c output_unit set to
 * \c counts it hands out the gain-compensated counts, which is also what a non-radiometric camera has to offer.
 *
 * \par Input: The raw counts of a T-linear Boson, in \c mono16.
 * \par Output: The same frame in the configured unit and its encoding.
 *
 * \par Node parameters (in the namespace of this filter within the chain):
 *      `gain` (string, default: high) The gain the camera produced the counts with. Nothing in the pixels says
 *          which gain made them, so the filter has to be told: one of the names listed by gainModeNames().
 *      `output_unit` (string, default: c) The unit the produced image carries. One of the names listed by
 *          tempModeNames(), except \c none. The unit decides the scaling and the encoding together; every unit
 *          is carried in \c 32FC1, and the counts unit only compensates for the gain.
 * \note configure() declares both parameters with a default descriptor, so they are not read-only and
 *       \c ros2 param set accepts them. \c filters::FilterChain, which is what a \c sensor_filters chain and the
 *       chain inside the driver run inside, forwards every change below the filter's prefix to reconfigureCB()
 *       -- as a callback called before the new value is stored, which is why reconfigureCB() reads the values
 *       from its argument and not from the parameter interface. A rejected value leaves the filter on the unit
 *       and the gain it had.
 */
class RadiometricFilter : public cras::FilterBase<sensor_msgs::msg::Image> {
public:
  RadiometricFilter();
  RadiometricFilter(GainMode default_gain_mode, TempMode default_temp_mode);

  /**
   * \brief Convert one frame.
   * \param[in] data_in The frame of raw counts.
   * \param[out] data_out The very same frame, in the configured temperature unit and encoding.
   * \return False when the input is not a whole frame of \c mono16 counts; the frame is then dropped rather
   *  than passed on half-converted.
   */
  bool update(const sensor_msgs::msg::Image& data_in, sensor_msgs::msg::Image& data_out) override;

  /**
   * \brief Apply a new gain mode or temperature unit handed to the filter by the chain it runs in.
   * \param[in] parameters The parameters the chain reports as changed; only \c gain and \c output_unit concern
   *  this filter.
   * \return An unsuccessful result when a reported value is not one the driver knows -- the chain then refuses
   *  the whole \c ros2 param set call and the filter keeps the gain and the unit it had.
   */
  rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters) override;

  /**
   * \brief Set the scaling, the depth and the encoding that go with the given gain mode and temperature unit.
   */
  bool changeUnits(GainMode gain_mode, TempMode temp_mode);

protected:
  bool configure() override;

private:
  void changeUnitsImpl(GainMode gain_mode, TempMode temp_mode);

  GainMode gain_mode_;
  TempMode temp_unit_;
  //! \brief The scale of the configured gain and unit, taken from the unit table of the pipeline by changeUnitsImpl().
  double scale_;
  //! \brief The offset of the configured unit, taken from the unit table of the pipeline by changeUnitsImpl().
  double offset_;
  //! \brief The OpenCV depth the configured unit is carried in.
  int output_depth_;
  //! \brief The image encoding that goes with the configured unit.
  std::string output_encoding_;
  //! \brief The buffer the converted frame is written to, reused across frames.
  cv::Mat image_;
};

}  // namespace flir_boson_usb
