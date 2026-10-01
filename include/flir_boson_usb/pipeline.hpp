// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image processing of the Boson frames, independent of the ROS generation.
 *
 * The whole point of this module is that it contains no ROS type: the normalisation, colourisation and
 * radiometric conversions can therefore be compiled and tested on synthetic frames without a camera and
 * without a running ROS. The node only feeds it a `cv::Mat` and publishes whatever comes out.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace flir_boson_usb {

/**
 * \brief The radiometric unit assumption of the whole package lives here.
 *
 * A radiometric Boson puts the scene temperature into the 16-bit raw counts as hundredths of Kelvin, so
 * 29315 counts is 20 deg C. Nothing else in the package is allowed to divide or multiply by these numbers.
 */
constexpr double kCountsPerDegC = 100.0;
constexpr double kDegCOffset = 273.15;

//! \brief Convert raw 16-bit counts to degrees Celsius.
double countsToDegC(double counts);

//! \brief Convert degrees Celsius to raw 16-bit counts (the inverse of countsToDegC()).
double degCToCounts(double deg_c);

//! \brief What lands on the visual topic.
enum class AgcMode {
  None = 0,  //!< No visual output at all.
  FixedRange = 1,  //!< Stretch the configured temperature limits.
  AutoRange = 2,  //!< Stretch the percentiles of the current frame.
};

//! \brief Whether (and from what) the colourised topic is produced.
enum class HeatmapMode {
  None = 0,
  Visual = 1,  //!< Colourise the visual output.
};

/**
 * \brief What is stamped on top of the colourised image.
 *
 * The numbering has to match the name table in pipeline.cpp. The three contents differ only in which of
 * the readings are stamped; every reading that is stamped is printed in the unit of the temperature mode.
 */
enum class OverlayMode {
  None = 0,
  MinMaxPtr = 1,  //!< Minimum, maximum and probe reading text plus the probe marker.
  MinMax = 2,  //!< Minimum and maximum reading text only, i.e. no probe marker.
  Ptr = 3,  //!< Probe reading text plus the probe marker.
};

/**
 * \brief The unit of the absolute temperature image, and therefore also its depth.
 *
 * The numbering has to match the unit table in pipeline.cpp. The plain units are carried in 32-bit floats,
 * the centi- units in 16-bit integers, which halves the bandwidth at the price of a coarser resolution.
 */
enum class TempMode {
  None = 0,
  DegC = 1,  //!< Degrees Celsius as a 32-bit float image.
  DegK = 2,  //!< Kelvin as a 32-bit float image.
  DegF = 3,  //!< Degrees Fahrenheit as a 32-bit float image.
  CentiDegC = 4,  //!< Hundredths of degree Celsius as a signed 16-bit image.
  CentiDegK = 5,  //!< Hundredths of Kelvin as an unsigned 16-bit image (the raw counts themselves).
  CentiDegF = 6,  //!< Hundredths of degree Fahrenheit as a signed 16-bit image.
};

/**
 * \brief The palette the colourised image is painted with.
 *
 * The values are the colour maps of cv::applyColorMap() and their numbering has to match the palette table
 * in Pipeline.cpp. A palette the built OpenCV does not have is not offered by colormapNames() and is
 * rejected by colormapFromString().
 */
enum class Colormap {
  Autumn = 0,
  Bone = 1,
  Jet = 2,
  Winter = 3,
  Rainbow = 4,
  Ocean = 5,
  Summer = 6,
  Spring = 7,
  Cool = 8,
  Hsv = 9,
  Pink = 10,
  Hot = 11,
  Parula = 12,
  Magma = 13,
  Inferno = 14,
  Plasma = 15,
  Viridis = 16,
  Cividis = 17,
  Twilight = 18,
  TwilightShifted = 19,
  Turbo = 20,
  DeepGreen = 21,
};

//! \brief The stretch bounds, always in raw 16-bit counts.
struct AgcBounds {
  double min_counts = 0.0;
  double max_counts = 0.0;
};

//! \brief The resolved state of the processing stage.
struct PipelineConfig {
  AgcMode agc_mode = AgcMode::AutoRange;
  HeatmapMode heatmap_mode = HeatmapMode::None;
  OverlayMode overlay_mode = OverlayMode::None;
  //! \brief The palette the heatmap is painted with; honoured when a heatmap is produced.
  Colormap colormap = Colormap::Jet;
  TempMode temp_mode = TempMode::None;
  bool radiometric = false;

  //! \brief Percentiles clipped from the bottom/top of the frame; honoured by AgcMode::AutoRange only.
  double agc_auto_low_pct = 1.0;
  double agc_auto_high_pct = 1.0;

  //! \brief Stretch bounds in degrees Celsius; honoured by AgcMode::FixedRange only.
  double agc_fixed_min_degC = 20.0;
  double agc_fixed_max_degC = 50.0;

  /**
   * \brief How much wider the observed bounds are taken when renormalising, in grey levels of the
   *  active stretch (i.e. 255 is the whole display range). Honoured when agc_norm is set.
   */
  double agc_norm_margin = 20.0;

  //! \brief Renormalise the visual output to the observed frame bounds widened by agc_norm_margin.
  bool agc_norm = false;

  //! \brief The temperature probe, in pixels of the published image.
  int probe_x = 319;
  int probe_y = 255;

  //! \brief The size of the frames handed to process() (the published size, after cropping and zoom).
  cv::Size size{640, 512};
};

//! \brief The images and readings produced by one call of Pipeline::process().
struct PipelineOutputs {
  bool visual = false;  //!< Whether visual8 was produced.
  bool heatmap = false;  //!< Whether heatmap8 was produced.
  bool temp = false;  //!< Whether tempImage was produced.

  cv::Mat visual8;  // CV_8UC1
  cv::Mat heatmap8;  // CV_8UC3
  cv::Mat tempImage;  // the depth of tempDepth(config.temp_mode), in the configured temperature unit

  /**
   * \brief The bounds of the stretch that produced visual8, in degrees Celsius. When there is no
   *  visual output, these are the scene extremes the reported readings describe.
   */
  double min_degC = 0.0;
  double max_degC = 0.0;

  //! \brief The absolute temperature at the probe, in degrees Celsius (only valid when probe_valid).
  double probe_degC = 0.0;
  bool probe_valid = false;
};

/**
 * \brief Build a 65536-bin histogram of the frame into a reusable buffer.
 * \param[in] raw16 The frame, CV_16UC1.
 * \param[in,out] hist The buffer to fill; it is reused when it already has the right size.
 */
void buildHistogram(const cv::Mat& raw16, cv::Mat& hist);

/**
 * \brief Turn a histogram into percentile-clipped stretch bounds.
 * \param[in] hist The 65536-bin histogram built by buildHistogram().
 * \param[in] total_pixels The number of pixels the histogram was computed from.
 * \param[in] clip_low_pct Percentage of the darkest pixels to clip away.
 * \param[in] clip_high_pct Percentage of the brightest pixels to clip away.
 * \return The bounds, guaranteed to have a strictly positive width.
 */
AgcBounds percentileBounds(const cv::Mat& hist, double total_pixels, double clip_low_pct, double clip_high_pct);

/**
 * \brief Widen the given bounds to the observed frame extremes.
 *
 * The observed minimum and maximum of the frame are taken and pushed apart by \p margin grey levels of
 * \p base (the width of \p base divided by 255, times \p margin). If the frame has no width at all (a
 * uniform frame), \p base is returned unchanged so that the caller never divides by zero.
 */
AgcBounds renormalize(const cv::Mat& raw16, const AgcBounds& base, double margin);

/**
 * \brief Linearly map the bounds onto 0..255.
 * \param[in] raw16 The frame, CV_16UC1.
 * \param[in] bounds The stretch bounds in counts.
 * \param[in,out] out8 The CV_8UC1 output (reallocated by OpenCV when needed).
 */
void stretchTo8Bit(const cv::Mat& raw16, cv::Mat& out8, const AgcBounds& bounds);

/**
 * \brief Format a single reading stamped by the overlay.
 *
 * A radiometric camera carries absolute temperatures, so the reading is converted to the unit of \p mode
 * exactly the way the temperature image converts it (including the rounding and saturation of the 16-bit
 * units) and printed with the unit that goes with it: `20.00 deg C` for TempMode::DegC, or the integer
 * `2000 cdeg C` for TempMode::CentiDegC. When the counts cannot be turned into absolute temperatures -- a
 * non-radiometric camera, or a temperature mode with no unit at all -- the raw counts are printed instead,
 * e.g. `29315 counts`.
 * \param[in] counts The reading in raw 16-bit counts.
 * \param[in] mode The unit the reading is printed in.
 * \param[in] radiometric Whether the counts carry absolute temperatures.
 */
std::string formatOverlayValue(double counts, TempMode mode, bool radiometric);

/**
 * \brief Stamp the reading text and the probe marker selected by the overlay content onto a colour image.
 *
 * \p content selects what is stamped: the bounds text, the probe text and its marker, or both. The lines
 * that are stamped are always laid out from the top of the image downwards in the order Max, Min, Ptr.
 * \param[in,out] image The image to draw on, CV_8UC3.
 * \param[in] content Which of the readings are stamped; OverlayMode::None stamps nothing.
 * \param[in] min_counts The lower bound of the stretch, in raw 16-bit counts.
 * \param[in] max_counts The upper bound of the stretch, in raw 16-bit counts.
 * \param[in] probe_counts The reading at the probe, in raw 16-bit counts.
 * \param[in] probe The probe position in pixels of the image.
 * \param[in] mode The unit the readings are printed in; see formatOverlayValue().
 * \param[in] radiometric Whether the counts carry absolute temperatures.
 */
void drawOverlay(cv::Mat& image, OverlayMode content, double min_counts, double max_counts, double probe_counts,
    cv::Point probe, TempMode mode, bool radiometric);

/**
 * \brief Check whether any of the candidate strings matches any of the regular expressions.
 * \param[in] candidates Strings to inspect (device identification, V4L2 card, driver, bus info).
 * \param[in] patterns Regular expressions; an empty list never matches.
 * \param[out] matchedPattern Set to the first pattern that matched (cleared when nothing matched).
 * \param[out] invalidPattern When not null and a pattern fails to compile, set to that pattern.
 * \return True when at least one candidate matched at least one pattern.
 */
bool matchAnyPattern(const std::vector<std::string>& candidates, const std::vector<std::string>& patterns,
    std::string& matchedPattern, std::string* invalidPattern = nullptr);

//! \brief The palette names accepted by colormapFromString(), in the order of the Colormap values.
const std::vector<std::string>& colormapNames();

/**
 * \brief Turn a palette name into a Colormap.
 * \param[in] name The palette name in lower case, i.e. one of the names listed by colormapNames().
 * \param[out] colormap Set to the requested palette when the name is known.
 * \return False when the name is not a palette of the built OpenCV.
 */
bool colormapFromString(const std::string& name, Colormap& colormap);

//! \brief The cv::applyColorMap() code of the given palette.
int cvColormap(Colormap colormap);

//! \brief The unit names accepted by tempModeFromString(), in the order of the TempMode values.
const std::vector<std::string>& tempModeNames();

/**
 * \brief Turn a unit name into a TempMode.
 * \param[in] name The unit name in lower case, i.e. one of the names listed by tempModeNames().
 * \param[out] mode Set to the requested unit when the name is known.
 * \return False when the name is not a supported unit.
 */
bool tempModeFromString(const std::string& name, TempMode& mode);

//! \brief The OpenCV depth of the temperature image the given unit is carried in.
int tempDepth(TempMode mode);

/**
 * \brief The image encoding of the temperature image the given unit is carried in.
 *
 * The returned string is the encoding name of the corresponding image message (e.g. \c 32FC1); it is a
 * plain string so that this module keeps containing no ROS type.
 */
const char* tempEncoding(TempMode mode);

//! \brief The overlay content names accepted by overlayModeFromString(), in the order of the OverlayMode values.
const std::vector<std::string>& overlayModeNames();

/**
 * \brief Turn an overlay content name into an OverlayMode.
 * \param[in] name The name in lower case, i.e. one of the names listed by overlayModeNames().
 * \param[out] mode Set to the requested content when the name is known.
 * \return False when the name is not a supported overlay content.
 */
bool overlayModeFromString(const std::string& name, OverlayMode& mode);

/**
 * \brief The image processing stage of the driver.
 *
 * The stage owns the reusable histogram buffer and all output images, which are allocated at the correct
 * depth once and then reused, so a frame costs no allocation beyond the images that are actually produced.
 */
class Pipeline {
public:
  //! \brief Set up the stage. Reallocates the outputs when the frame size or the temperature unit changed.
  void configure(const PipelineConfig& config);

  const PipelineConfig& config() const {
    return config_;
  }

  /**
   * \brief Process one frame.
   * \param[in] raw16 The frame at the published resolution, CV_16UC1.
   * \param[out] out The enabled outputs; the disabled ones are left empty. The images are references
   *  to buffers owned by the stage, so they are only valid until the next call of process().
   * \return False when the frame has a different size than configured (and was therefore skipped).
   */
  bool process(const cv::Mat& raw16, PipelineOutputs& out);

private:
  PipelineConfig config_;

  // The reusable buffers. process() only hands the caller a shallow reference to them, so the memory
  // is allocated once by configure() and then kept alive between frames.
  cv::Mat hist_, visual8_, heatmap8_, temp_;
};

}  // namespace flir_boson_usb
