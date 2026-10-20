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
 * \brief The unit of the temperature image.
 *
 * The numbering has to match the unit table in pipeline.cpp. Every unit is carried in 32-bit floats, so the
 * units differ only in the affine map they stand for, see tempAffine().
 */
enum class TempMode {
  None = 0,  //!< No temperature output.
  //! Raw counts brought to the high-gain scale (centiK in T-linear mode, otherwise unit-less).
  Counts = 1,
  DegC = 2,  //!< Degrees Celsius.
  DegK = 3,  //!< Kelvin.
  DegF = 4,  //!< Degrees Fahrenheit.
};

enum class GainMode {
  Low = 0,  //!< Low gain (0-1310 K).
  High = 1,  //!< High gain (0-655 K).
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
  uint16_t min_counts = 0;
  uint16_t max_counts = 0;
};

//! \brief The resolved state of the processing stage.
struct PipelineConfig {
  AgcMode agc_mode = AgcMode::AutoRange;
  HeatmapMode heatmap_mode = HeatmapMode::None;
  OverlayMode overlay_mode = OverlayMode::None;
  //! \brief The palette the heatmap is painted with; honoured when a heatmap is produced.
  Colormap colormap = Colormap::Jet;
  GainMode gain_mode = GainMode::High;
  TempMode temp_mode = TempMode::None;
  bool radiometric = false;

  //! \brief Percentiles clipped from the bottom/top of the frame; honoured by AgcMode::AutoRange only.
  double agc_auto_low_pct = 1.0;
  double agc_auto_high_pct = 1.0;

  //! \brief Stretch bounds in degrees Celsius; honoured by AgcMode::FixedRange only.
  float agc_fixed_min_degC = 20.0f;
  float agc_fixed_max_degC = 50.0f;

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

  cv::Mat visual8;  // CV_8UC1
  cv::Mat heatmap8;  // CV_8UC3

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
void castTo8Bit(const cv::Mat& raw16, cv::Mat& out8, const AgcBounds& bounds);

/**
 * \brief Format a single reading stamped by the overlay.
 *
 * The reading is converted from raw counts to the unit of \p temp_mode the same way the temperature image
 * converts its pixels -- with the affine map of tempAffine() for the camera gain -- and printed with the
 * unit that goes with it, e.g. `20.00 deg C` for TempMode::DegC. Counts have no fractional part in the
 * image either, so they are printed as integers: `58630 counts` for TempMode::Counts.
 * \param[in] counts The reading in raw 16-bit counts.
 * \param[in] gain_mode The gain the counts were produced with; it decides the scale, see tempAffine().
 * \param[in] temp_mode The unit the reading is printed in; TempMode::None prints an empty string.
 */
std::string formatOverlayValue(uint16_t counts, GainMode gain_mode, TempMode temp_mode);

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
 * \param[in] gain_mode The gain the counts were produced with; see formatOverlayValue().
 * \param[in] temp_mode The unit the readings are printed in; see formatOverlayValue().
 */
void drawOverlay(cv::Mat& image, OverlayMode content, uint16_t min_counts, uint16_t max_counts,
    uint16_t probe_counts, cv::Point probe, GainMode gain_mode, TempMode temp_mode);

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

//! \brief The gain names accepted by gainModeFromString(), in the order of the GainMode values.
const std::vector<std::string>& gainModeNames();

/**
 * \brief Turn a gain name into a GainMode.
 * \param[in] name The gain name in lower case, i.e. one of the names listed by gainModeNames().
 * \param[out] mode Set to the requested gain when the name is known.
 * \return False when the name is not a supported gain.
 */
bool gainModeFromString(const std::string& name, GainMode& mode);

std::string gainModeToString(GainMode mode);

//! \brief The unit names accepted by tempModeFromString(), in the order of the TempMode values.
const std::vector<std::string>& tempModeNames();

/**
 * \brief Turn a unit name into a TempMode.
 * \param[in] name The unit name in lower case, i.e. one of the names listed by tempModeNames().
 * \param[out] mode Set to the requested unit when the name is known.
 * \return False when the name is not a supported unit.
 */
bool tempModeFromString(const std::string& name, TempMode& mode);

std::string tempModeToString(TempMode mode);

//! \brief The OpenCV depth of the temperature image the given unit is carried in.
int tempDepth(TempMode mode);

/**
 * \brief The image encoding of the temperature image the given unit is carried in.
 *
 * The returned string is the encoding name of the corresponding image message (e.g. \c 32FC1); it is a
 * plain string so that this module keeps containing no ROS type.
 */
const char* tempEncoding(TempMode mode);

/**
 * \brief The affine map the temperature image of a unit is produced with.
 *
 * A temperature image is one rescaling of the raw counts, \c counts * scale + offset. The scale carries the
 * gain the counts were made with, because low-gain counts tick twice as fast as the high-gain ones the units
 * are defined with.
 */
struct TempAffine {
  float scale;  //!< The scale applied to the raw counts.
  float offset;  //!< The offset applied to the scaled raw counts.
};

//! \brief The affine map of the given units; see TempAffine.
TempAffine tempAffine(GainMode gain_mode, TempMode temp_mode);

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
  //! \brief Set up the stage. Reallocates the outputs when the frame size changed.
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
  // is allocated once by configure() and then kept alive between frames. The temperature image is not
  // among them: it is produced by RadiometricFilter, which owns its own buffer.
  cv::Mat hist_, visual8_, heatmap8_;
};

}  // namespace flir_boson_usb
