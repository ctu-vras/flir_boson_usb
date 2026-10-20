// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the ROS-free image pipeline on synthetic frames.
 *
 * The frames are built so that the expected stretch bounds can be computed by hand; the comments next to
 * the assertions spell the arithmetic out so a change in the pipeline cannot quietly redefine it.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <flir_boson_usb/pipeline.hpp>

namespace {

using flir_boson_usb::AgcMode;
using flir_boson_usb::buildHistogram;
using flir_boson_usb::Colormap;
using flir_boson_usb::colormapFromString;
using flir_boson_usb::colormapNames;
using flir_boson_usb::cvColormap;
using flir_boson_usb::formatOverlayValue;
using flir_boson_usb::GainMode;
using flir_boson_usb::gainModeFromString;
using flir_boson_usb::gainModeNames;
using flir_boson_usb::gainModeToString;
using flir_boson_usb::HeatmapMode;
using flir_boson_usb::OverlayMode;
using flir_boson_usb::overlayModeFromString;
using flir_boson_usb::overlayModeNames;
using flir_boson_usb::percentileBounds;
using flir_boson_usb::Pipeline;
using flir_boson_usb::PipelineConfig;
using flir_boson_usb::PipelineOutputs;
using flir_boson_usb::TempMode;
using flir_boson_usb::tempAffine;
using flir_boson_usb::tempDepth;
using flir_boson_usb::tempEncoding;
using flir_boson_usb::tempModeFromString;
using flir_boson_usb::tempModeNames;

constexpr int kRows = 100;
constexpr int kCols = 100;
constexpr double kEps = 1e-6;
// The affine map of the unit table is stored in single precision, so a converted reading is only as exact
// as a float can be at the temperature in question.
constexpr double kFloatEps = 1e-4;

PipelineConfig defaultConfig() {
  PipelineConfig config;
  config.size = cv::Size(kCols, kRows);
  config.probe_x = kCols - 1;
  config.probe_y = kRows - 1;
  return config;
}

//! \brief A frame with a single known value everywhere.
cv::Mat uniformFrame(const uint16_t counts) {
  return cv::Mat(kRows, kCols, CV_16UC1, cv::Scalar(counts));
}

/**
 * \brief A 100x100 frame with a bimodal distribution and a handful of outliers.
 *
 * The histogram is 10 pixels at 500, 4990 at 10000, 4990 at 50000 and 10 at 65000, so clipping 5% from
 * either side skips the outliers while clipping 0% keeps them.
 */
cv::Mat outlierFrame() {
  cv::Mat frame = uniformFrame(10000);
  for (int row = kRows / 2; row < kRows; row++) {
    for (int col = 0; col < kCols; col++) {
      frame.at<uint16_t>(row, col) = 50000;
    }
  }
  for (int col = 0; col < 10; col++) {
    frame.at<uint16_t>(0, col) = 500;
    frame.at<uint16_t>(kRows - 1, col) = 65000;
  }
  return frame;
}

/**
 * \brief The value the stage reports for a count value, i.e. the affine map of the unit table.
 *
 * The readings the stage hands back are always degrees Celsius, whatever unit the temperature image is
 * produced in; this spells the expected number out of the same table the stage reads instead of repeating
 * its constants here.
 */
double degC(const uint16_t counts, const GainMode gain_mode = GainMode::High) {
  const auto affine = tempAffine(gain_mode, TempMode::DegC);
  return counts * affine.scale + affine.offset;
}

//! \brief The count value a temperature configured in degrees Celsius stands for.
uint16_t countsOf(const double deg_c, const GainMode gain_mode = GainMode::High) {
  const auto affine = tempAffine(gain_mode, TempMode::DegC);
  return static_cast<uint16_t>((deg_c - affine.offset) / affine.scale);
}

/**
 * \brief The degrees Celsius a bound of the fixed stretch has to be set to to make the stage pick exactly
 *        the given counts.
 *
 * The stage truncates the counts it computes out of the configured temperature, so a configured bound that
 * is a whole number of counts lands half a count short of it. The middle of the count interval is the value
 * that survives the truncation, and that is what the node has to hand over.
 */
double boundOf(const uint16_t counts, const GainMode gain_mode = GainMode::High) {
  return degC(counts, gain_mode) + 0.5 * tempAffine(gain_mode, TempMode::DegC).scale;
}

/**
 * \brief Stamp the overlay of one configuration onto a copy of its heatmap.
 *
 * The heatmap the pipeline hands out is a reference to a buffer the stage reuses, so a configuration
 * whose stamped image has to be compared with another one has to be stamped separately.
 */
cv::Mat stampHeatmap(const PipelineConfig& config, const cv::Mat& frame) {
  Pipeline pipeline;
  pipeline.configure(config);
  PipelineOutputs out;
  EXPECT_TRUE(pipeline.process(frame, out));
  EXPECT_TRUE(out.heatmap);
  return out.heatmap8.clone();
}

}  // namespace

/**
 * \brief The affine map every consumer of the unit table builds its conversion from: the overlay text of
 *        the stage and, through tempAffine(), also RadiometricFilter.
 *
 * High-gain counts are hundredths of Kelvin, so 29315 counts are 293.15 K. Low gain doubles the scale of
 * every unit, because the very same counts then stand for twice the temperature span, while the
 * Kelvin-based offsets of the units stay where they are.
 */
TEST(Pipeline, TempAffineIsTheGainCompensatedRescaling) {
  EXPECT_NEAR(20.0, degC(29315), kFloatEps);
  EXPECT_NEAR(29315.0, countsOf(20.0), kEps);
  // A temperature only survives the trip through counts up to the resolution of a single count.
  for (const double value : {-30.0, 0.0, 20.0, 65.5, 100.0}) {
    EXPECT_NEAR(value, degC(countsOf(value)), tempAffine(GainMode::High, TempMode::DegC).scale);
  }

  EXPECT_NEAR(0.01, tempAffine(GainMode::High, TempMode::DegC).scale, kFloatEps);
  EXPECT_NEAR(0.02, tempAffine(GainMode::Low, TempMode::DegC).scale, kFloatEps);
  EXPECT_NEAR(29315.0 * 0.02 - 273.15, degC(29315, GainMode::Low), kFloatEps);

  // Counts are the one unit that is not a temperature, so they carry no offset -- only the gain.
  EXPECT_NEAR(1.0, tempAffine(GainMode::High, TempMode::Counts).scale, kFloatEps);
  EXPECT_NEAR(2.0, tempAffine(GainMode::Low, TempMode::Counts).scale, kFloatEps);
  EXPECT_NEAR(0.0, tempAffine(GainMode::Low, TempMode::Counts).offset, kFloatEps);

  EXPECT_NEAR(-273.15, tempAffine(GainMode::Low, TempMode::DegC).offset, kFloatEps);
  EXPECT_NEAR(0.0, tempAffine(GainMode::Low, TempMode::DegK).offset, kFloatEps);
  // Fahrenheit is 1.8 times Kelvin, and the table stores the scale that goes with counts, i.e. divided by
  // the hundredths the counts are carried in.
  EXPECT_NEAR(1.8 / 100.0, tempAffine(GainMode::High, TempMode::DegF).scale, kFloatEps);
  EXPECT_NEAR(1.8 * 2 / 100.0, tempAffine(GainMode::Low, TempMode::DegF).scale, kFloatEps);
  EXPECT_NEAR(-459.67, tempAffine(GainMode::High, TempMode::DegF).offset, kFloatEps);

  // A unit that produces no image has no map at all.
  EXPECT_TRUE(std::isnan(tempAffine(GainMode::High, TempMode::None).scale));
  EXPECT_TRUE(std::isnan(tempAffine(GainMode::High, TempMode::None).offset));
}

TEST(Pipeline, PercentileBoundsClipTheOutliers) {
  cv::Mat hist;
  const cv::Mat frame = outlierFrame();
  buildHistogram(frame, hist);

  // 0% clipping keeps the true extremes of the frame.
  const auto full = percentileBounds(hist, kRows * kCols, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(500.0, full.min_counts);
  EXPECT_DOUBLE_EQ(65000.0, full.max_counts);

  // 5% of 10000 pixels is 500, so the 10 outliers are not enough to reach the clip count and the
  // bounds jump to the bulk of the distribution.
  const auto clipped = percentileBounds(hist, kRows * kCols, 5.0, 5.0);
  EXPECT_DOUBLE_EQ(10000.0, clipped.min_counts);
  EXPECT_DOUBLE_EQ(50000.0, clipped.max_counts);
}

TEST(Pipeline, FixedRangeMapsTheLimitsToTheFullScale) {
  auto config = defaultConfig();
  config.agc_mode = AgcMode::FixedRange;
  config.agc_fixed_min_degC = 20.0;
  config.agc_fixed_max_degC = 50.0;
  Pipeline pipeline;
  pipeline.configure(config);

  // The two configured bounds are what the stage turns into counts, so those counts are what the reported
  // temperatures describe -- not exactly 20 and 50 deg C, because the counts in between do not exist.
  const uint16_t min_counts = countsOf(20.0);
  const uint16_t max_counts = countsOf(50.0);
  cv::Mat frame = uniformFrame(0);
  frame.at<uint16_t>(0, 0) = min_counts;
  frame.at<uint16_t>(0, 1) = max_counts;
  frame.at<uint16_t>(5, 5) = countsOf(35.0);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(frame, out));
  ASSERT_TRUE(out.visual);
  EXPECT_EQ(0, out.visual8.at<uint8_t>(0, 0));
  EXPECT_EQ(255, out.visual8.at<uint8_t>(0, 1));
  // The middle of the range lands in the middle of the grey scale.
  EXPECT_GE(out.visual8.at<uint8_t>(5, 5), 127);
  EXPECT_LE(out.visual8.at<uint8_t>(5, 5), 128);
  EXPECT_NEAR(degC(min_counts), out.min_degC, kFloatEps);
  EXPECT_NEAR(degC(max_counts), out.max_degC, kFloatEps);
  EXPECT_NEAR(20.0, out.min_degC, 0.02);
  EXPECT_NEAR(50.0, out.max_degC, 0.02);
}

TEST(Pipeline, ReadingsAreTheBoundsTheVisiblePixelsWereMadeFrom) {
  const cv::Mat frame = outlierFrame();
  for (const AgcMode mode : {AgcMode::FixedRange, AgcMode::AutoRange}) {
    auto config = defaultConfig();
    config.agc_mode = mode;
    config.agc_auto_low_pct = 0.0;
    config.agc_auto_high_pct = 0.0;
    // Make both modes describe the very same stretch so the two runs can share the assertions.
    config.agc_fixed_min_degC = boundOf(500);
    config.agc_fixed_max_degC = boundOf(65000);
    // The unit of the temperature image has nothing to say about the readings, which are always degrees
    // Celsius; the unit only decides what the overlay text is printed in.
    config.temp_mode = TempMode::DegK;
    config.heatmap_mode = HeatmapMode::Visual;
    config.overlay_mode = OverlayMode::MinMaxPtr;
    // The hottest pixel of the frame, so the probe and the bounds describe the same pixel.
    config.probe_x = 0;
    config.probe_y = kRows - 1;
    Pipeline pipeline;
    pipeline.configure(config);

    PipelineOutputs out;
    ASSERT_TRUE(pipeline.process(frame, out));
    ASSERT_TRUE(out.visual);
    ASSERT_TRUE(out.probe_valid);

    // The stretch covers the whole frame, so the coldest pixel is black and the hottest one is white,
    // and the reported bounds are exactly those two pixels.
    EXPECT_EQ(0, out.visual8.at<uint8_t>(0, 0));
    EXPECT_EQ(255, out.visual8.at<uint8_t>(kRows - 1, 0));
    EXPECT_NEAR(degC(500), out.min_degC, kFloatEps);
    EXPECT_NEAR(degC(65000), out.max_degC, kFloatEps);
    // The probe is read from the raw counts, not from the stretched image.
    EXPECT_NEAR(degC(65000), out.probe_degC, kFloatEps);
  }
}

/**
 * \brief The readings are gain compensated, and so are the stretch bounds configured in degrees Celsius.
 *
 * The very same counts stand for half the temperature span at high gain, so one scene has to be reported
 * with two different sets of temperatures depending on the gain the camera ran with -- while the picture
 * itself, which is stretched in counts, stays the same.
 */
TEST(Pipeline, ReadingsFollowTheConfiguredGain) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.agc_auto_low_pct = 0.0;
  config.agc_auto_high_pct = 0.0;

  config.gain_mode = GainMode::High;
  Pipeline high;
  high.configure(config);
  PipelineOutputs high_out;
  ASSERT_TRUE(high.process(frame, high_out));

  config.gain_mode = GainMode::Low;
  Pipeline low;
  low.configure(config);
  PipelineOutputs low_out;
  ASSERT_TRUE(low.process(frame, low_out));

  // The stretch is in counts, so the picture is identical whatever gain it is described with ...
  EXPECT_EQ(0.0, cv::sum(high_out.visual8 != low_out.visual8)[0]);
  // ... but the temperatures it stands for are exactly twice as far apart at low gain.
  EXPECT_NEAR(degC(500, GainMode::Low), low_out.min_degC, kFloatEps);
  EXPECT_NEAR(degC(65000, GainMode::Low), low_out.max_degC, kFloatEps);
  EXPECT_NEAR(2.0 * (high_out.max_degC - high_out.min_degC), low_out.max_degC - low_out.min_degC, kFloatEps);
}

/**
 * \brief A temperature mode with no visual output still gets the readings computed.
 *
 * The temperature image itself is produced by RadiometricFilter, but the scalar readings the node publishes
 * come from this stage, and they are needed even when nobody asked for a picture.
 */
TEST(Pipeline, ReadingsAreProducedWithoutAVisualOutput) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::None;
  config.agc_auto_low_pct = 0.0;
  config.agc_auto_high_pct = 0.0;
  config.temp_mode = TempMode::DegC;
  config.probe_x = 0;
  config.probe_y = kRows - 1;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(frame, out));
  EXPECT_FALSE(out.visual);
  EXPECT_FALSE(out.heatmap);
  ASSERT_TRUE(out.probe_valid);
  EXPECT_NEAR(degC(65000), out.probe_degC, kFloatEps);
  // With no stretch to describe, the reported bounds are the extremes of the scene.
  EXPECT_NEAR(degC(500), out.min_degC, kFloatEps);
  EXPECT_NEAR(degC(65000), out.max_degC, kFloatEps);
}

TEST(Pipeline, TempModeNamesRoundTrip) {
  // The offered list is the parameter surface: everything the node accepts is listed, nothing else.
  const std::vector<std::string> expected{"none", "counts", "c", "k", "f"};
  EXPECT_EQ(expected, tempModeNames());

  // Every offered name resolves back to the mode it was listed under.
  const std::vector<TempMode> modes{TempMode::None, TempMode::Counts, TempMode::DegC, TempMode::DegK, TempMode::DegF};
  ASSERT_EQ(expected.size(), modes.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    TempMode mode;
    ASSERT_TRUE(tempModeFromString(expected[i], mode)) << expected[i];
    EXPECT_EQ(static_cast<int>(modes[i]), static_cast<int>(mode)) << expected[i];
    EXPECT_EQ(expected[i], tempModeToString(mode)) << expected[i];
  }

  // An unknown name is refused and leaves the value alone. The node lower-cases the name before calling.
  TempMode mode = TempMode::DegK;
  EXPECT_FALSE(tempModeFromString("kelvin", mode));
  EXPECT_FALSE(tempModeFromString("CENTI_K", mode));
  EXPECT_FALSE(tempModeFromString("", mode));
  EXPECT_EQ(static_cast<int>(TempMode::DegK), static_cast<int>(mode));
}

TEST(Pipeline, GainModeNamesRoundTrip) {
  const std::vector<std::string> expected{"low", "high"};
  EXPECT_EQ(expected, gainModeNames());

  for (const GainMode mode : {GainMode::Low, GainMode::High}) {
    GainMode parsed;
    ASSERT_TRUE(gainModeFromString(expected[static_cast<size_t>(mode)], parsed));
    EXPECT_EQ(static_cast<int>(mode), static_cast<int>(parsed));
    EXPECT_EQ(expected[static_cast<size_t>(mode)], gainModeToString(mode));
  }

  GainMode untouched = GainMode::High;
  EXPECT_FALSE(gainModeFromString("auto", untouched));
  EXPECT_FALSE(gainModeFromString("HIGH", untouched));
  EXPECT_FALSE(gainModeFromString("", untouched));
  EXPECT_EQ(static_cast<int>(GainMode::High), static_cast<int>(untouched));
}

TEST(Pipeline, GainAndUnitDecideTheEncodingTogether) {
  // Every unit is a 32-bit float image now: the centi- units, which needed 16-bit integers, are gone and
  // counts are carried in the same depth as the temperatures.
  const std::vector<std::pair<TempMode, const char*>> units{
    {TempMode::None, ""}, {TempMode::Counts, "32FC1"}, {TempMode::DegC, "32FC1"}, {TempMode::DegK, "32FC1"},
    {TempMode::DegF, "32FC1"}};
  for (const auto& unit : units) {
    EXPECT_EQ(CV_32FC1, tempDepth(unit.first)) << unit.second;
    EXPECT_STREQ(unit.second, tempEncoding(unit.first));
  }
}

TEST(Pipeline, AgcNormRenormalizesTheRawCounts) {
  const cv::Mat frame = outlierFrame();

  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.agc_auto_low_pct = 5.0;
  config.agc_auto_high_pct = 5.0;
  config.agc_norm = false;
  Pipeline plain;
  plain.configure(config);
  PipelineOutputs plain_out;
  ASSERT_TRUE(plain.process(frame, plain_out));

  // A pixel at the bottom percentile is black when the percentiles drive the stretch.
  EXPECT_EQ(0, plain_out.visual8.at<uint8_t>(0, 20));
  EXPECT_NEAR(degC(10000), plain_out.min_degC, kFloatEps);

  // With agc_norm the same raw counts are stretched over the observed extremes widened by 20 grey levels
  // of the previous stretch (20 / 255 * 40000 counts), so the pixel is no longer at the very bottom.
  config.agc_norm = true;
  config.agc_norm_margin = 20.0;
  Pipeline norm;
  norm.configure(config);
  PipelineOutputs norm_out;
  ASSERT_TRUE(norm.process(frame, norm_out));

  EXPECT_LT(norm_out.min_degC, plain_out.min_degC);
  EXPECT_GT(norm_out.max_degC, plain_out.max_degC);
  EXPECT_GT(norm_out.visual8.at<uint8_t>(0, 20), plain_out.visual8.at<uint8_t>(0, 20));
  // The renormalisation is driven by the raw frame extremes, not by the saturated 8-bit image. The
  // widened bounds reach past both ends of the 16-bit count range and are clipped to it.
  EXPECT_NEAR(degC(0), norm_out.min_degC, kFloatEps);
  EXPECT_NEAR(degC(65535), norm_out.max_degC, kFloatEps);
}

TEST(Pipeline, UniformFrameStaysDefined) {
  const cv::Mat frame = uniformFrame(30000);
  for (const bool norm : {false, true}) {
    auto config = defaultConfig();
    config.agc_mode = AgcMode::AutoRange;
    config.agc_norm = norm;
    Pipeline pipeline;
    pipeline.configure(config);

    PipelineOutputs out;
    ASSERT_TRUE(pipeline.process(frame, out));
    ASSERT_TRUE(out.visual);
    ASSERT_EQ(CV_8UC1, out.visual8.type());
    EXPECT_TRUE(std::isfinite(out.min_degC));
    EXPECT_TRUE(std::isfinite(out.max_degC));
    EXPECT_GT(out.max_degC, out.min_degC);
    for (int col = 0; col < kCols; col++) {
      EXPECT_EQ(out.visual8.at<uint8_t>(0, 0), out.visual8.at<uint8_t>(0, col));
    }
  }
}

TEST(Pipeline, DegeneratePercentileClipStaysWithinTheCountRange) {
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  // Clipping away everything leaves no room between the two bounds; the pipeline has to invent a
  // one-count-wide stretch rather than divide by zero.
  config.agc_auto_low_pct = 99.9999;
  config.agc_auto_high_pct = 0.0001;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(outlierFrame(), out));
  ASSERT_TRUE(out.visual);
  EXPECT_EQ(CV_8UC1, out.visual8.type());
  EXPECT_GT(out.max_degC, out.min_degC);
  // The stretch the pipeline invents is exactly one count wide, so the two reported temperatures are one
  // count apart in the unit of the configured gain.
  EXPECT_NEAR(tempAffine(GainMode::High, TempMode::DegC).scale, out.max_degC - out.min_degC, kFloatEps);
}

TEST(Pipeline, NarrowAndDegenerateFixedRangeStayFinite) {
  auto config = defaultConfig();
  config.agc_mode = AgcMode::FixedRange;
  // A stretch far narrower than the frame saturates, but must not produce anything but valid pixels.
  config.agc_fixed_min_degC = 25.0;
  config.agc_fixed_max_degC = 26.0;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(outlierFrame(), out));
  ASSERT_TRUE(out.visual);
  for (int col = 0; col < kCols; col++) {
    const uint8_t value = out.visual8.at<uint8_t>(0, col);
    EXPECT_TRUE(value == 0 || value == 255);
  }

  // A stretch with no width at all cannot be scaled; the output is defined as black.
  config.agc_fixed_max_degC = 25.0;
  pipeline.configure(config);
  PipelineOutputs flat;
  ASSERT_TRUE(pipeline.process(outlierFrame(), flat));
  ASSERT_TRUE(flat.visual);
  EXPECT_DOUBLE_EQ(flat.min_degC, flat.max_degC);
  EXPECT_EQ(0, flat.visual8.at<uint8_t>(50, 50));
}

TEST(Pipeline, DisabledOutputsAreNotProduced) {
  const cv::Mat frame = outlierFrame();

  auto config = defaultConfig();
  config.agc_mode = AgcMode::None;
  config.heatmap_mode = HeatmapMode::Visual;
  config.overlay_mode = OverlayMode::MinMaxPtr;
  config.temp_mode = TempMode::DegC;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(frame, out));
  EXPECT_FALSE(out.visual);
  EXPECT_FALSE(out.heatmap);
  EXPECT_TRUE(out.visual8.empty());
  EXPECT_TRUE(out.heatmap8.empty());

  // The heatmap and the overlay hang off the visual output, so they need it to be enabled as well.
  config.agc_mode = AgcMode::AutoRange;
  config.heatmap_mode = HeatmapMode::None;
  pipeline.configure(config);
  PipelineOutputs no_heatmap;
  ASSERT_TRUE(pipeline.process(frame, no_heatmap));
  EXPECT_TRUE(no_heatmap.visual);
  EXPECT_FALSE(no_heatmap.heatmap);
  EXPECT_TRUE(no_heatmap.heatmap8.empty());
}

TEST(Pipeline, OverlayIsStampedOnTheHeatmap) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.heatmap_mode = HeatmapMode::Visual;
  config.overlay_mode = OverlayMode::MinMaxPtr;
  // The probe sits on a pixel at the very top of the stretch, which the JET palette paints red.
  config.probe_x = 60;
  config.probe_y = 60;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs with_overlay;
  ASSERT_TRUE(pipeline.process(frame, with_overlay));
  ASSERT_TRUE(with_overlay.heatmap);
  ASSERT_EQ(CV_8UC3, with_overlay.heatmap8.type());
  // The stage reuses its buffers, so keep a copy to compare against.
  const cv::Mat stamped = with_overlay.heatmap8.clone();

  config.overlay_mode = OverlayMode::None;
  pipeline.configure(config);
  PipelineOutputs without_overlay;
  ASSERT_TRUE(pipeline.process(frame, without_overlay));
  ASSERT_TRUE(without_overlay.heatmap);

  const cv::Mat diff = stamped != without_overlay.heatmap8;
  const auto changed = cv::sum(diff);
  EXPECT_NE(0.0, changed[0] + changed[1] + changed[2]);
  // The probe marker is drawn where the probe is, and nowhere else.
  EXPECT_EQ(255, stamped.at<cv::Vec3b>(60, 60)[0]);
  EXPECT_EQ(0, without_overlay.heatmap8.at<cv::Vec3b>(60, 60)[0]);
}

TEST(Pipeline, OverlayModeNamesRoundTrip) {
  // The offered list is the parameter surface: everything the node accepts is listed, nothing else.
  const std::vector<std::string> expected{"none", "min_max_ptr", "min_max", "ptr"};
  EXPECT_EQ(expected, overlayModeNames());

  for (const OverlayMode mode : {OverlayMode::None, OverlayMode::MinMaxPtr, OverlayMode::MinMax, OverlayMode::Ptr}) {
    OverlayMode parsed;
    ASSERT_TRUE(overlayModeFromString(expected[static_cast<size_t>(mode)], parsed));
    EXPECT_EQ(static_cast<int>(mode), static_cast<int>(parsed));
  }

  // An unknown name is refused and leaves the value alone. The node lower-cases the name before calling.
  OverlayMode untouched = OverlayMode::MinMaxPtr;
  EXPECT_FALSE(overlayModeFromString("min_maxptr", untouched));
  EXPECT_FALSE(overlayModeFromString("MIN_MAX_PTR", untouched));
  EXPECT_FALSE(overlayModeFromString("", untouched));
  EXPECT_EQ(static_cast<int>(OverlayMode::MinMaxPtr), static_cast<int>(untouched));
}

TEST(Pipeline, OverlayContentSelectsWhatIsStamped) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.heatmap_mode = HeatmapMode::Visual;
  config.temp_mode = TempMode::DegC;
  config.probe_x = 60;
  config.probe_y = 60;

  config.overlay_mode = OverlayMode::None;
  const cv::Mat bare = stampHeatmap(config, frame);
  config.overlay_mode = OverlayMode::MinMaxPtr;
  const cv::Mat min_max_ptr = stampHeatmap(config, frame);
  config.overlay_mode = OverlayMode::MinMax;
  const cv::Mat min_max = stampHeatmap(config, frame);
  config.overlay_mode = OverlayMode::Ptr;
  const cv::Mat ptr = stampHeatmap(config, frame);

  EXPECT_NE(0.0, cv::sum(min_max_ptr != bare)[0]);
  EXPECT_NE(0.0, cv::sum(min_max != bare)[0]);
  EXPECT_NE(0.0, cv::sum(ptr != bare)[0]);

  // The stamped lines are laid out from the top of the image downwards, so a content that leaves a line
  // out draws nothing where that line would be. The probe marker sits at (60, 60), below them all.
  // min_max: the two bounds lines and no marker ...
  EXPECT_NE(0.0, cv::sum(min_max.rowRange(25, 40) != bare.rowRange(25, 40))[0]);
  EXPECT_EQ(0.0, cv::sum(min_max.rowRange(40, kRows) != bare.rowRange(40, kRows))[0]);
  // ptr: a single line, so the band under it is left to the marker alone ...
  EXPECT_EQ(0.0, cv::sum(ptr.rowRange(25, 50) != bare.rowRange(25, 50))[0]);
  EXPECT_NE(0.0, cv::sum(ptr.rowRange(50, kRows) != bare.rowRange(50, kRows))[0]);
  // min_max_ptr: both the second line and the marker are there.
  EXPECT_NE(0.0, cv::sum(min_max_ptr.rowRange(25, 50) != bare.rowRange(25, 50))[0]);
  EXPECT_NE(0.0, cv::sum(min_max_ptr.rowRange(50, kRows) != bare.rowRange(50, kRows))[0]);
  // The marker is the same wherever it is stamped, and only the bounds content leaves the probe out.
  EXPECT_EQ(ptr.at<cv::Vec3b>(60, 60), min_max_ptr.at<cv::Vec3b>(60, 60));
  EXPECT_EQ(bare.at<cv::Vec3b>(60, 60), min_max.at<cv::Vec3b>(60, 60));
}

TEST(Pipeline, BoundsOverlayIsStampedWithoutAProbe) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.heatmap_mode = HeatmapMode::Visual;
  config.temp_mode = TempMode::DegC;
  // The probe is not in the published image, so there is no probe reading to print.
  config.probe_x = -1;
  config.probe_y = -1;

  config.overlay_mode = OverlayMode::None;
  const cv::Mat bare = stampHeatmap(config, frame);
  config.overlay_mode = OverlayMode::MinMax;
  const cv::Mat min_max = stampHeatmap(config, frame);
  config.overlay_mode = OverlayMode::Ptr;
  const cv::Mat ptr = stampHeatmap(config, frame);
  config.overlay_mode = OverlayMode::MinMaxPtr;
  const cv::Mat min_max_ptr = stampHeatmap(config, frame);

  // The bounds are the stretch the picture was made from, which is known without a probe ... while the
  // contents that print the probe have nothing to print and leave the picture alone.
  EXPECT_NE(0.0, cv::sum(min_max != bare)[0]);
  EXPECT_EQ(0.0, cv::sum(ptr != bare)[0]);
  EXPECT_EQ(0.0, cv::sum(min_max_ptr != bare)[0]);
}

/**
 * \brief The text stamped on the heatmap is the very number the temperature image carries.
 *
 * 30000 counts are 300 K at high gain, i.e. 26.85 deg C and 80.33 deg F. At low gain the same counts are
 * twice the span, i.e. 600 K. Counts are printed without decimals, because that is what they are.
 */
TEST(Pipeline, OverlayValueIsPrintedInTheConfiguredUnit) {
  struct PrintedValue {
    TempMode mode;
    const char* high_gain;
    const char* low_gain;
  };
  const std::vector<PrintedValue> cases{
    {TempMode::None, "", ""},  // no unit at all, so nothing is stamped
    {TempMode::Counts, "30000 counts", "60000 counts"},
    {TempMode::DegC, "26.85 deg C", "326.85 deg C"},
    {TempMode::DegK, "300.00 deg K", "600.00 deg K"},
    {TempMode::DegF, "80.33 deg F", "620.33 deg F"},
  };
  for (const auto& expected : cases) {
    EXPECT_EQ(expected.high_gain, formatOverlayValue(30000, GainMode::High, expected.mode))
      << static_cast<int>(expected.mode);
    EXPECT_EQ(expected.low_gain, formatOverlayValue(30000, GainMode::Low, expected.mode))
      << static_cast<int>(expected.mode);
  }

  // The whole count range survives in the float units, so the extremes are printed as they are.
  EXPECT_EQ("-273.15 deg C", formatOverlayValue(0, GainMode::High, TempMode::DegC));
  EXPECT_EQ("655.35 deg K", formatOverlayValue(65535, GainMode::High, TempMode::DegK));
  EXPECT_EQ("0 counts", formatOverlayValue(0, GainMode::Low, TempMode::Counts));
}

TEST(Pipeline, OverlayTextFollowsTheConfiguredUnit) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.agc_auto_low_pct = 0.0;
  config.agc_auto_high_pct = 0.0;
  config.heatmap_mode = HeatmapMode::Visual;
  config.overlay_mode = OverlayMode::MinMaxPtr;
  config.probe_x = 60;
  config.probe_y = 60;

  auto unit_config = config;
  unit_config.temp_mode = TempMode::DegC;
  const cv::Mat degc = stampHeatmap(unit_config, frame);
  unit_config.temp_mode = TempMode::DegK;
  const cv::Mat degk = stampHeatmap(unit_config, frame);
  unit_config.temp_mode = TempMode::Counts;
  const cv::Mat counts = stampHeatmap(unit_config, frame);
  unit_config.temp_mode = TempMode::DegC;
  unit_config.gain_mode = GainMode::Low;
  const cv::Mat low_gain = stampHeatmap(unit_config, frame);

  // The unit and the gain change the stamped text, so all four heatmaps differ from each other ...
  EXPECT_NE(0.0, cv::sum(degc != degk)[0]);
  EXPECT_NE(0.0, cv::sum(degc != counts)[0]);
  EXPECT_NE(0.0, cv::sum(degc != low_gain)[0]);
  // ... but only the text: the picture the readings are stamped on is the same stretch, because the
  // stretch itself is in counts.
  EXPECT_EQ(0.0, cv::sum(degc.rowRange(70, kRows) != degk.rowRange(70, kRows))[0]);
  EXPECT_EQ(0.0, cv::sum(degc.rowRange(70, kRows) != counts.rowRange(70, kRows))[0]);
  EXPECT_EQ(0.0, cv::sum(degc.rowRange(70, kRows) != low_gain.rowRange(70, kRows))[0]);

  // Turning the overlay off changes nothing but the text.
  config.overlay_mode = OverlayMode::None;
  config.temp_mode = TempMode::DegC;
  Pipeline plain;
  plain.configure(config);
  PipelineOutputs plain_out;
  ASSERT_TRUE(plain.process(frame, plain_out));
  ASSERT_TRUE(plain_out.heatmap);
  EXPECT_NE(0.0, cv::sum(degc != plain_out.heatmap8)[0]);
  EXPECT_EQ(0.0, cv::sum(degc.rowRange(70, kRows) != plain_out.heatmap8.rowRange(70, kRows))[0]);
}

TEST(Pipeline, ColormapNamesRoundTrip) {
  Colormap colormap;
  std::vector<int> codes;
  for (const auto& name : colormapNames()) {
    ASSERT_TRUE(colormapFromString(name, colormap)) << "the offered name " << name << " was not accepted";
    codes.push_back(cvColormap(colormap));
  }
  EXPECT_FALSE(codes.empty());
  // Two palettes must not share a colour map code, or the parameter would be silently aliased.
  std::sort(codes.begin(), codes.end());
  EXPECT_EQ(codes.end(), std::unique(codes.begin(), codes.end()));

  Colormap untouched = Colormap::Jet;
  EXPECT_FALSE(colormapFromString("not_a_palette", untouched));
  EXPECT_FALSE(colormapFromString("", untouched));
  EXPECT_EQ(Colormap::Jet, untouched);
}

TEST(Pipeline, HeatmapFollowsTheConfiguredPalette) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.heatmap_mode = HeatmapMode::Visual;
  Colormap colormap;
  ASSERT_TRUE(colormapFromString("jet", colormap));
  config.colormap = colormap;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs outputs;
  ASSERT_TRUE(pipeline.process(frame, outputs));
  ASSERT_TRUE(outputs.heatmap);
  const cv::Mat painted = outputs.heatmap8.clone();
  // The default palette is still the one the driver painted with before it could be configured.
  cv::Mat expected;
  cv::applyColorMap(outputs.visual8, expected, cv::COLORMAP_JET);
  EXPECT_EQ(0.0, cv::sum(painted != expected)[0]);

  ASSERT_TRUE(colormapFromString("hot", colormap));
  config.colormap = colormap;
  pipeline.configure(config);
  ASSERT_TRUE(pipeline.process(frame, outputs));
  const cv::Mat recoloured = outputs.heatmap8.clone();
  cv::applyColorMap(outputs.visual8, expected, cv::COLORMAP_HOT);
  EXPECT_EQ(0.0, cv::sum(recoloured != expected)[0]);

  // The palette has to actually change the picture, and only the picture: the visual output is what both
  // palettes are applied to.
  const cv::Mat diff = painted != recoloured;
  const auto changed = cv::sum(diff);
  EXPECT_NE(0.0, changed[0] + changed[1] + changed[2]);
}

TEST(Pipeline, ProbeOutsideThePublishedImageIsReported) {
  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.temp_mode = TempMode::DegC;
  config.heatmap_mode = HeatmapMode::Visual;
  config.overlay_mode = OverlayMode::MinMaxPtr;

  // 0 is the last row and column of the published image, 1 is just past it.
  for (const int offset : {0, 1, -5}) {
    config.probe_x = kCols - 1 + offset;
    config.probe_y = kRows - 1 + offset;
    Pipeline pipeline;
    pipeline.configure(config);

    PipelineOutputs out;
    ASSERT_TRUE(pipeline.process(outlierFrame(), out));
    EXPECT_EQ(offset != 1, out.probe_valid);
    if (out.probe_valid) {
      EXPECT_NEAR(degC(50000), out.probe_degC, kFloatEps);
    }
  }
}

TEST(Pipeline, FrameWithUnexpectedSizeIsSkipped) {
  auto config = defaultConfig();
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  EXPECT_FALSE(pipeline.process(cv::Mat(200, 200, CV_16UC1, cv::Scalar(0)), out));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
