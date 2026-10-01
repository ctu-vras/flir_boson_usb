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
using flir_boson_usb::countsToDegC;
using flir_boson_usb::cvColormap;
using flir_boson_usb::degCToCounts;
using flir_boson_usb::HeatmapMode;
using flir_boson_usb::kCountsPerDegC;
using flir_boson_usb::matchAnyPattern;
using flir_boson_usb::OverlayMode;
using flir_boson_usb::percentileBounds;
using flir_boson_usb::Pipeline;
using flir_boson_usb::PipelineConfig;
using flir_boson_usb::PipelineOutputs;
using flir_boson_usb::TempMode;
using flir_boson_usb::tempDepth;
using flir_boson_usb::tempEncoding;
using flir_boson_usb::tempModeFromString;
using flir_boson_usb::tempModeNames;

constexpr int kRows = 100;
constexpr int kCols = 100;
constexpr double kEps = 1e-6;
// The plain temperature units are carried in 32-bit floats, which cannot resolve a hundredth of Kelvin
// at the top of the temperature range.
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

//! \brief Read one pixel of a temperature image, whatever depth the configured unit is carried in.
double tempPixel(const cv::Mat& temp, const int row, const int col) {
  switch (temp.type()) {
    case CV_32FC1:
      return temp.at<float>(row, col);
    case CV_16SC1:
      return temp.at<int16_t>(row, col);
    case CV_16UC1:
      return temp.at<uint16_t>(row, col);
    default:
      return std::nan("");
  }
}

}  // namespace

TEST(Pipeline, CountsToDegcMatchesTheDocumentedFormula) {
  EXPECT_NEAR(20.0, countsToDegC(29315.0), kEps);
  EXPECT_NEAR(-273.15, countsToDegC(0.0), kEps);
  EXPECT_NEAR(29315.0, degCToCounts(20.0), kEps);
  for (const double degC : {-30.0, 0.0, 20.0, 65.5, 100.0}) {
    EXPECT_NEAR(degC, countsToDegC(degCToCounts(degC)), kEps);
  }
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
  config.min_limit_degC = 20.0;
  config.max_limit_degC = 50.0;
  Pipeline pipeline;
  pipeline.configure(config);

  cv::Mat frame = uniformFrame(0);
  frame.at<uint16_t>(0, 0) = static_cast<uint16_t>(degCToCounts(20.0));
  frame.at<uint16_t>(0, 1) = static_cast<uint16_t>(degCToCounts(50.0));
  frame.at<uint16_t>(5, 5) = static_cast<uint16_t>(degCToCounts(35.0));

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(frame, out));
  ASSERT_TRUE(out.visual);
  EXPECT_EQ(0, out.visual8.at<uint8_t>(0, 0));
  EXPECT_EQ(255, out.visual8.at<uint8_t>(0, 1));
  // The middle of the range lands in the middle of the grey scale.
  EXPECT_GE(out.visual8.at<uint8_t>(5, 5), 127);
  EXPECT_LE(out.visual8.at<uint8_t>(5, 5), 128);
  EXPECT_NEAR(20.0, out.min_degC, kEps);
  EXPECT_NEAR(50.0, out.max_degC, kEps);
}

TEST(Pipeline, ReadingsAreTheBoundsTheVisiblePixelsWereMadeFrom) {
  const cv::Mat frame = outlierFrame();
  for (const AgcMode mode : {AgcMode::FixedRange, AgcMode::AutoRange}) {
    auto config = defaultConfig();
    config.agc_mode = mode;
    config.agc_low_pct = 0.0;
    config.agc_high_pct = 0.0;
    // Make both modes describe the very same stretch so the two runs can share the assertions.
    config.min_limit_degC = countsToDegC(500.0);
    config.max_limit_degC = countsToDegC(65000.0);
    config.radiometric = true;
    config.temp_mode = TempMode::DegC;
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
    EXPECT_NEAR(countsToDegC(500.0), out.min_degC, kEps);
    EXPECT_NEAR(countsToDegC(65000.0), out.max_degC, kEps);
    // The probe is read from the raw counts, not from the stretched image.
    EXPECT_NEAR(countsToDegC(65000.0), out.probe_degC, kEps);
  }
}

TEST(Pipeline, TemperatureImageIsIndependentOfTheAgc) {
  const cv::Mat frame = outlierFrame();
  for (const AgcMode mode : {AgcMode::FixedRange, AgcMode::AutoRange, AgcMode::None}) {
    auto config = defaultConfig();
    config.agc_mode = mode;
    config.radiometric = true;
    config.temp_mode = TempMode::DegC;
    Pipeline pipeline;
    pipeline.configure(config);

    PipelineOutputs out;
    ASSERT_TRUE(pipeline.process(frame, out));
    ASSERT_TRUE(out.temp);
    ASSERT_EQ(CV_32FC1, out.tempImage.type());
    EXPECT_NEAR(countsToDegC(500.0), out.tempImage.at<float>(0, 0), kFloatEps);
    EXPECT_NEAR(countsToDegC(65000.0), out.tempImage.at<float>(kRows - 1, 0), kFloatEps);
  }
}

TEST(Pipeline, TempModeNamesRoundTrip) {
  // The offered list is the parameter surface: everything the node accepts is listed, nothing else.
  const std::vector<std::string> expected{
    "none", "c", "k", "f", "centi_c", "centi_k", "centi_f"};
  EXPECT_EQ(expected, tempModeNames());

  // Every offered name resolves back to the mode it was listed under.
  const std::vector<TempMode> modes{
    TempMode::None, TempMode::DegC, TempMode::DegK, TempMode::DegF, TempMode::CentiDegC, TempMode::CentiDegK,
    TempMode::CentiDegF};
  ASSERT_EQ(expected.size(), modes.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    TempMode mode;
    ASSERT_TRUE(tempModeFromString(expected[i], mode)) << expected[i];
    EXPECT_EQ(static_cast<int>(modes[i]), static_cast<int>(mode)) << expected[i];
  }

  // An unknown name is refused and leaves the value alone. The node lower-cases the name before calling.
  TempMode mode = TempMode::DegK;
  EXPECT_FALSE(tempModeFromString("kelvin", mode));
  EXPECT_FALSE(tempModeFromString("CENTI_C", mode));
  EXPECT_FALSE(tempModeFromString("", mode));
  EXPECT_EQ(static_cast<int>(TempMode::DegK), static_cast<int>(mode));
}

TEST(Pipeline, TemperatureUnitsCarryTheSameScene) {
  // Two known values that every unit can carry: 10000 counts (100 K) in the top half, 30000 counts
  // (300 K) in the bottom half. The extremes of the count range are covered by the saturation test.
  constexpr double kColdCounts = 10000.0;
  constexpr double kWarmCounts = 30000.0;
  cv::Mat frame = uniformFrame(kColdCounts);
  for (int row = kRows / 2; row < kRows; row++) {
    for (int col = 0; col < kCols; col++) {
      frame.at<uint16_t>(row, col) = kWarmCounts;
    }
  }

  struct ExpectedUnit {
    TempMode mode;
    int depth;
    const char* encoding;
    double cold;  // the value of the 10000-counts pixel
    double warm;  // the value of the 30000-counts pixel
  };
  // The counts are hundredths of Kelvin; Fahrenheit is 1.8 times Kelvin, shifted by -459.67 deg F.
  const std::vector<ExpectedUnit> cases{
    {TempMode::DegC, CV_32FC1, "32FC1", kColdCounts / 100.0 - 273.15, kWarmCounts / 100.0 - 273.15},
    {TempMode::DegK, CV_32FC1, "32FC1", kColdCounts / 100.0, kWarmCounts / 100.0},
    {TempMode::DegF, CV_32FC1, "32FC1", kColdCounts / 100.0 * 1.8 - 459.67, kWarmCounts / 100.0 * 1.8 - 459.67},
    {TempMode::CentiDegC, CV_16SC1, "16SC1", kColdCounts - 27315.0, kWarmCounts - 27315.0},
    {TempMode::CentiDegK, CV_16UC1, "16UC1", kColdCounts, kWarmCounts},
    // 100 K is -279.67 deg F and 300 K is 80.33 deg F (the same formula, in hundredths).
    {TempMode::CentiDegF, CV_16SC1, "16SC1", -27967.0, 8033.0},
  };

  for (const auto& expected : cases) {
    auto config = defaultConfig();
    config.agc_mode = AgcMode::None;  // the unit has nothing to do with the stretch
    config.radiometric = true;
    config.temp_mode = expected.mode;
    Pipeline pipeline;
    pipeline.configure(config);

    PipelineOutputs out;
    ASSERT_TRUE(pipeline.process(frame, out));
    ASSERT_TRUE(out.temp);
    EXPECT_EQ(expected.depth, out.tempImage.type()) << expected.encoding;
    EXPECT_EQ(expected.depth, tempDepth(expected.mode)) << expected.encoding;
    EXPECT_STREQ(expected.encoding, tempEncoding(expected.mode));
    EXPECT_EQ(cv::Size(kCols, kRows), out.tempImage.size());

    EXPECT_NEAR(expected.cold, tempPixel(out.tempImage, 0, 0), kFloatEps) << expected.encoding;
    EXPECT_NEAR(expected.warm, tempPixel(out.tempImage, kRows - 1, 0), kFloatEps) << expected.encoding;
  }
}

TEST(Pipeline, CentiUnitsSaturateInsteadOfWrapping) {
  // The extremes of the count range, where the 16-bit units cannot carry the value. OpenCV saturates
  // them into the corner of the range instead of wrapping around to the opposite sign.
  struct Saturation {
    TempMode mode;
    double hot;  // the value of 65535 counts, i.e. 655.35 K
    double cold;  // the value of 0 counts, i.e. -273.15 deg C
  };
  const std::vector<Saturation> cases{
    {TempMode::CentiDegC, 32767.0, -27315.0},  // 382.20 deg C does not fit into a signed 16-bit value
    {TempMode::CentiDegK, 65535.0, 0.0},  // the raw counts are already hundredths of Kelvin
    {TempMode::CentiDegF, 32767.0, -32768.0},  // -459.67 deg F does not fit either
  };

  for (const auto& expected : cases) {
    auto config = defaultConfig();
    config.radiometric = true;
    config.temp_mode = expected.mode;
    Pipeline pipeline;
    pipeline.configure(config);

    PipelineOutputs hot_out;
    ASSERT_TRUE(pipeline.process(uniformFrame(65535), hot_out));
    EXPECT_DOUBLE_EQ(expected.hot, tempPixel(hot_out.tempImage, 0, 0)) << static_cast<int>(expected.mode);

    PipelineOutputs cold_out;
    ASSERT_TRUE(pipeline.process(uniformFrame(0), cold_out));
    EXPECT_DOUBLE_EQ(expected.cold, tempPixel(cold_out.tempImage, 0, 0)) << static_cast<int>(expected.mode);
  }
}

TEST(Pipeline, ReconfiguringTheUnitChangesTheImageDepth) {
  const cv::Mat frame = outlierFrame();
  auto config = defaultConfig();
  config.radiometric = true;
  config.temp_mode = TempMode::DegC;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs floats;
  ASSERT_TRUE(pipeline.process(frame, floats));
  ASSERT_EQ(CV_32FC1, floats.tempImage.type());

  // The frame size stayed the same, so the buffer is only reallocated because of the new depth.
  config.temp_mode = TempMode::CentiDegK;
  pipeline.configure(config);
  PipelineOutputs counts;
  ASSERT_TRUE(pipeline.process(frame, counts));
  ASSERT_EQ(CV_16UC1, counts.tempImage.type());
  EXPECT_DOUBLE_EQ(10000.0, counts.tempImage.at<uint16_t>(0, kCols - 1));

  // And back to a unit that needs no image at all.
  config.temp_mode = TempMode::None;
  pipeline.configure(config);
  PipelineOutputs none;
  ASSERT_TRUE(pipeline.process(frame, none));
  EXPECT_FALSE(none.temp);
}

TEST(Pipeline, AgcNormRenormalizesTheRawCounts) {
  const cv::Mat frame = outlierFrame();

  auto config = defaultConfig();
  config.agc_mode = AgcMode::AutoRange;
  config.agc_low_pct = 5.0;
  config.agc_high_pct = 5.0;
  config.agc_norm = false;
  Pipeline plain;
  plain.configure(config);
  PipelineOutputs plain_out;
  ASSERT_TRUE(plain.process(frame, plain_out));

  // A pixel at the bottom percentile is black when the percentiles drive the stretch.
  EXPECT_EQ(0, plain_out.visual8.at<uint8_t>(0, 20));
  EXPECT_NEAR(countsToDegC(10000.0), plain_out.min_degC, kEps);

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
  EXPECT_NEAR(countsToDegC(0.0), norm_out.min_degC, kEps);
  EXPECT_NEAR(countsToDegC(65535.0), norm_out.max_degC, kEps);
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
  config.agc_low_pct = 99.9999;
  config.agc_high_pct = 0.0001;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(outlierFrame(), out));
  ASSERT_TRUE(out.visual);
  EXPECT_EQ(CV_8UC1, out.visual8.type());
  EXPECT_GT(out.max_degC, out.min_degC);
  EXPECT_NEAR(1.0 / kCountsPerDegC, out.max_degC - out.min_degC, kEps);
}

TEST(Pipeline, NarrowAndDegenerateFixedRangeStayFinite) {
  auto config = defaultConfig();
  config.agc_mode = AgcMode::FixedRange;
  // A stretch far narrower than the frame saturates, but must not produce anything but valid pixels.
  config.min_limit_degC = 25.0;
  config.max_limit_degC = 26.0;
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
  config.max_limit_degC = 25.0;
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
  config.radiometric = false;
  Pipeline pipeline;
  pipeline.configure(config);

  PipelineOutputs out;
  ASSERT_TRUE(pipeline.process(frame, out));
  EXPECT_FALSE(out.visual);
  EXPECT_FALSE(out.heatmap);
  EXPECT_FALSE(out.temp);
  EXPECT_TRUE(out.visual8.empty());
  EXPECT_TRUE(out.heatmap8.empty());
  EXPECT_TRUE(out.tempImage.empty());

  // The heatmap and the overlay hang off the visual output, so they need it to be enabled as well.
  config.agc_mode = AgcMode::AutoRange;
  config.heatmap_mode = HeatmapMode::None;
  pipeline.configure(config);
  PipelineOutputs no_heatmap;
  ASSERT_TRUE(pipeline.process(frame, no_heatmap));
  EXPECT_TRUE(no_heatmap.visual);
  EXPECT_FALSE(no_heatmap.heatmap);
  EXPECT_TRUE(no_heatmap.heatmap8.empty());

  // The metric image is a radiometric feature and is never produced for a non-radiometric camera.
  config.temp_mode = TempMode::DegC;
  config.radiometric = false;
  pipeline.configure(config);
  PipelineOutputs not_radiometric;
  ASSERT_TRUE(pipeline.process(frame, not_radiometric));
  EXPECT_FALSE(not_radiometric.temp);

  config.radiometric = true;
  pipeline.configure(config);
  PipelineOutputs radiometric;
  ASSERT_TRUE(pipeline.process(frame, radiometric));
  EXPECT_TRUE(radiometric.temp);
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
  config.radiometric = true;
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
      EXPECT_NEAR(countsToDegC(50000.0), out.probe_degC, kEps);
    }
  }
}

TEST(Pipeline, MatchAnyPatternReportsTheMatchedPattern) {
  const std::vector<std::string> candidates{
    "usb-FLIR_Boson_640_radiometric_1234-video-index0", "Boson640", "uvcvideo", "usb-0000:00:14.0-2"};
  std::string matched;

  // The by-id identifier is the first candidate and carries the pattern.
  EXPECT_TRUE(matchAnyPattern(candidates, {"[Rr]adiometric"}, matched));
  EXPECT_EQ("[Rr]adiometric", matched);

  // A pattern that only matches one of the V4L2 strings is found as well, and the first pattern that
  // matches anything is the one that gets reported.
  EXPECT_TRUE(matchAnyPattern(candidates, {"^nomatch$", "^Boson640$"}, matched));
  EXPECT_EQ("^Boson640$", matched);

  EXPECT_TRUE(matchAnyPattern(candidates, {"video-index[0-9]+$"}, matched));
  EXPECT_EQ("video-index[0-9]+$", matched);

  // Nothing matches, so the caller has to treat the camera as non-radiometric.
  EXPECT_FALSE(matchAnyPattern(candidates, {"^radiometric$"}, matched));
  EXPECT_TRUE(matched.empty());

  // An empty pattern list never matches, and neither does an empty candidate list.
  EXPECT_FALSE(matchAnyPattern(candidates, {}, matched));
  EXPECT_FALSE(matchAnyPattern({}, {"Boson"}, matched));

  // Empty strings are placeholders, not catch-all patterns.
  EXPECT_FALSE(matchAnyPattern(candidates, {""}, matched));
  EXPECT_FALSE(matchAnyPattern({""}, {"Boson"}, matched));
}

TEST(Pipeline, MatchAnyPatternSurvivesAnInvalidRegex) {
  const std::vector<std::string> candidates{"Boson radiometric 640"};
  std::string matched;
  std::string invalid;

  // A broken pattern is reported and skipped, and the remaining patterns are still evaluated.
  EXPECT_TRUE(matchAnyPattern(candidates, {"[unclosed(", "radiometric"}, matched, &invalid));
  EXPECT_EQ("radiometric", matched);
  EXPECT_EQ("[unclosed(", invalid);

  // When every pattern is broken, the answer is simply "not radiometric".
  matched.clear();
  invalid.clear();
  EXPECT_FALSE(matchAnyPattern(candidates, {"[unclosed(", "(?)"}, matched, &invalid));
  EXPECT_TRUE(matched.empty());
  EXPECT_FALSE(invalid.empty());

  // The out-parameter is optional.
  EXPECT_TRUE(matchAnyPattern(candidates, {"Boson"}, matched));
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
