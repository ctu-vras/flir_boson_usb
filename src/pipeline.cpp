// SPDX-License-Identifier: MIT
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
#include <cstddef>
#include <cstdio>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <flir_boson_usb/pipeline.hpp>

/**
 * \brief Whether the OpenCV we are built against is at least the given version.
 *
 * OpenCV has no version comparison macro of its own, and the newer colour maps have to be compiled out on
 * the older versions this package still supports.
 */
#define FLIR_CV_VERSION_GE(major, minor) \
  (CV_VERSION_MAJOR > (major) || (CV_VERSION_MAJOR == (major) && CV_VERSION_MINOR >= (minor)))

namespace {

//! \brief The number of histogram bins needed to cover every possible value of a 16-bit frame.
constexpr uint32_t kHistSize = static_cast<uint32_t>(std::numeric_limits<uint16_t>::max()) + 1;

//! \brief One row of the palette table; an empty name means the built OpenCV does not have the palette.
struct ColormapEntry {
  const char* name;
  int code;
};

const std::vector<ColormapEntry>& colormapTable() {
  // The position of each row is the value of the equally positioned Colormap enumerator, so a palette the
  // built OpenCV does not know is left in place under an empty name to keep the numbering stable.
  static const std::vector<ColormapEntry> table = {
    {"autumn", cv::COLORMAP_AUTUMN}, {"bone", cv::COLORMAP_BONE},
    {"jet", cv::COLORMAP_JET}, {"winter", cv::COLORMAP_WINTER},
    {"rainbow", cv::COLORMAP_RAINBOW}, {"ocean", cv::COLORMAP_OCEAN},
    {"summer", cv::COLORMAP_SUMMER}, {"spring", cv::COLORMAP_SPRING},
    {"cool", cv::COLORMAP_COOL}, {"hsv", cv::COLORMAP_HSV},
    {"pink", cv::COLORMAP_PINK}, {"hot", cv::COLORMAP_HOT},
#if FLIR_CV_VERSION_GE(4, 0)
    {"parula", cv::COLORMAP_PARULA},
#else
    {"", cv::COLORMAP_JET},
#endif
#if FLIR_CV_VERSION_GE(4, 4)
    {"magma", cv::COLORMAP_MAGMA}, {"inferno", cv::COLORMAP_INFERNO},
    {"plasma", cv::COLORMAP_PLASMA}, {"viridis", cv::COLORMAP_VIRIDIS},
    {"cividis", cv::COLORMAP_CIVIDIS},
#else
    {"", cv::COLORMAP_JET}, {"", cv::COLORMAP_JET}, {"", cv::COLORMAP_JET},
    {"", cv::COLORMAP_JET}, {"", cv::COLORMAP_JET},
#endif
#if FLIR_CV_VERSION_GE(4, 5)
    {"twilight", cv::COLORMAP_TWILIGHT}, {"twilight_shifted", cv::COLORMAP_TWILIGHT_SHIFTED},
    {"turbo", cv::COLORMAP_TURBO}, {"deepgreen", cv::COLORMAP_DEEPGREEN},
#else
    {"", cv::COLORMAP_JET}, {"", cv::COLORMAP_JET}, {"", cv::COLORMAP_JET}, {"", cv::COLORMAP_JET},
#endif
  };
  return table;
}

std::vector<std::string> buildColormapNames() {
  std::vector<std::string> names;
  for (const auto& entry : colormapTable()) {
    if (*entry.name != '\0') {
      names.emplace_back(entry.name);
    }
  }
  return names;
}

struct GainTableEntry {
  flir_boson_usb::GainMode gain_mode;
  const char* name;
  size_t multiplier;  //!< The gain multiplier value.
};

/**
 * \brief The gain modes in the order of the GainMode values.
 */
const std::vector<GainTableEntry>& gainTable() {
  static const std::vector<GainTableEntry> table = {
    {flir_boson_usb::GainMode::Low, "low", 2},
    {flir_boson_usb::GainMode::High, "high", 1},
  };
  return table;
}

const GainTableEntry& gainUnit(const flir_boson_usb::GainMode gain_mode) {
  const auto& table = gainTable();
  const auto index = static_cast<size_t>(gain_mode);
  return index < table.size() ? table[index] : table[0];
}

std::vector<std::string> buildGainModeNames() {
  std::vector<std::string> names;
  for (const auto& unit : gainTable()) {
    names.emplace_back(unit.name);
  }
  return names;
}

//! \brief One row of the temperature unit table.
struct TempUnitEntry {
  const char* name;  //!< The parameter value accepted for this unit.
  const char* encoding;  //!< The image encoding the unit is carried in.
  const char* unit;  //!< The unit the overlay prints the values of this unit in.
  int depth;  //!< The OpenCV depth of the encoding.
  float scale;  //!< The scale applied to Kelvins.
  float offset;  //!< The offset applied to the scaled Kelvins.
};

/**
 * \brief The temperature units in the order of the TempMode values.
 *
 * The raw counts already are hundredths of Kelvin, so every unit is only a rescaling of them.
 */
const std::vector<TempUnitEntry>& tempUnitTable() {
  static const std::vector<TempUnitEntry> table = {
    {"none", "", "", CV_32FC1, 0.0f, 0.0f},  // no temp output
    {"counts", "32FC1", "counts", CV_32FC1, 1.0f, 0.0f},  // counts do not get scaled
    {"c", "32FC1", "deg C", CV_32FC1, 1.0f, -273.15f},
    {"k", "32FC1", "deg K", CV_32FC1, 1.0f, 0.0f},
    {"f", "32FC1", "deg F", CV_32FC1, 1.8f, -459.67f},
  };
  return table;
}

const TempUnitEntry& tempUnit(const flir_boson_usb::TempMode mode) {
  const auto& table = tempUnitTable();
  const auto index = static_cast<size_t>(mode);
  return index < table.size() ? table[index] : table[0];
}

std::vector<std::string> buildTempModeNames() {
  std::vector<std::string> names;
  for (const auto& unit : tempUnitTable()) {
    names.emplace_back(unit.name);
  }
  return names;
}

/**
 * \brief Convert raw counts to the unit of the given mode, the way the temperature image does it.
 *
 * The integer units are rounded and saturated into the range of their depth, so a converted reading is
 * always the value the corresponding pixel of the temperature image carries. TempMode::None has no unit
 * of its own, so the counts pass through unchanged.
 */
float countsToTempUnit(
    const uint16_t counts, const flir_boson_usb::GainMode gain_mode, const flir_boson_usb::TempMode temp_mode) {
  if (temp_mode == flir_boson_usb::TempMode::None) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto [scale, offset] = flir_boson_usb::tempAffine(gain_mode, temp_mode);
  return counts * scale + offset;
}

/**
 * \brief Convert raw counts to the unit of the given mode, the way the temperature image does it.
 *
 * The integer units are rounded and saturated into the range of their depth, so a converted reading is
 * always the value the corresponding pixel of the temperature image carries. TempMode::None has no unit
 * of its own, so the counts pass through unchanged.
 */
uint16_t tempUnitToCounts(
    const float temp, const flir_boson_usb::GainMode gain_mode, const flir_boson_usb::TempMode temp_mode) {
  if (temp_mode == flir_boson_usb::TempMode::None) {
    return 0;
  }
  const auto [scale, offset] = flir_boson_usb::tempAffine(gain_mode, temp_mode);
  const auto counts_float = std::max(0.0f, (temp - offset) / scale);
  return static_cast<uint16_t>(std::min(counts_float, static_cast<float>(std::numeric_limits<uint16_t>::max())));
}

/**
 * \brief The overlay contents in the order of the OverlayMode values.
 *
 * The position of each name is the value of the equally positioned OverlayMode enumerator.
 */
const std::vector<const char*>& overlayModeTable() {
  static const std::vector<const char*> table = {"none", "min_max_ptr", "min_max", "ptr"};
  return table;
}

std::vector<std::string> buildOverlayModeNames() {
  std::vector<std::string> names;
  for (const auto& name : overlayModeTable()) {
    names.emplace_back(name);
  }
  return names;
}

}  // namespace

namespace flir_boson_usb {

const std::vector<std::string>& colormapNames() {
  static const std::vector<std::string> names = buildColormapNames();
  return names;
}

bool colormapFromString(const std::string& name, Colormap& colormap) {
  const auto& table = colormapTable();
  for (size_t i = 0; i < table.size(); ++i) {
    if (*table[i].name != '\0' && name == table[i].name) {
      colormap = static_cast<Colormap>(i);
      return true;
    }
  }
  return false;
}

int cvColormap(const Colormap colormap) {
  const auto& table = colormapTable();
  const auto index = static_cast<size_t>(colormap);
  return index < table.size() ? table[index].code : cv::COLORMAP_JET;
}

const std::vector<std::string>& gainModeNames() {
  static const std::vector<std::string> names = buildGainModeNames();
  return names;
}

bool gainModeFromString(const std::string& name, GainMode& mode) {
  const auto& table = gainTable();
  for (size_t i = 0; i < table.size(); ++i) {
    if (name == table[i].name) {
      mode = static_cast<GainMode>(i);
      return true;
    }
  }
  return false;
}

std::string gainModeToString(const GainMode mode) {
  return gainUnit(mode).name;
}

const std::vector<std::string>& tempModeNames() {
  static const std::vector<std::string> names = buildTempModeNames();
  return names;
}

bool tempModeFromString(const std::string& name, TempMode& mode) {
  const auto& table = tempUnitTable();
  for (size_t i = 0; i < table.size(); ++i) {
    if (name == table[i].name) {
      mode = static_cast<TempMode>(i);
      return true;
    }
  }
  return false;
}

std::string tempModeToString(const TempMode mode) {
  return tempUnit(mode).name;
}

int tempDepth(const TempMode mode) {
  return tempUnit(mode).depth;
}

const char* tempEncoding(const TempMode mode) {
  return tempUnit(mode).encoding;
}

TempAffine tempAffine(const GainMode gain_mode, const TempMode temp_mode) {
  const auto& gain_unit = gainUnit(gain_mode);
  if (temp_mode == TempMode::None) {
    return TempAffine{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
  } else if (temp_mode == TempMode::Counts) {
    return TempAffine{1.0f * gain_unit.multiplier, 0.0f};
  }
  const auto& temp_unit = tempUnit(temp_mode);
  return TempAffine{temp_unit.scale * gain_unit.multiplier / 100.0f, temp_unit.offset};
}

const std::vector<std::string>& overlayModeNames() {
  static const std::vector<std::string> names = buildOverlayModeNames();
  return names;
}

bool overlayModeFromString(const std::string& name, OverlayMode& mode) {
  const auto& table = overlayModeTable();
  for (size_t i = 0; i < table.size(); ++i) {
    if (name == table[i]) {
      mode = static_cast<OverlayMode>(i);
      return true;
    }
  }
  return false;
}

void buildHistogram(const cv::Mat& raw16, cv::Mat& hist) {
  CV_Assert(raw16.type() == CV_16UC1);

  const int hist_size = kHistSize;
  float range[] = {0, hist_size};
  const float* hist_range = {range};

  // calcHist() reuses the memory of hist when the size and type already match.
  cv::calcHist(&raw16, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);
}

AgcBounds percentileBounds(const cv::Mat& hist, const double total_pixels, const double clip_low_pct,
    const double clip_high_pct) {
  CV_Assert(hist.type() == CV_32FC1 && hist.total() == static_cast<size_t>(kHistSize));

  // A frame without pixels has nothing to clip; keep the full range rather than dividing by zero.
  if (total_pixels <= 0.0) {
    return AgcBounds{0, 65535};
  }

  const double clip_low_count = (clip_low_pct / 100.0) * total_pixels;
  const double clip_high_count = (clip_high_pct / 100.0) * total_pixels;

  uint16_t min_val = 0;
  uint16_t max_val = 65535;
  // double prevents truncation on large sensors
  float current_count = 0.0f;

  // Find bottom percentile
  for (int i = 0; i < kHistSize; i++) {
    current_count += hist.at<float>(i);
    if (current_count > clip_low_count) {
      min_val = i;
      break;
    }
  }

  // Find top percentile
  current_count = 0.0;
  for (int i = kHistSize - 1; i >= 0; i--) {
    current_count += hist.at<float>(i);
    if (current_count > clip_high_count) {
      max_val = i;
      break;
    }
  }

  if (max_val <= min_val) {
    // Prevent division by zero, but stay inside the count range.
    min_val = std::max(0, max_val - 1);
    if (max_val <= min_val) {
      max_val = min_val + 1;
    }
  }

  return AgcBounds{min_val, max_val};
}

AgcBounds renormalize(const cv::Mat& raw16, const AgcBounds& base, const double margin) {
  CV_Assert(raw16.type() == CV_16UC1);

  double obs_min_dbl, obs_max_dbl;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(raw16, &obs_min_dbl, &obs_max_dbl, &min_loc, &max_loc);
  const auto obs_min = raw16.at<uint16_t>(min_loc);
  const auto obs_max = raw16.at<uint16_t>(max_loc);

  // The margin is given in grey levels of the active stretch, so it has to be expressed in counts.
  const auto margin_counts = static_cast<uint16_t>(margin / 255.0 * (base.max_counts - base.min_counts));

  AgcBounds out;
  out.min_counts = obs_min >= margin_counts ? obs_min - margin_counts : 0;
  out.max_counts = static_cast<uint32_t>(obs_max) + margin_counts < 65535 ? obs_max + margin_counts : 65535;

  if (out.max_counts <= out.min_counts) {
    // A uniform frame has no width to renormalise to; keep the incoming bounds.
    return base;
  }

  return out;
}

void castTo8Bit(const cv::Mat& raw16, cv::Mat& out8, const AgcBounds& bounds) {
  CV_Assert(raw16.type() == CV_16UC1);

  const double width = bounds.max_counts - bounds.min_counts;
  if (width <= 0.0) {
    out8.create(raw16.size(), CV_8UC1);
    out8 = cv::Scalar(0);
    return;
  }

  // Scale using SIMD-optimized convertTo
  const double scale = 255.0 / width;
  const double shift = -bounds.min_counts * scale;
  raw16.convertTo(out8, CV_8UC1, scale, shift);
}

std::string formatOverlayValue(
    const uint16_t counts, const GainMode gain_mode, const TempMode temp_mode) {
  if (temp_mode == TempMode::None) {
    return "";
  }

  const double value = countsToTempUnit(counts, gain_mode, temp_mode);

  // Counts are integers in the image, so they are printed as integers as well.
  const bool integral = temp_mode == TempMode::Counts;

  const auto unit = tempUnit(temp_mode).unit;

  char text[64];
  if (integral) {
    std::snprintf(text, sizeof(text), "%.0f %s", value, unit);
  } else {
    std::snprintf(text, sizeof(text), "%.2f %s", value, unit);
  }
  return std::string(text);
}

void drawOverlay(cv::Mat& image, const OverlayMode content, const uint16_t min_counts, const uint16_t max_counts,
    const uint16_t probe_counts, const cv::Point probe, const GainMode gain_mode, const TempMode temp_mode) {
  CV_Assert(image.type() == CV_8UC3);
  if (content == OverlayMode::None) {
    return;
  }

  const bool want_bounds = content == OverlayMode::MinMaxPtr || content == OverlayMode::MinMax;
  const bool want_probe = content == OverlayMode::MinMaxPtr || content == OverlayMode::Ptr;

  const int font = cv::FONT_HERSHEY_SIMPLEX;
  std::vector<std::string> texts;
  if (temp_mode != TempMode::None) {
    if (want_bounds) {
      texts.push_back("Max: " + formatOverlayValue(max_counts, gain_mode, temp_mode));
      texts.push_back("Min: " + formatOverlayValue(min_counts, gain_mode, temp_mode));
    }
    if (want_probe) {
      texts.push_back("Ptr: " + formatOverlayValue(probe_counts, gain_mode, temp_mode));
    }
  }
  for (size_t line = 0; line < texts.size(); ++line) {
    // The palette paints whatever colour the value under the text happens to have, so the text is drawn
    // white on a black outline to stay readable on all of them.
    const cv::Point origin(15, 15 + 15 * static_cast<int>(line));
    cv::putText(image, texts[line], origin, font, 0.4, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(image, texts[line], origin, font, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }

  if (want_probe) {
    cv::circle(image, probe, 3, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::circle(image, probe, 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  }
}

void Pipeline::configure(const PipelineConfig& config) {
  const bool reallocate = config_.size != config.size;
  config_ = config;

  if (reallocate) {
    hist_ = cv::Mat(kHistSize, 1, CV_32FC1);
    visual8_ = cv::Mat(config_.size, CV_8UC1);
    heatmap8_ = cv::Mat(config_.size, CV_8UC3);
  }
}

bool Pipeline::process(const cv::Mat& raw16, PipelineOutputs& out) {
  CV_Assert(raw16.type() == CV_16UC1);

  if (raw16.size() != config_.size) {
    return false;
  }

  out.visual = false;
  out.heatmap = false;
  out.probe_valid = false;

  const bool want_visual = config_.agc_mode != AgcMode::None;
  const bool want_heatmap = want_visual && config_.heatmap_mode != HeatmapMode::None;
  const bool want_overlay = want_heatmap && config_.overlay_mode != OverlayMode::None;
  const bool want_temp = config_.temp_mode != TempMode::None;

  if (!want_visual && !want_temp) {
    // Nothing to compute: only the raw frame the node already has is of interest.
    return true;
  }

  AgcBounds bounds;
  if (want_visual && config_.agc_mode == AgcMode::FixedRange) {
    bounds.min_counts = tempUnitToCounts(
      std::min(config_.agc_fixed_min_degC, config_.agc_fixed_max_degC), config_.gain_mode, TempMode::DegC);
    bounds.max_counts = tempUnitToCounts(
      std::max(config_.agc_fixed_min_degC, config_.agc_fixed_max_degC), config_.gain_mode, TempMode::DegC);
  } else {
    // Either the percentiles drive the stretch, or there is no stretch at all and the reported
    // readings are the only consumer of the scene extremes.
    buildHistogram(raw16, hist_);
    bounds = percentileBounds(
      hist_, static_cast<double>(raw16.rows) * raw16.cols, config_.agc_auto_low_pct, config_.agc_auto_high_pct);
  }

  if (want_visual && config_.agc_norm) {
    bounds = renormalize(raw16, bounds, config_.agc_norm_margin);
  }

  // The reported bounds are exactly the bounds the visible pixels were made from.
  out.min_degC = countsToTempUnit(bounds.min_counts, config_.gain_mode, TempMode::DegC);
  out.max_degC = countsToTempUnit(bounds.max_counts, config_.gain_mode, TempMode::DegC);

  if (want_visual) {
    castTo8Bit(raw16, visual8_, bounds);
    out.visual = true;
    out.visual8 = visual8_;
  }

  const bool probe_ok =
    config_.probe_x >= 0 && config_.probe_x < raw16.cols && config_.probe_y >= 0 && config_.probe_y < raw16.rows;
  uint16_t probe_counts = 0;
  if (want_temp || want_overlay) {
    out.probe_valid = probe_ok;
    if (probe_ok) {
      probe_counts = raw16.at<uint16_t>(config_.probe_y, config_.probe_x);
      out.probe_degC = countsToTempUnit(probe_counts, config_.gain_mode, TempMode::DegC);
    }
  }

  if (want_heatmap) {
    cv::applyColorMap(visual8_, heatmap8_, cvColormap(config_.colormap));
    // Only the probe content needs a probe inside the published image; the bounds text describes the
    // stretch and is stamped whatever the probe point is.
    const bool needs_probe = config_.overlay_mode != OverlayMode::MinMax;
    if (want_overlay && (probe_ok || !needs_probe)) {
      // The stamped readings are in the unit of the temperature image, which is where the numbers
      // published on the temperature topic come from as well.
      drawOverlay(heatmap8_, config_.overlay_mode, bounds.min_counts, bounds.max_counts, probe_counts,
          cv::Point(config_.probe_x, config_.probe_y), config_.gain_mode, config_.temp_mode);
    }
    out.heatmap = true;
    out.heatmap8 = heatmap8_;
  }

  return true;
}

}  // namespace flir_boson_usb
