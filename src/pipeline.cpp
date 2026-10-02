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
constexpr int kHistSize = 65536;

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

//! \brief The Fahrenheit scale relative to Kelvin: one kelvin is 1.8 deg F, zero kelvin is -459.67 deg F.
constexpr double kFahrenheitPerKelvin = 1.8;
constexpr double kFahrenheitOffset = 459.67;

//! \brief One row of the temperature unit table.
struct TempUnitEntry {
  const char* name;  //!< The parameter value accepted for this unit.
  const char* encoding;  //!< The image encoding the unit is carried in.
  const char* unit;  //!< The unit the overlay prints the values of this unit in.
  int depth;  //!< The OpenCV depth of the encoding.
  double alpha;  //!< The scale applied to the raw counts.
  double beta;  //!< The offset applied to the scaled raw counts.
};

/**
 * \brief The temperature units in the order of the TempMode values.
 *
 * The raw counts already are hundredths of Kelvin, so every unit is only a rescaling of them: the centi-
 * units differ from the plain ones by the depth they saturate into, and centi_k is the identity.
 */
const std::vector<TempUnitEntry>& tempUnitTable() {
  static const std::vector<TempUnitEntry> table = {
    {"none", "", "counts", CV_8UC1, 1.0, 0.0},  // no temperature image at all
    {"c", "32FC1", "deg C", CV_32FC1, 1.0 / flir_boson_usb::kCountsPerDegC, -flir_boson_usb::kDegCOffset},
    {"k", "32FC1", "deg K", CV_32FC1, 1.0 / flir_boson_usb::kCountsPerDegC, 0.0},
    {"f", "32FC1", "deg F", CV_32FC1, kFahrenheitPerKelvin / flir_boson_usb::kCountsPerDegC, -kFahrenheitOffset},
    {"centi_c", "16SC1", "cdeg C", CV_16SC1, 1.0, -flir_boson_usb::kDegCOffset * flir_boson_usb::kCountsPerDegC},
    {"centi_k", "16UC1", "cdeg K", CV_16UC1, 1.0, 0.0},
    {"centi_f", "16SC1", "cdeg F", CV_16SC1, kFahrenheitPerKelvin,
      -kFahrenheitOffset * flir_boson_usb::kCountsPerDegC},
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
double countsToTempUnit(const double counts, const flir_boson_usb::TempMode mode) {
  const auto& entry = tempUnit(mode);
  const double value = counts * entry.alpha + entry.beta;
  switch (entry.depth) {
    case CV_16SC1:
      return cv::saturate_cast<int16_t>(value);
    case CV_16UC1:
      return cv::saturate_cast<uint16_t>(value);
    default:
      return value;
  }
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

double countsToDegC(const double counts) {
  return counts / kCountsPerDegC - kDegCOffset;
}

double degCToCounts(const double deg_c) {
  return (deg_c + kDegCOffset) * kCountsPerDegC;
}

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

int tempDepth(const TempMode mode) {
  return tempUnit(mode).depth;
}

const char* tempEncoding(const TempMode mode) {
  return tempUnit(mode).encoding;
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
  float range[] = {0, 65536};
  const float* hist_range = {range};

  // calcHist() reuses the memory of hist when the size and type already match.
  cv::calcHist(&raw16, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);
}

AgcBounds percentileBounds(const cv::Mat& hist, const double total_pixels, const double clip_low_pct,
    const double clip_high_pct) {
  CV_Assert(hist.type() == CV_32FC1 && hist.total() == static_cast<size_t>(kHistSize));

  // A frame without pixels has nothing to clip; keep the full range rather than dividing by zero.
  if (total_pixels <= 0.0) {
    return AgcBounds{0.0, 65535.0};
  }

  const double clip_low_count = (clip_low_pct / 100.0) * total_pixels;
  const double clip_high_count = (clip_high_pct / 100.0) * total_pixels;

  double min_val = 0.0;
  double max_val = 65535.0;
  // double prevents truncation on large sensors
  double current_count = 0.0;

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
    min_val = std::max(0.0, max_val - 1.0);
    if (max_val <= min_val) {
      max_val = min_val + 1.0;
    }
  }

  return AgcBounds{min_val, max_val};
}

AgcBounds renormalize(const cv::Mat& raw16, const AgcBounds& base, const double margin) {
  CV_Assert(raw16.type() == CV_16UC1);

  double obs_min = 0.0;
  double obs_max = 0.0;
  cv::minMaxLoc(raw16, &obs_min, &obs_max);

  // The margin is given in grey levels of the active stretch, so it has to be expressed in counts.
  const double margin_counts = (margin / 255.0) * (base.max_counts - base.min_counts);

  AgcBounds out;
  out.min_counts = std::max(0.0, obs_min - margin_counts);
  out.max_counts = std::min(65535.0, obs_max + margin_counts);

  if (out.max_counts <= out.min_counts) {
    // A uniform frame has no width to renormalise to; keep the incoming bounds.
    return base;
  }

  return out;
}

void stretchTo8Bit(const cv::Mat& raw16, cv::Mat& out8, const AgcBounds& bounds) {
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

std::string formatOverlayValue(const double counts, const TempMode mode, const bool radiometric) {
  // Without absolute temperatures there is nothing to convert to, and TempMode::None has no unit either;
  // both cases print the raw counts of the frame.
  const TempMode unit_mode = radiometric ? mode : TempMode::None;
  const auto& entry = tempUnit(unit_mode);
  const double value = countsToTempUnit(counts, unit_mode);

  // The hundredths units are integers in the image, so they are printed as integers as well.
  const bool integral = entry.depth != CV_32FC1 && entry.depth != CV_64FC1;

  char text[64];
  if (integral) {
    std::snprintf(text, sizeof(text), "%.0f %s", value, entry.unit);
  } else {
    std::snprintf(text, sizeof(text), "%.2f %s", value, entry.unit);
  }
  return std::string(text);
}

void drawOverlay(cv::Mat& image, const OverlayMode content, const double min_counts, const double max_counts,
    const double probe_counts, cv::Point probe, const TempMode mode, const bool radiometric) {
  CV_Assert(image.type() == CV_8UC3);
  if (content == OverlayMode::None) {
    return;
  }

  const bool want_bounds = content == OverlayMode::MinMaxPtr || content == OverlayMode::MinMax;
  const bool want_probe = content == OverlayMode::MinMaxPtr || content == OverlayMode::Ptr;

  const int font = cv::FONT_HERSHEY_SIMPLEX;
  std::vector<std::string> texts;
  if (want_bounds) {
    texts.push_back("Max: " + formatOverlayValue(max_counts, mode, radiometric));
    texts.push_back("Min: " + formatOverlayValue(min_counts, mode, radiometric));
  }
  if (want_probe) {
    texts.push_back("Ptr: " + formatOverlayValue(probe_counts, mode, radiometric));
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
  // The temperature buffer depends on the unit depth as well as on the frame size.
  const bool reallocate = config_.size != config.size || config_.temp_mode != config.temp_mode;
  config_ = config;

  if (reallocate) {
    hist_ = cv::Mat(kHistSize, 1, CV_32FC1);
    visual8_ = cv::Mat(config_.size, CV_8UC1);
    heatmap8_ = cv::Mat(config_.size, CV_8UC3);
    if (config_.temp_mode == TempMode::None) {
      temp_.release();
    } else {
      temp_ = cv::Mat(config_.size, tempDepth(config_.temp_mode));
    }
  }
}

bool Pipeline::process(const cv::Mat& raw16, PipelineOutputs& out) {
  CV_Assert(raw16.type() == CV_16UC1);

  if (raw16.size() != config_.size) {
    return false;
  }

  out.visual = false;
  out.heatmap = false;
  out.temp = false;
  out.probe_valid = false;

  const bool want_visual = config_.agc_mode != AgcMode::None;
  const bool want_heatmap = want_visual && config_.heatmap_mode != HeatmapMode::None;
  const bool want_overlay = want_heatmap && config_.overlay_mode != OverlayMode::None;
  const bool want_temp = config_.radiometric && config_.temp_mode != TempMode::None;

  if (!want_visual && !want_temp) {
    // Nothing to compute: only the raw frame the node already has is of interest.
    return true;
  }

  AgcBounds bounds;
  if (want_visual && config_.agc_mode == AgcMode::FixedRange) {
    bounds.min_counts = degCToCounts(std::min(config_.agc_fixed_min_degC, config_.agc_fixed_max_degC));
    bounds.max_counts = degCToCounts(std::max(config_.agc_fixed_min_degC, config_.agc_fixed_max_degC));
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
  out.min_degC = countsToDegC(bounds.min_counts);
  out.max_degC = countsToDegC(bounds.max_counts);

  if (want_visual) {
    stretchTo8Bit(raw16, visual8_, bounds);
    out.visual = true;
    out.visual8 = visual8_;
  }

  const bool probe_ok =
    config_.probe_x >= 0 && config_.probe_x < raw16.cols && config_.probe_y >= 0 && config_.probe_y < raw16.rows;
  double probe_counts = 0.0;
  if (want_temp || want_overlay) {
    out.probe_valid = probe_ok;
    if (probe_ok) {
      probe_counts = raw16.at<uint16_t>(config_.probe_y, config_.probe_x);
      out.probe_degC = countsToDegC(probe_counts);
    }
  }

  if (want_heatmap) {
    cv::applyColorMap(visual8_, heatmap8_, cvColormap(config_.colormap));
    // Only the probe content needs a probe inside the published image; the bounds text describes the
    // stretch and is stamped whatever the probe point is.
    const bool needs_probe = config_.overlay_mode != OverlayMode::MinMax;
    if (want_overlay && (probe_ok || !needs_probe)) {
      // The stamped readings are in the unit of the temperature image, or in counts when the camera
      // cannot be turned into absolute temperatures.
      drawOverlay(heatmap8_, config_.overlay_mode, bounds.min_counts, bounds.max_counts, probe_counts,
          cv::Point(config_.probe_x, config_.probe_y), config_.temp_mode, config_.radiometric);
    }
    out.heatmap = true;
    out.heatmap8 = heatmap8_;
  }

  if (want_temp) {
    // One saturating rescaling of the raw counts, straight into the buffer of the unit's depth.
    const auto& unit = tempUnit(config_.temp_mode);
    raw16.convertTo(temp_, unit.depth, unit.alpha, unit.beta);
    out.temp = true;
    out.tempImage = temp_;
  }

  return true;
}

}  // namespace flir_boson_usb
