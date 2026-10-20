// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the radiometric sensor filter on synthetic frames.
 *
 * The frames are the same kind of synthetic input test_pipeline.cpp uses, so no camera and no ROS graph are
 * involved: the filter is configured against a plain node that only exists to hold the parameters, and the
 * messages are built by hand.
 *
 * The filter is instantiated directly here, which is why this test links the filter library. The name the
 * driver asks pluginlib for when it builds its chain is a different matter: it is only resolved when the
 * driver starts, and a test that links this library and loads it as a plugin at the same time makes
 * class_loader report a namespace collision.
 */

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <filters/filter_base.hpp>

#include <flir_boson_usb/pipeline.hpp>
#include <flir_boson_usb/radiometric_filter.hpp>

namespace {

using flir_boson_usb::AgcMode;
using flir_boson_usb::GainMode;
using flir_boson_usb::HeatmapMode;
using flir_boson_usb::Pipeline;
using flir_boson_usb::PipelineConfig;
using flir_boson_usb::PipelineOutputs;
using flir_boson_usb::RadiometricFilter;
using flir_boson_usb::TempMode;
using flir_boson_usb::tempAffine;
using flir_boson_usb::tempDepth;
using flir_boson_usb::tempEncoding;

using Message = sensor_msgs::msg::Image;

constexpr int kRows = 60;
constexpr int kCols = 80;
//! \brief The bytes the test pads every row with, so that the stride of the message is not the row width.
constexpr size_t kPadding = 6;
// Every unit of this driver is carried in 32-bit floats, which cannot resolve a hundredth of Kelvin at the
// top of the range.
constexpr double kFloatEps = 1e-4;

/**
 * \brief A frame that is not uniform, so that a filter mixing up rows or the stride cannot pass.
 *
 * The counts are 29000 plus the linear index, i.e. every pixel has its own value and the values increase
 * along the row and then along the column.
 */
cv::Mat gradientFrame() {
  cv::Mat frame(kRows, kCols, CV_16UC1);
  for (int row = 0; row < frame.rows; ++row) {
    for (int col = 0; col < frame.cols; ++col) {
      frame.at<uint16_t>(row, col) = static_cast<uint16_t>(29000 + row * frame.cols + col);
    }
  }
  return frame;
}

/**
 * \brief A frame with two known extremes, so that the filter can be compared with the stage readings.
 *
 * The same frame test_pipeline.cpp uses: 500 counts in the top left corner, 65000 counts in the bottom one
 * and 50000 counts everywhere in between.
 */
cv::Mat outlierFrame() {
  cv::Mat frame(kRows, kCols, CV_16UC1);
  for (int row = 0; row < frame.rows; ++row) {
    for (int col = 0; col < frame.cols; ++col) {
      frame.at<uint16_t>(row, col) = 50000;
    }
  }
  for (int col = 0; col < 10; col++) {
    frame.at<uint16_t>(0, col) = 500;
    frame.at<uint16_t>(kRows - 1, col) = 65000;
  }
  return frame;
}

//! \brief Wrap a CV_16UC1 frame into a mono16 message, with the rows padded by \p padding bytes.
Message makeCountsMessage(const cv::Mat& counts, const size_t padding = kPadding) {
  Message msg;
  msg.header.frame_id = "thermal_camera_optical_frame";
  msg.header.stamp = rclcpp::Time(1234, 567);
  msg.height = static_cast<uint32_t>(counts.rows);
  msg.width = static_cast<uint32_t>(counts.cols);
  msg.encoding = "mono16";
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(counts.cols * sizeof(uint16_t) + padding);
  msg.data.assign(msg.step * counts.rows, 0U);
  for (int row = 0; row < counts.rows; ++row) {
    std::memcpy(msg.data.data() + static_cast<size_t>(row) * msg.step, counts.ptr<uint16_t>(row),
        counts.cols * sizeof(uint16_t));
  }
  return msg;
}

/**
 * \brief The value the unit table says a count value stands for.
 *
 * The filter takes the numbers from the same table the overlay text of the stage is printed with, so the
 * table is what the tests check against rather than a copy of its constants here.
 */
double expectedValue(const uint16_t counts, const GainMode gain_mode, const TempMode unit) {
  const auto affine = tempAffine(gain_mode, unit);
  return counts * affine.scale + affine.offset;
}

}  // namespace

class RadiometricFilterTest : public testing::Test {
protected:
  /**
   * \brief Build a filter and configure it against a node that carries the given parameters.
   *
   * The parameters are handed to the node as overrides, which is how they reach a filter of a chain when a
   * user gives them on the command line or in a parameter file: they are there before the filter declares
   * them, so what the test checks afterwards is what the filter makes of them.
   *
   * \param[in] prefix The parameter prefix the filter is configured with; a chain gives it
   *  <tt>&lt;chain&gt;.filter&lt;N&gt;.params.</tt>
   * \param[in] gain The gain parameter of the filter, or empty to let it keep the one it was built with.
   * \param[in] unit The output_unit parameter of the filter, or empty for the same.
   * \param[in] gain_mode, temp_mode The defaults the filter is constructed with.
   */
  bool configureFilter(const std::string& prefix = "", const std::string& gain = "", const std::string& unit = "",
      const GainMode gain_mode = GainMode::High, const TempMode temp_mode = TempMode::DegC) {
    std::vector<rclcpp::Parameter> overrides;
    if (!gain.empty()) {
      overrides.emplace_back(prefix + "gain", gain);
    }
    if (!unit.empty()) {
      overrides.emplace_back(prefix + "output_unit", unit);
    }
    node_ = std::make_shared<rclcpp::Node>(
        ::testing::UnitTest::GetInstance()->current_test_info()->name(),
        rclcpp::NodeOptions().parameter_overrides(overrides));

    filter_ = std::make_unique<RadiometricFilter>(gain_mode, temp_mode);
    // The qualified call is not decoration: the zero-argument configure() the filter overrides hides the
    // four-argument one it inherits, and the plain call would be refused for that reason alone.
    return filter_->filters::FilterBase<Message>::configure(
      prefix, "radiometric", node_->get_node_logging_interface(), node_->get_node_parameters_interface());
  }

  //! \brief The value the filter put into one pixel of the message it produced.
  double pixel(const Message& message, const int row, const int col) {
    const cv::Mat image = cv_bridge::toCvCopy(message)->image;
    EXPECT_EQ(CV_32FC1, image.type());
    return image.at<float>(row, col);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<RadiometricFilter> filter_;
};

TEST_F(RadiometricFilterTest, ConvertsCountsToDegreesCelsius) {
  ASSERT_TRUE(configureFilter());
  const cv::Mat counts = gradientFrame();
  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(counts), out));

  EXPECT_EQ("32FC1", out.encoding);
  const cv::Mat temperatures = cv_bridge::toCvCopy(out)->image;
  ASSERT_EQ(counts.size(), temperatures.size());
  for (int row = 0; row < counts.rows; ++row) {
    for (int col = 0; col < counts.cols; ++col) {
      EXPECT_NEAR(expectedValue(counts.at<uint16_t>(row, col), GainMode::High, TempMode::DegC),
        temperatures.at<float>(row, col), kFloatEps);
    }
  }
}

TEST_F(RadiometricFilterTest, ConvertsTheKnownPoint) {
  // 29315 counts are hundredths of Kelvin at high gain, i.e. 20 deg C; this is the point the whole package
  // is built on.
  ASSERT_TRUE(configureFilter());
  const cv::Mat counts = cv::Mat(kRows, kCols, CV_16UC1, cv::Scalar(29315));
  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(counts), out));

  EXPECT_NEAR(20.0, pixel(out, 0, 0), kFloatEps);
  EXPECT_NEAR(20.0, pixel(out, kRows - 1, kCols - 1), kFloatEps);
}

TEST_F(RadiometricFilterTest, KeepsTheHeaderAndGeometry) {
  ASSERT_TRUE(configureFilter());
  const Message in = makeCountsMessage(gradientFrame());
  Message out;
  ASSERT_TRUE(filter_->update(in, out));

  EXPECT_EQ(in.header.frame_id, out.header.frame_id);
  EXPECT_EQ(in.header.stamp.sec, out.header.stamp.sec);
  EXPECT_EQ(in.header.stamp.nanosec, out.header.stamp.nanosec);
  EXPECT_EQ(in.height, out.height);
  EXPECT_EQ(in.width, out.width);
  EXPECT_FALSE(out.is_bigendian);
  // The output is continuous, so its stride is exactly its row width -- the padding of the input is gone.
  EXPECT_EQ(static_cast<uint32_t>(out.width) * sizeof(float), out.step);
  EXPECT_EQ(static_cast<size_t>(out.step) * out.height, out.data.size());
}

/**
 * \brief The gain is not a knob of the output; it is the other half of the conversion.
 *
 * The very same counts stand for twice the temperature span at low gain, so a filter that was told the
 * camera ran with low gain has to hand out twice the temperatures.
 */
TEST_F(RadiometricFilterTest, ConvertsAccordingToTheConfiguredGain) {
  ASSERT_TRUE(configureFilter("", "low"));
  const cv::Mat counts = gradientFrame();
  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(counts), out));

  EXPECT_EQ("32FC1", out.encoding);
  const cv::Mat temperatures = cv_bridge::toCvCopy(out)->image;
  for (int row = 0; row < counts.rows; ++row) {
    for (int col = 0; col < counts.cols; ++col) {
      const double shown = temperatures.at<float>(row, col);
      const double high_gain = expectedValue(counts.at<uint16_t>(row, col), GainMode::High, TempMode::DegC);
      EXPECT_NEAR(expectedValue(counts.at<uint16_t>(row, col), GainMode::Low, TempMode::DegC), shown, kFloatEps);
      // Kelvin is what doubles here; degrees Celsius are Kelvin shifted by an offset the gain does not touch.
      EXPECT_NEAR(2.0 * (high_gain + 273.15) - 273.15, shown, kFloatEps);
    }
  }
}

TEST_F(RadiometricFilterTest, ProducesEveryUnitOfTheDriver) {
  const std::vector<std::pair<const char*, TempMode>> units{
    {"counts", TempMode::Counts}, {"c", TempMode::DegC}, {"k", TempMode::DegK}, {"f", TempMode::DegF}};
  const std::vector<std::pair<const char*, GainMode>> gains{
    {std::make_pair("high", GainMode::High)}, {std::make_pair("low", GainMode::Low)}};

  for (const auto& unit : units) {
    for (const auto& gain : gains) {
      ASSERT_TRUE(configureFilter("", gain.first, unit.first)) << unit.first << '/' << gain.first;
      const cv::Mat counts = gradientFrame();
      Message out;
      ASSERT_TRUE(filter_->update(makeCountsMessage(counts), out)) << unit.first << '/' << gain.first;

      EXPECT_EQ(tempEncoding(unit.second), out.encoding) << unit.first;
      const cv::Mat probe(1, 1, tempDepth(unit.second));
      EXPECT_EQ(static_cast<size_t>(probe.elemSize()) * out.width * out.height, out.data.size()) << unit.first;
      // Every unit is carried in 32-bit floats, so the encoding alone cannot tell them apart -- the numbers
      // written into the pixels have to be checked too.
      EXPECT_NEAR(expectedValue(counts.at<uint16_t>(3, 4), gain.second, unit.second), pixel(out, 3, 4), kFloatEps)
        << unit.first << '/' << gain.first;
    }
  }
}

/**
 * \brief The two halves of the driver have to describe the same scene with the same numbers.
 *
 * The scalar readings the node publishes come from the processing stage, the pixels come from this filter,
 * and both are fed the same gain. The stage reports its readings in degrees Celsius whatever unit the
 * temperature image carries, so the filter is deliberately configured to a different unit than the stage to
 * pin that down as well.
 */
TEST_F(RadiometricFilterTest, AgreesWithTheReadingsOfTheStage) {
  const cv::Mat counts = outlierFrame();
  PipelineConfig config;
  config.size = cv::Size(kCols, kRows);
  config.agc_mode = AgcMode::AutoRange;
  config.agc_auto_low_pct = 0.0;
  config.agc_auto_high_pct = 0.0;
  config.heatmap_mode = HeatmapMode::None;
  config.gain_mode = GainMode::Low;
  config.temp_mode = TempMode::DegK;  // the unit of the image says nothing about the unit of the readings
  config.probe_x = 0;
  config.probe_y = kRows - 1;
  Pipeline pipeline;
  pipeline.configure(config);
  PipelineOutputs readings;
  ASSERT_TRUE(pipeline.process(counts, readings));
  ASSERT_TRUE(readings.probe_valid);

  ASSERT_TRUE(configureFilter("", "low", "c"));
  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(counts), out));

  // The bottom left pixel is both the probe and the upper bound of the stretch, the top left one is the
  // lower bound.
  EXPECT_NEAR(readings.probe_degC, pixel(out, config.probe_y, config.probe_x), kFloatEps);
  EXPECT_NEAR(readings.max_degC, pixel(out, kRows - 1, 0), kFloatEps);
  EXPECT_NEAR(readings.min_degC, pixel(out, 0, 0), kFloatEps);
}

/**
 * \brief The unit can be changed while the filter runs.
 *
 * Two things are checked here: that the parameter is not read-only, which is what makes \c ros2 param set
 * accept it at all, and that the filter applies the value the chain reports. The chain calls reconfigureCB()
 * before it stores the new value, so the value has to come from its argument -- reading the node here would
 * not prove the filter does not do that.
 */
TEST_F(RadiometricFilterTest, ReconfiguresTheUnit) {
  const std::string prefix = "image_temp.filter1.params.";
  const std::string name = prefix + "output_unit";
  // The filter declares the parameter itself here, so that what the test checks afterwards is the filter's
  // declaration and not one the test made on its behalf.
  ASSERT_TRUE(configureFilter(prefix));
  const auto result = node_->set_parameter(rclcpp::Parameter(name, "counts"));
  ASSERT_TRUE(result.successful) << result.reason;

  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(gradientFrame()), out));
  EXPECT_EQ(tempEncoding(TempMode::DegC), out.encoding);

  EXPECT_TRUE(filter_->reconfigureCB({rclcpp::Parameter(name, "k")}).successful);
  ASSERT_TRUE(filter_->update(makeCountsMessage(gradientFrame()), out));
  EXPECT_EQ(tempEncoding(TempMode::DegK), out.encoding);
  EXPECT_NEAR(expectedValue(29000, GainMode::High, TempMode::DegK), pixel(out, 0, 0), kFloatEps);
}

//! \brief The gain can be changed the same way, and it changes the numbers without changing the encoding.
TEST_F(RadiometricFilterTest, ReconfiguresTheGain) {
  const std::string prefix = "image_temp.filter1.params.";
  ASSERT_TRUE(configureFilter(prefix));

  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(gradientFrame()), out));
  const double high_gain = pixel(out, 0, 0);

  EXPECT_TRUE(filter_->reconfigureCB({rclcpp::Parameter(prefix + "gain", "low")}).successful);
  ASSERT_TRUE(filter_->update(makeCountsMessage(gradientFrame()), out));
  EXPECT_EQ(tempEncoding(TempMode::DegC), out.encoding);
  EXPECT_NEAR(expectedValue(29000, GainMode::Low, TempMode::DegC), pixel(out, 0, 0), kFloatEps);
  EXPECT_GT(pixel(out, 0, 0), high_gain);
}

//! \brief The chain forwards every parameter whose name contains the filter's prefix; the filter has to pick
//! its own out of that list rather than act on anything that arrives.
TEST_F(RadiometricFilterTest, IgnoresParametersThatAreNotItsOwn) {
  const std::string prefix = "image_temp.filter1.params.";
  ASSERT_TRUE(configureFilter(prefix));

  EXPECT_TRUE(filter_->reconfigureCB(
      {rclcpp::Parameter("image_temp.filter1.name", "radiometric"),
        rclcpp::Parameter("image_temp.filter2.params.output_unit", "k"),
        rclcpp::Parameter(prefix + "somebody_elses_param", 1),
        rclcpp::Parameter(prefix + "output_unit", 20)}).successful);

  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(gradientFrame()), out));
  EXPECT_EQ(tempEncoding(TempMode::DegC), out.encoding);
  EXPECT_NEAR(expectedValue(29000, GainMode::High, TempMode::DegC), pixel(out, 0, 0), kFloatEps);
}

/**
 * \brief A unit or a gain the driver does not know is refused, and the filter keeps what it had.
 *
 * The chain turns an unsuccessful result into a refused \c ros2 param set, so a typo cannot silently leave
 * the filter converting with a scaling that belongs to no unit.
 */
TEST_F(RadiometricFilterTest, KeepsTheOldValuesWhenReconfigurationFails) {
  const std::string prefix = "image_temp.filter1.params.";
  ASSERT_TRUE(configureFilter(prefix, "low", "k"));

  EXPECT_FALSE(filter_->reconfigureCB({rclcpp::Parameter(prefix + "output_unit", "centi_k")}).successful);
  EXPECT_FALSE(filter_->reconfigureCB({rclcpp::Parameter(prefix + "gain", "auto")}).successful);
  // A value of the wrong type never reaches the filter -- the node refuses it while checking the parameter
  // against its declaration -- so the only thing the filter can do with one is to ignore it.
  EXPECT_TRUE(filter_->reconfigureCB({rclcpp::Parameter(prefix + "output_unit", 20)}).successful);
  // One rejected name in the list refuses the whole call, so the valid one next to it must not be applied
  // either -- that is what keeps a failed ros2 param set from leaving anything half-changed.
  EXPECT_FALSE(filter_->reconfigureCB({rclcpp::Parameter(prefix + "output_unit", "f"),
        rclcpp::Parameter(prefix + "gain", "wrong")}).successful);

  Message out;
  ASSERT_TRUE(filter_->update(makeCountsMessage(gradientFrame()), out));
  EXPECT_EQ(tempEncoding(TempMode::DegK), out.encoding);
  EXPECT_NEAR(expectedValue(29000, GainMode::Low, TempMode::DegK), pixel(out, 0, 0), kFloatEps);
}

TEST_F(RadiometricFilterTest, RefusesUnknownParameterValues) {
  EXPECT_FALSE(configureFilter("", "medium", "c"));
  EXPECT_FALSE(configureFilter("", "high", "kelvin"));
}

TEST_F(RadiometricFilterTest, DropsFramesThatAreNotCounts) {
  ASSERT_TRUE(configureFilter());
  Message in = makeCountsMessage(gradientFrame());
  in.encoding = "8UC1";
  Message out;
  EXPECT_FALSE(filter_->update(in, out));

  in = makeCountsMessage(gradientFrame());
  in.is_bigendian = 1;
  EXPECT_FALSE(filter_->update(in, out));
}

TEST_F(RadiometricFilterTest, DropsTruncatedFrames) {
  ASSERT_TRUE(configureFilter());
  Message in = makeCountsMessage(gradientFrame());
  in.data.resize(in.step * (in.height - 1));
  Message out;
  EXPECT_FALSE(filter_->update(in, out));

  in = makeCountsMessage(gradientFrame(), 0);
  in.step = in.width * sizeof(uint16_t) - 1;
  EXPECT_FALSE(filter_->update(in, out));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
