// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the filter chain the driver builds for the temperature image.
 *
 * BosonCamera::createFilterChain() configures a filters::FilterChain under the image_temp parameter prefix and
 * the node then pushes its gain and temp_mode into the filter. This test does the same on a plain node, so that
 * the parameter layout the driver relies on, and the plugin name it asks pluginlib for, are checked without a
 * camera. The conversions themselves are checked by test_radiometric_filter.cpp; the frames here only have to be
 * long enough to notice that the chain produced something.
 *
 * The filter is loaded from the plugin description of the installed package, which is why this test deliberately
 * does not link the filter library: a class that is both linked into the executable and loaded from a library
 * makes class_loader report a namespace collision.
 */

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <filters/filter_chain.hpp>

#include <flir_boson_usb/pipeline.hpp>

// The prefix this package is installed to, so that the test can find its own plugin description when it is run
// straight from the build directory. ament_add_gtest() is given the same value in CMakeLists.txt.
#ifndef FLIR_BOSON_USB_PACKAGE_PREFIX
#define FLIR_BOSON_USB_PACKAGE_PREFIX ""
#endif

namespace {

constexpr char kPackagePrefix[] = FLIR_BOSON_USB_PACKAGE_PREFIX;
constexpr char kPackageName[] = "flir_boson_usb";

using flir_boson_usb::GainMode;
using flir_boson_usb::TempMode;
using flir_boson_usb::tempAffine;

using Message = sensor_msgs::msg::Image;
using Chain = filters::FilterChain<Message>;

//! \brief The prefix, name and type the driver gives to createFilterChain(); keep in sync with BosonCamera.cpp.
constexpr char kPrefix[] = "image_temp";
constexpr char kFilterName[] = "radiometry";
constexpr char kFilterType[] = "flir_boson_usb/RadiometricFilter";
//! \brief The names of the two parameters of the one filter of that chain, which the driver sets as well.
constexpr char kGainParam[] = "image_temp.filter1.params.gain";
constexpr char kUnitParam[] = "image_temp.filter1.params.output_unit";

constexpr int kRows = 60;
constexpr int kCols = 80;

/**
 * \brief Put the installation prefix of this package onto AMENT_PREFIX_PATH if it is not there already.
 *
 * class_loader looks the filter up in the ament resource index, so the prefix the plugin description of this
 * package lives under has to be on AMENT_PREFIX_PATH. colcon test arranges that; plain ctest in the build
 * directory does not, and then the prefix recorded at build time is the only place it can be taken from. Has to
 * run before the resource index is read for the first time, i.e. before rclcpp is initialised.
 */
void addPackagePrefixToAmentPath() {
  if (kPackagePrefix[0] == '\0') {
    return;
  }
  const std::filesystem::path packageMarker =
    std::filesystem::path(kPackagePrefix) / "share/ament_index/resource_index/packages" / kPackageName;
  if (!std::filesystem::exists(packageMarker)) {
    return;  // the package was installed elsewhere or not at all -- the chain reports what it did find
  }
  const char* current = std::getenv("AMENT_PREFIX_PATH");
  const std::string paths = current != nullptr ? current : "";
  std::istringstream stream(paths);
  std::string path;
  while (std::getline(stream, path, ':')) {
    if (path == kPackagePrefix) {
      return;
    }
  }
  setenv("AMENT_PREFIX_PATH", (std::string(kPackagePrefix) + (paths.empty() ? "" : ":" + paths)).c_str(), 1);
}

/**
 * \brief A frame where every pixel has its own count value.
 *
 * The counts are 29000 plus the linear index, so a chain that mixed up the rows or the stride could not pass.
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

//! \brief Wrap a CV_16UC1 frame into a mono16 message.
Message makeCountsMessage(const cv::Mat& counts) {
  Message msg;
  msg.header.frame_id = "thermal_camera_optical_frame";
  msg.header.stamp = rclcpp::Time(1234, 567);
  msg.height = static_cast<uint32_t>(counts.rows);
  msg.width = static_cast<uint32_t>(counts.cols);
  msg.encoding = "mono16";
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(counts.cols) * sizeof(uint16_t);
  msg.data.assign(static_cast<size_t>(msg.step) * counts.rows, 0U);
  for (int row = 0; row < counts.rows; ++row) {
    std::memcpy(msg.data.data() + static_cast<size_t>(row) * msg.step, counts.ptr<uint16_t>(row),
        counts.cols * sizeof(uint16_t));
  }
  return msg;
}

/**
 * \brief The value the unit table says a count value stands for.
 *
 * The filter takes the numbers from the same table the overlay of the stage is printed with, so the table is
 * what the test checks against rather than a copy of its constants here.
 */
double expectedValue(const uint16_t counts, const GainMode gain_mode, const TempMode unit) {
  const auto affine = tempAffine(gain_mode, unit);
  return counts * affine.scale + affine.offset;
}

}  // namespace

class RadiometricFilterChainTest : public testing::Test {
protected:
  /**
   * \brief Build the chain the way the driver builds it, and give its filter the given parameters.
   *
   * The name and the type of the filter have to be in the parameter overrides, because FilterChain::configure()
   * walks the overrides looking for <tt>&lt;prefix&gt;.filter&lt;N&gt;.name</tt>; that is what
   * addFiltersToNodeOptions() arranges in the driver. gain and output_unit are set only once the chain runs,
   * which is what createPublishers() does too, so the values reach the filter through its reconfigureCB().
   */
  bool configureChain(const std::string& gain, const std::string& unit) {
    std::vector<rclcpp::Parameter> overrides;
    overrides.emplace_back(std::string(kPrefix) + ".filter1.name", std::string(kFilterName));
    overrides.emplace_back(std::string(kPrefix) + ".filter1.type", std::string(kFilterType));
    node_ = std::make_shared<rclcpp::Node>(
      ::testing::UnitTest::GetInstance()->current_test_info()->name(),
      rclcpp::NodeOptions().parameter_overrides(overrides));
    node_->declare_parameter(std::string(kPrefix) + ".filter1.name", kFilterName);
    node_->declare_parameter(std::string(kPrefix) + ".filter1.type", kFilterType);

    chain_ = std::make_unique<Chain>("sensor_msgs::msg::Image");
    if (!chain_->configure(kPrefix, node_->get_node_logging_interface(), node_->get_node_parameters_interface())) {
      return false;
    }
    if (chain_->get_length() != 1) {
      return false;
    }
    return setFilterParameters({{kGainParam, gain}, {kUnitParam, unit}});
  }

  //! \brief Set parameters of the filter the way createPublishers() does, and report whether it took them.
  bool setFilterParameters(const std::vector<std::pair<std::string, std::string>>& parameters) {
    std::vector<rclcpp::Parameter> message;
    for (const auto& parameter : parameters) {
      message.emplace_back(parameter.first, parameter.second);
    }
    return node_->set_parameters_atomically(message).successful;
  }

  //! \brief The value the chain put into one pixel of the message it produced.
  double pixel(const Message& message, const int row, const int col) {
    const cv::Mat image = cv_bridge::toCvCopy(message)->image;
    EXPECT_EQ(CV_32FC1, image.type());
    return image.at<float>(row, col);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<Chain> chain_;
};

TEST_F(RadiometricFilterChainTest, LoadsTheFilterTheDriverAsksFor) {
  ASSERT_TRUE(configureChain("high", "c"));

  const cv::Mat counts = gradientFrame();
  const Message in = makeCountsMessage(counts);
  Message out;
  ASSERT_TRUE(chain_->update(in, out));

  EXPECT_EQ("32FC1", out.encoding);
  EXPECT_EQ(counts.rows, static_cast<int>(out.height));
  EXPECT_EQ(counts.cols, static_cast<uint32_t>(out.width));
  // The chain passes the header of the incoming frame through, which is what keeps image_temp matched to image_raw.
  EXPECT_EQ("thermal_camera_optical_frame", out.header.frame_id);
  EXPECT_EQ(in.header.stamp.sec, out.header.stamp.sec);
  EXPECT_EQ(in.header.stamp.nanosec, out.header.stamp.nanosec);

  EXPECT_FLOAT_EQ(expectedValue(counts.at<uint16_t>(0, 0), GainMode::High, TempMode::DegC), pixel(out, 0, 0));
  EXPECT_FLOAT_EQ(
    expectedValue(counts.at<uint16_t>(kRows - 1, kCols - 1), GainMode::High, TempMode::DegC),
    pixel(out, kRows - 1, kCols - 1));
}

TEST_F(RadiometricFilterChainTest, AdoptsTheParametersTheDriverPushes) {
  // Low gain counts tick twice as fast, so the same frame has to come out twice as high in the counts unit.
  ASSERT_TRUE(configureChain("low", "counts"));
  const cv::Mat counts = gradientFrame();
  Message out;
  ASSERT_TRUE(chain_->update(makeCountsMessage(counts), out));
  EXPECT_FLOAT_EQ(expectedValue(counts.at<uint16_t>(3, 7), GainMode::Low, TempMode::Counts), pixel(out, 3, 7));

  ASSERT_TRUE(setFilterParameters({{kUnitParam, "f"}}));
  ASSERT_TRUE(chain_->update(makeCountsMessage(counts), out));
  EXPECT_EQ("32FC1", out.encoding);
  EXPECT_FLOAT_EQ(expectedValue(counts.at<uint16_t>(3, 7), GainMode::Low, TempMode::DegF), pixel(out, 3, 7));
}

TEST_F(RadiometricFilterChainTest, RefusesAUnitItDoesNotKnowAndKeepsConverting) {
  ASSERT_TRUE(configureChain("high", "k"));
  const cv::Mat counts = gradientFrame();

  // The hundredths units are gone from the unit table, so what a user of an older driver tries here is refused
  // rather than silently turned into something else -- and it is refused before it reaches the parameter as well.
  EXPECT_FALSE(setFilterParameters({{kUnitParam, "centi_k"}}));
  EXPECT_EQ("k", node_->get_parameter(kUnitParam).as_string());

  Message out;
  ASSERT_TRUE(chain_->update(makeCountsMessage(counts), out));
  EXPECT_EQ("32FC1", out.encoding);
  EXPECT_FLOAT_EQ(expectedValue(counts.at<uint16_t>(10, 20), GainMode::High, TempMode::DegK), pixel(out, 10, 20));
}

TEST_F(RadiometricFilterChainTest, RunsNothingWithoutTheParameterOverrideTheDriverAdds) {
  // The chain takes its list of filters from the parameter overrides, so a node that does not have them -- because
  // nobody added the filter to addFiltersToNodeOptions() -- configures happily and runs no filter at all. That is
  // why createFilterChain() insists on the override and on the number of filters the chain yielded.
  auto node = std::make_shared<rclcpp::Node>(
    ::testing::UnitTest::GetInstance()->current_test_info()->name(), rclcpp::NodeOptions());
  Chain chain("sensor_msgs::msg::Image");
  ASSERT_TRUE(
    chain.configure("image_visual", node->get_node_logging_interface(), node->get_node_parameters_interface()));
  EXPECT_EQ(0u, chain.get_length());
}

int main(int argc, char** argv) {
  addPackagePrefixToAmentPath();
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
