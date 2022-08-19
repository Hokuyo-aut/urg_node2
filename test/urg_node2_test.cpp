// Copyright 2022 eSOL Co.,Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "urg_node2/urg_node2.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;

double test_scan_period = 10.0;

bool scan_flag = false;
int receive_count = 0;

rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
rclcpp::Time scan_time;

// single scan callback
sensor_msgs::msg::LaserScan scan_msg;
void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (scan_flag) {
    // store last receive scan topic
    scan_msg = *msg;
    scan_time = system_clock.now();
    receive_count++;
  }
}

// multiecho scan callback
sensor_msgs::msg::MultiEchoLaserScan multiecho_msg;
void multiecho_callback(const sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg)
{
  if (scan_flag) {
    // store last receive multi echo scan topic
    multiecho_msg = *msg;
    scan_time = system_clock.now();
    receive_count++;
  }
}

// first(multiecho) scan callback
sensor_msgs::msg::LaserScan first_msg;
void first_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (scan_flag) {
    first_msg = *msg;
    receive_count++;
  }
}

// last(multiecho) scan callback
sensor_msgs::msg::LaserScan last_msg;
void last_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (scan_flag) {
    last_msg = *msg;
    receive_count++;
  }
}

// most_intense(multiecho) scan callback
sensor_msgs::msg::LaserScan most_msg;
void most_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (scan_flag) {
    most_msg = *msg;
    receive_count++;
  }
}

void scan_wait(
  rclcpp::executors::SingleThreadedExecutor & exe1, double wait_period)
{
  // queue flush
  exe1.spin_some();

  // timer count
  rclcpp::Time prev_time = system_clock.now();

  // start receive scan count
  scan_flag = true;

  // receive scan for wait_period seconds
  while (rclcpp::ok()) {
    exe1.spin_some();

    rclcpp::Time current_time = system_clock.now();
    rclcpp::Duration period = current_time - prev_time;
    if (period.seconds() >= wait_period) {
      break;
    }
  }
  scan_flag = false;
}

TEST(UTM_30_LX_EW, normal_scan) {

  // initialize
  scan_msg = sensor_msgs::msg::LaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("ip_address", "192.168.0.10")};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    scan_callback);
  exe1.add_node(sub_node);

  // publisher test
  std::vector<rclcpp::TopicEndpointInfo> ep_scan = node->get_publishers_info_by_topic("scan");
  std::vector<rclcpp::TopicEndpointInfo> ep_multiecho =
    node->get_publishers_info_by_topic("echoes");
  std::vector<rclcpp::TopicEndpointInfo> ep_first = node->get_publishers_info_by_topic("first");
  std::vector<rclcpp::TopicEndpointInfo> ep_last = node->get_publishers_info_by_topic("last");
  std::vector<rclcpp::TopicEndpointInfo> ep_most =
    node->get_publishers_info_by_topic("most_intense");
  std::vector<rclcpp::TopicEndpointInfo> ep_diag =
    node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 0);
  EXPECT_EQ((int)ep_multiecho.size(), 0);
  EXPECT_EQ((int)ep_first.size(), 0);
  EXPECT_EQ((int)ep_last.size(), 0);
  EXPECT_EQ((int)ep_most.size(), 0);
  EXPECT_EQ((int)ep_diag.size(), 0);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // publisher test
  ep_scan = node->get_publishers_info_by_topic("scan");
  ep_multiecho = node->get_publishers_info_by_topic("echoes");
  ep_first = node->get_publishers_info_by_topic("first");
  ep_last = node->get_publishers_info_by_topic("last");
  ep_most = node->get_publishers_info_by_topic("most_intense");
  ep_diag = node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 1);
  EXPECT_EQ((int)ep_multiecho.size(), 0);
  EXPECT_EQ((int)ep_first.size(), 0);
  EXPECT_EQ((int)ep_last.size(), 0);
  EXPECT_EQ((int)ep_most.size(), 0);
  EXPECT_EQ((int)ep_diag.size(), 0);

  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  EXPECT_EQ(receive_count, 0);  // no receive
  receive_count = 0;

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  // publisher test
  ep_scan = node->get_publishers_info_by_topic("scan");
  ep_multiecho = node->get_publishers_info_by_topic("echoes");
  ep_first = node->get_publishers_info_by_topic("first");
  ep_last = node->get_publishers_info_by_topic("last");
  ep_most = node->get_publishers_info_by_topic("most_intense");
  ep_diag = node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 1);
  EXPECT_EQ((int)ep_multiecho.size(), 0);
  EXPECT_EQ((int)ep_first.size(), 0);
  EXPECT_EQ((int)ep_last.size(), 0);
  EXPECT_EQ((int)ep_most.size(), 0);
  EXPECT_EQ((int)ep_diag.size(), 1);

  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(scan_msg.header.stamp.sec, scan_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 0.25);
  EXPECT_EQ(scan_msg.header.frame_id, "laser");
  EXPECT_NEAR(scan_msg.angle_min, (2 * M_PI * -540) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_max, (2 * M_PI * 540) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_increment, 1 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.time_increment, 1 * (270.0 / 360.0) * 0.025 / 1080.0, 1e-6);
  EXPECT_NEAR(scan_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(scan_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(scan_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)scan_msg.ranges.size(), 1081);
  bool flag_nan = false;
  for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
    if (scan_msg.ranges[i] == std::numeric_limits<float>::quiet_NaN()) {
      flag_nan = true;
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(scan_msg.intensities.empty(), true);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  // publisher test
  ep_scan = node->get_publishers_info_by_topic("scan");
  ep_multiecho = node->get_publishers_info_by_topic("echoes");
  ep_first = node->get_publishers_info_by_topic("first");
  ep_last = node->get_publishers_info_by_topic("last");
  ep_most = node->get_publishers_info_by_topic("most_intense");
  ep_diag = node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 0);
  EXPECT_EQ((int)ep_multiecho.size(), 0);
  EXPECT_EQ((int)ep_first.size(), 0);
  EXPECT_EQ((int)ep_last.size(), 0);
  EXPECT_EQ((int)ep_most.size(), 0);
  EXPECT_EQ((int)ep_diag.size(), 0);

  rclcpp::shutdown();
}

TEST(UTM_30_LX_EW, intensity_scan) {

  // initialize
  scan_msg = sensor_msgs::msg::LaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params =
  {rclcpp::Parameter("ip_address", "192.168.0.10"), rclcpp::Parameter("angle_min", -1.57),
    rclcpp::Parameter("angle_max", 1.57), rclcpp::Parameter("frame_id", "hokuyo"),
    rclcpp::Parameter("publish_intensity", true), rclcpp::Parameter("time_offset", 10.0),
    rclcpp::Parameter("skip", 1), rclcpp::Parameter("cluster", 2)};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    scan_callback);
  exe1.add_node(sub_node);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  receive_count = 0;
  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(scan_msg.header.stamp.sec, scan_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 10 + 0.25);
  EXPECT_EQ(scan_msg.header.frame_id, "hokuyo");
  EXPECT_NEAR(scan_msg.angle_min, (2 * M_PI * -360) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_max, (2 * M_PI * 360) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_increment, 2 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.time_increment, 2 * (180.0 / 360.0) * 0.025 / 720.0, 1e-6);
  EXPECT_NEAR(scan_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(scan_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(scan_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)scan_msg.ranges.size(), 361);
  bool flag_nan = false;
  for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
    if (scan_msg.ranges[i] == std::numeric_limits<float>::quiet_NaN()) {
      flag_nan = true;
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ((int)scan_msg.intensities.size(), 361);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  rclcpp::shutdown();
}

TEST(UTM_30_LX_EW, multiecho_scan) {

  // initialize
  multiecho_msg = sensor_msgs::msg::MultiEchoLaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params =
  {rclcpp::Parameter("ip_address", "192.168.0.10"), rclcpp::Parameter("angle_min", 1.57),
    rclcpp::Parameter("angle_max", -1.57), rclcpp::Parameter("publish_multiecho", true),
    rclcpp::Parameter("skip", 2), rclcpp::Parameter("cluster", 0)};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::MultiEchoLaserScan>(
    "echoes", 10,
    multiecho_callback);
  exe1.add_node(sub_node);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(
    multiecho_msg.header.stamp.sec,
    multiecho_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 0.25);
  EXPECT_EQ(multiecho_msg.header.frame_id, "laser");
  EXPECT_NEAR(multiecho_msg.angle_min, (2 * M_PI * -360) / 1440.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.angle_max, (2 * M_PI * 360) / 1440.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.angle_increment, 1 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.time_increment, 1 * (180.0 / 360.0) * 0.025 / 720.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(multiecho_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(multiecho_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)multiecho_msg.ranges.size(), 721);
  bool flag_nan = false;
  bool flag_size = false;
  for (size_t i = 0; i < multiecho_msg.ranges.size(); i++) {
    if (multiecho_msg.ranges[i].echoes.size() < 1 || 3 < multiecho_msg.ranges[i].echoes.size()) {
      flag_size = true;
    }
    for (size_t j = 0; j < multiecho_msg.ranges[i].echoes.size(); j++) {
      if (multiecho_msg.ranges[i].echoes[j] == std::numeric_limits<float>::quiet_NaN()) {
        flag_nan = true;
      }
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(flag_size, false);
  EXPECT_EQ(multiecho_msg.intensities.empty(), true);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  rclcpp::shutdown();
}

TEST(UTM_30_LX_EW, multiecho_intensity_scan) {

  // initialize
  multiecho_msg = sensor_msgs::msg::MultiEchoLaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params =
  {rclcpp::Parameter("ip_address", "192.168.0.10"), rclcpp::Parameter("frame_id", "/hokuyo"),
    rclcpp::Parameter("publish_intensity", true), rclcpp::Parameter("publish_multiecho", true),
    rclcpp::Parameter("skip", 9), rclcpp::Parameter("cluster", 99)};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::MultiEchoLaserScan>(
    "echoes", 10,
    multiecho_callback);
  auto subscriber_1 = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "first", 10,
    first_callback);
  auto subscriber_2 = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "last", 10,
    last_callback);
  auto subscriber_3 = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "most_intense", 10,
    most_callback);
  exe1.add_node(sub_node);

  // publisher test
  std::vector<rclcpp::TopicEndpointInfo> ep_scan = node->get_publishers_info_by_topic("scan");
  std::vector<rclcpp::TopicEndpointInfo> ep_multiecho =
    node->get_publishers_info_by_topic("echoes");
  std::vector<rclcpp::TopicEndpointInfo> ep_first = node->get_publishers_info_by_topic("first");
  std::vector<rclcpp::TopicEndpointInfo> ep_last = node->get_publishers_info_by_topic("last");
  std::vector<rclcpp::TopicEndpointInfo> ep_most =
    node->get_publishers_info_by_topic("most_intense");
  std::vector<rclcpp::TopicEndpointInfo> ep_diag =
    node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 0);
  EXPECT_EQ((int)ep_multiecho.size(), 0);
  EXPECT_EQ((int)ep_first.size(), 0);
  EXPECT_EQ((int)ep_last.size(), 0);
  EXPECT_EQ((int)ep_most.size(), 0);
  EXPECT_EQ((int)ep_diag.size(), 0);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // publisher test
  ep_scan = node->get_publishers_info_by_topic("scan");
  ep_multiecho = node->get_publishers_info_by_topic("echoes");
  ep_first = node->get_publishers_info_by_topic("first");
  ep_last = node->get_publishers_info_by_topic("last");
  ep_most = node->get_publishers_info_by_topic("most_intense");
  ep_diag = node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 0);
  EXPECT_EQ((int)ep_multiecho.size(), 1);
  EXPECT_EQ((int)ep_first.size(), 1);
  EXPECT_EQ((int)ep_last.size(), 1);
  EXPECT_EQ((int)ep_most.size(), 1);
  EXPECT_EQ((int)ep_diag.size(), 0);

  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  EXPECT_EQ(receive_count, 0);  // no receive
  receive_count = 0;

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  receive_count = 0;
  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(
    multiecho_msg.header.stamp.sec,
    multiecho_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 0.25);
  EXPECT_EQ(multiecho_msg.header.frame_id, "hokuyo");
  EXPECT_NEAR(multiecho_msg.angle_min, (2 * M_PI * -540) / 1440.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.angle_max, (2 * M_PI * 540) / 1440.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.angle_increment, 99 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.time_increment, 99 * (270.0 / 360.0) * 0.025 / 1080.0, 1e-6);
  EXPECT_NEAR(multiecho_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(multiecho_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(multiecho_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)multiecho_msg.ranges.size(), 11);
  bool flag_nan = false;
  bool flag_size = false;
  for (size_t i = 0; i < multiecho_msg.ranges.size(); i++) {
    if (multiecho_msg.ranges[i].echoes.size() < 1 || 3 < multiecho_msg.ranges[i].echoes.size()) {
      flag_size = true;
    }
    for (size_t j = 0; j < multiecho_msg.ranges[i].echoes.size(); j++) {
      if (multiecho_msg.ranges[i].echoes[j] == std::numeric_limits<float>::quiet_NaN()) {
        flag_nan = true;
      }
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(flag_size, false);
  EXPECT_EQ((int)multiecho_msg.intensities.size(), 11);
  flag_nan = false;
  flag_size = false;
  for (size_t i = 0; i < multiecho_msg.intensities.size(); i++) {
    if (multiecho_msg.intensities[i].echoes.size() < 1 ||
      3 < multiecho_msg.intensities[i].echoes.size())
    {
      flag_size = true;
    }
    for (size_t j = 0; j < multiecho_msg.intensities[i].echoes.size(); j++) {
      if (multiecho_msg.intensities[i].echoes[j] == std::numeric_limits<float>::quiet_NaN()) {
        flag_nan = true;
      }
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(flag_size, false);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  // publisher test
  ep_scan = node->get_publishers_info_by_topic("scan");
  ep_multiecho = node->get_publishers_info_by_topic("echoes");
  ep_first = node->get_publishers_info_by_topic("first");
  ep_last = node->get_publishers_info_by_topic("last");
  ep_most = node->get_publishers_info_by_topic("most_intense");
  ep_diag = node->get_publishers_info_by_topic("diagnostics");
  EXPECT_EQ((int)ep_scan.size(), 0);
  EXPECT_EQ((int)ep_multiecho.size(), 0);
  EXPECT_EQ((int)ep_first.size(), 0);
  EXPECT_EQ((int)ep_last.size(), 0);
  EXPECT_EQ((int)ep_most.size(), 0);
  EXPECT_EQ((int)ep_diag.size(), 0);

  rclcpp::shutdown();
}

TEST(UTM_30_LX_EW, normal_scan_param) {

  // initialize
  scan_msg = sensor_msgs::msg::LaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params =
  {rclcpp::Parameter("ip_address", "192.168.0.10"), rclcpp::Parameter("angle_min", 1.57),
    rclcpp::Parameter("angle_max", 1.57), rclcpp::Parameter("skip", 100)};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    scan_callback);
  exe1.add_node(sub_node);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  receive_count = 0;
  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(scan_msg.header.stamp.sec, scan_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 0.25);
  EXPECT_EQ(scan_msg.header.frame_id, "laser");
  EXPECT_NEAR(scan_msg.angle_min, (2 * M_PI * 359) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_max, (2 * M_PI * 360) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_increment, 1 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.time_increment, 1 * (270.0 / 360.0) * 0.025 / 1080.0, 1e-6);
  EXPECT_NEAR(scan_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(scan_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(scan_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)scan_msg.ranges.size(), 2);
  bool flag_nan = false;
  for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
    if (scan_msg.ranges[i] == std::numeric_limits<float>::quiet_NaN()) {
      flag_nan = true;
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(scan_msg.intensities.empty(), true);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  rclcpp::shutdown();
}

TEST(UTM_30_LX_EW, normal_scan_param_angle_min) {

  // initialize
  scan_msg = sensor_msgs::msg::LaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params =
  {rclcpp::Parameter("ip_address", "192.168.0.10"), rclcpp::Parameter("angle_min", 2.37),
    rclcpp::Parameter("angle_max", 2.37), rclcpp::Parameter("skip", -1)};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    scan_callback);
  exe1.add_node(sub_node);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  receive_count = 0;
  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(scan_msg.header.stamp.sec, scan_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 0.25);
  EXPECT_EQ(scan_msg.header.frame_id, "laser");
  EXPECT_NEAR(scan_msg.angle_min, (2 * M_PI * 539) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_max, (2 * M_PI * 540) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_increment, 1 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.time_increment, 1 * (270.0 / 360.0) * 0.025 / 1080.0, 1e-6);
  EXPECT_NEAR(scan_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(scan_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(scan_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)scan_msg.ranges.size(), 2);
  bool flag_nan = false;
  for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
    if (scan_msg.ranges[i] == std::numeric_limits<float>::quiet_NaN()) {
      flag_nan = true;
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(scan_msg.intensities.empty(), true);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  rclcpp::shutdown();
}

TEST(UTM_30_LX_EW, normal_scan_param_angle_max) {

  // initialize
  scan_msg = sensor_msgs::msg::LaserScan();
  scan_flag = false;
  receive_count = 0;
  scan_time = rclcpp::Time(0);

  // ros init
  rclcpp::init(0, nullptr);

  rclcpp::executors::SingleThreadedExecutor exe1;

  // urg_node2 setup
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params =
  {rclcpp::Parameter("ip_address", "192.168.0.10"), rclcpp::Parameter("angle_min", -2.37),
    rclcpp::Parameter("angle_max", -2.37), rclcpp::Parameter("cluster", -1)};
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);

  std::shared_ptr<urg_node2::UrgNode2> node = std::make_shared<urg_node2::UrgNode2>(node_options);
  exe1.add_node(node->get_node_base_interface());

  // subscriber test node setup
  auto sub_node = rclcpp::Node::make_shared("test_subscription");
  auto subscriber = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    scan_callback);
  exe1.add_node(sub_node);

  // urg_node2 transition (Uncondigured -> Inactive)
  node->configure();

  EXPECT_EQ(node->get_current_state().label(), "inactive");

  // urg_node2 transition (Inactive -> Active)
  node->activate();

  EXPECT_EQ(node->get_current_state().label(), "active");

  receive_count = 0;
  // scan wait for 10sec
  scan_wait(exe1, 10.0);

  // compare
  rclcpp::Time header_time = rclcpp::Time(scan_msg.header.stamp.sec, scan_msg.header.stamp.nanosec);
  rclcpp::Duration diff = scan_time - header_time;
  EXPECT_LE(diff.seconds(), 0.25);
  EXPECT_EQ(scan_msg.header.frame_id, "laser");
  EXPECT_NEAR(scan_msg.angle_min, (2 * M_PI * -540) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_max, (2 * M_PI * -539) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.angle_increment, 1 * (2 * M_PI * 1) / 1440.0, 1e-6);
  EXPECT_NEAR(scan_msg.time_increment, 1 * (270.0 / 360.0) * 0.025 / 1080.0, 1e-6);
  EXPECT_NEAR(scan_msg.scan_time, 0.025, 1e-6);
  EXPECT_NEAR(scan_msg.range_min, 0.023, 1e-6);
  EXPECT_NEAR(scan_msg.range_max, 60.0, 1e-6);
  EXPECT_EQ((int)scan_msg.ranges.size(), 2);
  bool flag_nan = false;
  for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
    if (scan_msg.ranges[i] == std::numeric_limits<float>::quiet_NaN()) {
      flag_nan = true;
    }
  }
  EXPECT_EQ(flag_nan, false);
  EXPECT_EQ(scan_msg.intensities.empty(), true);

  // urg_node2 transition (Active -> Finalize)
  node->shutdown();

  EXPECT_EQ(node->get_current_state().label(), "finalized");

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
