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

#include "urg_node2/urg_node2.hpp"

namespace urg_node2
{

UrgNode2::UrgNode2(const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode("urg_node2", node_options),
  error_count_(0),
  is_connected_(false),
  is_measurement_started_(false),
  is_stable_(false),
  use_intensity_(false),
  use_multiecho_(false),
  system_latency_(0ns),
  user_latency_(0ns),
  first_step_(0),
  last_step_(0)
{
  // urg_open後にLiDARの電源がOFFになった状態でLiDARと通信しようとするとSIGPIPEシグナルが発生する
  // ROS1ではROSのライブラリで設定されていたがROS2では未対応のため、ここで設定する
  std::signal(SIGPIPE, SIG_IGN);

  // パラメータの登録
  ip_address_ = declare_parameter<std::string>("ip_address", "");
  ip_port_ = declare_parameter<int>("ip_port", 10940);
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  serial_baud_ = declare_parameter<int>("serial_baud", 115200);
  frame_id_ = declare_parameter<std::string>("frame_id", "laser");
  calibrate_time_ = declare_parameter<bool>("calibrate_time", false);
  synchronize_time_ = declare_parameter<bool>("synchronize_time", false);
  publish_intensity_ = declare_parameter<bool>("publish_intensity", false);
  publish_multiecho_ = declare_parameter<bool>("publish_multiecho", false);
  error_limit_ = declare_parameter<int>("error_limit", 4);
  error_reset_period_ = declare_parameter<double>("error_reset_period", 5.0),
  diagnostics_tolerance_ = declare_parameter<double>("diagnostics_tolerance", 0.05);
  diagnostics_window_time_ = declare_parameter<double>("diagnostics_window_time", 5.0);
  time_offset_ = declare_parameter<double>("time_offset", 0.0);
  angle_min_ = declare_parameter<double>("angle_min", -M_PI);
  angle_max_ = declare_parameter<double>("angle_max", M_PI);
  skip_ = declare_parameter<int>("skip", 0);
  cluster_ = declare_parameter<int>("cluster", 1);
}

// デストラクタ
UrgNode2::~UrgNode2()
{
  // スレッドの停止
  stop_thread();
}

// onConfigure
UrgNode2::CallbackReturn UrgNode2::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "transition Configuring from %s", state.label().c_str());

  initialize();

  if (!connect()) {
    return CallbackReturn::FAILURE;
  }

  // Publisher設定
  if (use_multiecho_) {
    echo_pub_ = std::make_unique<laser_proc::LaserPublisher>(get_node_topics_interface(), 20);
  } else {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(20));
  }

  // スレッド起動
  start_thread();

  return CallbackReturn::SUCCESS;
}

// onActivate
UrgNode2::CallbackReturn UrgNode2::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "transition Activating from %s", state.label().c_str());

  if (!is_connected_) {
    return CallbackReturn::ERROR;
  } else {
    // publisherの有効化
    if (scan_pub_) {
      scan_pub_->on_activate();
    }

    // Diagnostics開始
    start_diagnostics();

    // 累計エラーカウントの初期化
    total_error_count_ = 0;

    return CallbackReturn::SUCCESS;
  }
}

// onDeactivate
UrgNode2::CallbackReturn UrgNode2::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "transition Deactivating from %s", state.label().c_str());

  // Diagnostics停止
  stop_diagnostics();

  if (!is_connected_) {
    return CallbackReturn::ERROR;
  } else {
    return CallbackReturn::SUCCESS;
  }
}

// onCleanup
UrgNode2::CallbackReturn UrgNode2::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "transition CleaningUp from %s", state.label().c_str());

  // スレッドの停止
  stop_thread();

  // publisherの解放
  if (use_multiecho_) {
    echo_pub_.reset();
  } else {
    scan_pub_.reset();
  }

  // 切断
  disconnect();

  return CallbackReturn::SUCCESS;
}

// onShutdown
UrgNode2::CallbackReturn UrgNode2::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "transition Shutdown from %s", state.label().c_str());

  // スレッドの停止
  stop_thread();

  // Diagnostics停止
  stop_diagnostics();

  // publisherの解放
  if (use_multiecho_) {
    echo_pub_.reset();
  } else {
    scan_pub_.reset();
  }

  // 切断
  disconnect();

  return CallbackReturn::SUCCESS;
}

// onError
UrgNode2::CallbackReturn UrgNode2::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "transition Error from %s", state.label().c_str());

  // スレッドの停止
  stop_thread();

  // Diagnostics停止
  stop_diagnostics();

  // publisherの解放
  if (use_multiecho_) {
    echo_pub_.reset();
  } else {
    scan_pub_.reset();
  }

  // 切断
  disconnect();

  return CallbackReturn::SUCCESS;
}

// 初期化
void UrgNode2::initialize()
{
  // パラメータ取得
  ip_address_ = get_parameter("ip_address").as_string();
  ip_port_ = get_parameter("ip_port").as_int();
  serial_port_ = get_parameter("serial_port").as_string();
  serial_baud_ = get_parameter("serial_baud").as_int();
  frame_id_ = get_parameter("frame_id").as_string();
  calibrate_time_ = get_parameter("calibrate_time").as_bool();
  synchronize_time_ = get_parameter("synchronize_time").as_bool();
  publish_intensity_ = get_parameter("publish_intensity").as_bool();
  publish_multiecho_ = get_parameter("publish_multiecho").as_bool();
  error_limit_ = get_parameter("error_limit").as_int();
  error_reset_period_ = get_parameter("error_reset_period").as_double();
  diagnostics_tolerance_ = get_parameter("diagnostics_tolerance").as_double();
  diagnostics_window_time_ = get_parameter("diagnostics_window_time").as_double();
  time_offset_ = get_parameter("time_offset").as_double();
  angle_min_ = get_parameter("angle_min").as_double();
  angle_max_ = get_parameter("angle_max").as_double();
  skip_ = get_parameter("skip").as_int();
  cluster_ = get_parameter("cluster").as_int();

  // 範囲チェック
  angle_min_ = (angle_min_ < -M_PI) ? -M_PI : ((angle_min_ > M_PI) ? M_PI : angle_min_);
  angle_max_ = (angle_max_ < -M_PI) ? -M_PI : ((angle_max_ > M_PI) ? M_PI : angle_max_);
  skip_ = (skip_ < 0) ? 0 : ((skip_ > 9) ? 9 : skip_);
  cluster_ = (cluster_ < 1) ? 1 : ((cluster_ > 99) ? 99 : cluster_);

  // 内部変数初期化
  is_connected_ = false;
  is_measurement_started_ = false;
  is_stable_ = false;
  user_latency_ = rclcpp::Duration::from_seconds(time_offset_);

  // メッセージヘッダのframe_id設定
  header_frame_id_ =
    (frame_id_.find_first_not_of('/') == std::string::npos) ? "" : frame_id_.substr(
    frame_id_.find_first_not_of(
      '/'));

  hardware_clock_ = 0.0;
  last_hardware_time_stamp_ = 0;
  hardware_clock_adj_ = 0;
  adj_count_ = 0;
}

// Lidarとの接続処理
bool UrgNode2::connect()
{
  if (!ip_address_.empty()) {
    // イーサネット接続
    int result = urg_open(&urg_, URG_ETHERNET, ip_address_.c_str(), ip_port_);
    if (result < 0) {
      RCLCPP_ERROR(
        get_logger(), "Could not open network Hokuyo 2D LiDAR\n%s:%d\n%s",
        ip_address_.c_str(), ip_port_, urg_error(&urg_));
      return false;
    }
  } else {
    // シリアル接続
    int result = urg_open(&urg_, URG_SERIAL, serial_port_.c_str(), serial_baud_);
    if (result < 0) {
      RCLCPP_ERROR(
        get_logger(), "Could not open serial Hokuyo 2D LiDAR\n%s:%d\n%s",
        serial_port_.c_str(), serial_baud_, urg_error(&urg_));
      return false;
    }
  }

  is_connected_ = true;

  std::stringstream ss;
  ss << "Connected to a ";
  if (!ip_address_.empty()) {
    ss << "network ";
  } else {
    ss << "serial ";
  }
  ss << "device with ";
  if (publish_multiecho_) {
    ss << "multiecho ";
  } else {
    ss << "single ";
  }
  if (publish_intensity_) {
    ss << "and intensity ";
  }
  ss << "scan. Hardware ID: " << urg_sensor_serial_id(&urg_);
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

  // LiDAR情報格納
  device_status_ = urg_sensor_status(&urg_);
  sensor_status_ = urg_sensor_state(&urg_);
  product_name_ = urg_sensor_product_type(&urg_);
  firmware_version_ = urg_sensor_firmware_version(&urg_);
  device_id_ = urg_sensor_serial_id(&urg_);
  scan_period_ = 1.e-6 * static_cast<double>(urg_scan_usec(&urg_));

  // 接続先のLiDARが強度出力に対応しているか
  if (publish_intensity_) {
    use_intensity_ = is_intensity_supported();
    if (!use_intensity_) {
      RCLCPP_WARN(
        get_logger(),
        "parameter 'publish_intensity' is true, but this device does not support intensity mode. disable intensity mode.");
    }
  }
  // 接続先のLiDARがマルチエコー出力に対応しているか
  if (publish_multiecho_) {
    use_multiecho_ = is_multiecho_supported();
    if (!use_multiecho_) {
      RCLCPP_WARN(
        get_logger(),
        "parameter 'publish_multiecho' is true, but this device does not support multiecho scan mode. switch single scan mode.");
    }
  }

  // 計測タイプ設定
  if (use_intensity_ && use_multiecho_) {
    measurement_type_ = URG_MULTIECHO_INTENSITY;
  } else if (use_intensity_) {
    measurement_type_ = URG_DISTANCE_INTENSITY;
  } else if (use_multiecho_) {
    measurement_type_ = URG_MULTIECHO;
  } else {
    measurement_type_ = URG_DISTANCE;
  }

  return true;
}

// スキャン設定
void UrgNode2::set_scan_parameter()
{
  // スキャンデータ受信領域の確保（AMAX+1が返る）
  int urg_data_size = urg_max_data_size(&urg_);
  // AMAXは1440が最大と想定されているが一応制限（仮実装）をかける
  if (urg_data_size > URG_NODE2_MAX_DATA_SIZE) {
    urg_data_size = URG_NODE2_MAX_DATA_SIZE;
  }

  // マルチエコー分の領域を確保(URG_MAX_ECHO = 3)
  distance_.resize(urg_data_size * URG_MAX_ECHO);
  intensity_.resize(urg_data_size * URG_MAX_ECHO);

  // 指定範囲のステップ変換（範囲クリップ&丸め）
  first_step_ = urg_rad2step(&urg_, angle_min_);
  last_step_ = urg_rad2step(&urg_, angle_max_);

  // 逆転してた場合入れ替え
  if (last_step_ < first_step_) {
    std::swap(first_step_, last_step_);
  }

  // 変換後ステップが同値の場合は1ずらす
  if (last_step_ == first_step_) {
    int min_step, max_step;
    urg_step_min_max(&urg_, &min_step, &max_step);
    if (first_step_ == min_step) {
      last_step_ = first_step_ + 1;
    } else {
      first_step_ = last_step_ - 1;
    }
  }

  // スキャンデータ範囲の反映
  angle_min_ = urg_step2rad(&urg_, first_step_);
  angle_max_ = urg_step2rad(&urg_, last_step_);

  // スキャンデータ範囲の設定
  urg_set_scanning_parameter(&urg_, first_step_, last_step_, cluster_);

  // スキャントピック用のパラメータ設定
  topic_angle_min_ = urg_step2rad(&urg_, first_step_);
  topic_angle_max_ = urg_step2rad(&urg_, last_step_);
  topic_angle_increment_ = cluster_ * urg_step2rad(&urg_, 1);

  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  topic_time_increment_ = cluster_ *
    ((urg_step2rad(
      &urg_,
      max_step) -
    urg_step2rad(
      &urg_,
      min_step)) / (2.0 * M_PI)) * scan_period_ / static_cast<double>(max_step - min_step);

  long min_dis;
  long max_dis;
  urg_distance_min_max(&urg_, &min_dis, &max_dis);
  topic_range_min_ = static_cast<double>(min_dis) / 1000.0;
  topic_range_max_ = static_cast<double>(max_dis) / 1000.0;
}

// Lidarとの切断処理
void UrgNode2::disconnect()
{
  if (is_connected_) {
    urg_close(&urg_);
    is_connected_ = false;
  }
}

// Lidarとの再接続処理
void UrgNode2::reconnect()
{
  // 切断
  disconnect();
  // 接続
  connect();
}

// scanスレッド
void UrgNode2::scan_thread()
{
  reconnect_count_ = 0;

  while (!close_thread_) {
    if (!is_connected_) {
      if (!connect()) {
        rclcpp::sleep_for(500ms);
        continue;
      }
    }

    // Inactive状態判定
    rclcpp_lifecycle::State state = get_current_state();
    if (state.label() == "inactive") {
      is_stable_ = urg_is_stable(&urg_);
      if (!is_stable_) {
        // 再接続処理
        reconnect();
        reconnect_count_++;
      }
      rclcpp::sleep_for(100ms);
      continue;
    }

    // スキャン設定
    set_scan_parameter();

    // 調整モード
    if (calibrate_time_) {
      calibrate_system_latency(URG_NODE2_CALIBRATION_MEASUREMENT_TIME);
    }

    // LiDAR状態更新
    device_status_ = urg_sensor_status(&urg_);
    sensor_status_ = urg_sensor_state(&urg_);
    is_stable_ = urg_is_stable(&urg_);

    // 計測開始
    int ret = urg_start_measurement(&urg_, measurement_type_, 0, skip_, 0);
    if (ret < 0) {
      RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n%s", urg_error(&urg_));

      // 再接続処理
      reconnect();
      reconnect_count_++;

      continue;
    }

    is_measurement_started_ = true;
    error_count_ = 0;

    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time prev_time = system_clock.now();

    while (!close_thread_) {
      // Inactive状態判定
      rclcpp_lifecycle::State state = get_current_state();
      if (state.label() == "inactive") {
        urg_stop_measurement(&urg_);
        is_measurement_started_ = false;
        break;
      }

      if (use_multiecho_) {
        sensor_msgs::msg::MultiEchoLaserScan msg;
        if (create_scan_message(msg)) {
          echo_pub_->publish(msg);
          if (echo_freq_) {
            echo_freq_->tick();
          }
        } else {
          RCLCPP_WARN(get_logger(), "Could not get multi echo scan.");
          error_count_++;
          total_error_count_++;
          device_status_ = urg_sensor_status(&urg_);
          sensor_status_ = urg_sensor_state(&urg_);
          is_stable_ = urg_is_stable(&urg_);
        }
      } else {
        sensor_msgs::msg::LaserScan msg;
        if (create_scan_message(msg)) {
          scan_pub_->publish(msg);
          if (scan_freq_) {
            scan_freq_->tick();
          }
        } else {
          RCLCPP_WARN(get_logger(), "Could not get single echo scan.");
          error_count_++;
          total_error_count_++;
          device_status_ = urg_sensor_status(&urg_);
          sensor_status_ = urg_sensor_state(&urg_);
          is_stable_ = urg_is_stable(&urg_);
        }
      }

      // エラーカウント判定
      if (error_count_ > error_limit_) {
        RCLCPP_ERROR(get_logger(), "Error count exceeded limit, reconnecting.");
        // 再接続処理
        is_measurement_started_ = false;
        reconnect();
        reconnect_count_++;
        break;
      } else {
        // エラーカウントのリセット
        rclcpp::Time current_time = system_clock.now();
        rclcpp::Duration period = current_time - prev_time;
        if (period.seconds() >= error_reset_period_) {
          prev_time = current_time;
          error_count_ = 0;
        }
      }
    }
  }

  // 切断処理
  disconnect();
}

// スキャンデータ取得
bool UrgNode2::create_scan_message(sensor_msgs::msg::LaserScan & msg)
{
  msg.header.frame_id = header_frame_id_;
  msg.angle_min = topic_angle_min_;
  msg.angle_max = topic_angle_max_;
  msg.angle_increment = topic_angle_increment_;
  msg.scan_time = scan_period_;
  msg.time_increment = topic_time_increment_;
  msg.range_min = topic_range_min_;
  msg.range_max = topic_range_max_;

  int num_beams = 0;
  long time_stamp = 0;
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Time system_time_stamp = system_clock.now();

  if (use_intensity_) {
    num_beams = urg_get_distance_intensity(&urg_, &distance_[0], &intensity_[0], &time_stamp);
  } else {
    num_beams = urg_get_distance(&urg_, &distance_[0], &time_stamp);
  }
  if (num_beams <= 0) {
    return false;
  }

  // タイムスタンプ設定
  if (synchronize_time_) {
    system_time_stamp = get_synchronized_time(time_stamp, system_time_stamp);
  }
  msg.header.stamp = system_time_stamp + system_latency_ + user_latency_ +
    get_angular_time_offset();

  // データ領域確保
  msg.ranges.resize(num_beams);
  if (use_intensity_) {
    msg.intensities.resize(num_beams);
  }

  for (int i = 0; i < num_beams; i++) {
    if (distance_[i] != 0) {
      msg.ranges[i] = static_cast<float>(distance_[i]) / 1000.0;
      if (use_intensity_) {
        msg.intensities[i] = intensity_[i];
      }
    } else {
      msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }

  return true;
}

// マルチエコースキャンデータ取得
bool UrgNode2::create_scan_message(sensor_msgs::msg::MultiEchoLaserScan & msg)
{
  msg.header.frame_id = header_frame_id_;
  msg.angle_min = topic_angle_min_;
  msg.angle_max = topic_angle_max_;
  msg.angle_increment = topic_angle_increment_;
  msg.scan_time = scan_period_;
  msg.time_increment = topic_time_increment_;
  msg.range_min = topic_range_min_;
  msg.range_max = topic_range_max_;

  int num_beams = 0;
  long time_stamp = 0;
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Time system_time_stamp = system_clock.now();
  if (use_intensity_) {
    num_beams = urg_get_multiecho_intensity(&urg_, &distance_[0], &intensity_[0], &time_stamp);
  } else {
    num_beams = urg_get_multiecho(&urg_, &distance_[0], &time_stamp);
  }
  if (num_beams <= 0) {
    return false;
  }

  // タイムスタンプ設定
  if (synchronize_time_) {
    system_time_stamp = get_synchronized_time(time_stamp, system_time_stamp);
  }
  msg.header.stamp = system_time_stamp + system_latency_ + user_latency_ +
    get_angular_time_offset();

  // データ領域確保
  msg.ranges.reserve(num_beams);
  if (use_intensity_) {
    msg.intensities.reserve(num_beams);
  }

  for (int i = 0; i < num_beams; i++) {
    sensor_msgs::msg::LaserEcho distance_echo;
    distance_echo.echoes.reserve(URG_MAX_ECHO);
    sensor_msgs::msg::LaserEcho intensity_echo;
    if (use_intensity_) {
      intensity_echo.echoes.reserve(URG_MAX_ECHO);
    }
    for (int j = 0; j < URG_MAX_ECHO; j++) {
      if (distance_[(URG_MAX_ECHO * i) + j] != 0) {
        distance_echo.echoes.push_back(
          static_cast<float>(distance_[(URG_MAX_ECHO * i) + j]) / 1000.0f);
        if (use_intensity_) {
          intensity_echo.echoes.push_back(intensity_[(URG_MAX_ECHO * i) + j]);
        }
      } else {
        break;
      }
    }
    msg.ranges.push_back(distance_echo);
    if (use_intensity_) {
      msg.intensities.push_back(intensity_echo);
    }
  }

  return true;
}

// 診断情報入力
void UrgNode2::populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  if (!is_connected_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not Connected");
    return;
  }

  if (!ip_address_.empty()) {
    status.add("IP Address", ip_address_);
    status.add("IP Port", ip_port_);
  } else {
    status.add("Serial Port", serial_port_);
    status.add("Serial Baud", serial_baud_);
  }

  if (!is_measurement_started_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Not started: " + device_status_);
  } else if (publish_intensity_ && !use_intensity_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Intensity mode has been set, but not working in intensity mode");
  } else if (publish_multiecho_ && !use_multiecho_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Multiecho mode has been set, but not working in multiecho mode");
  } else if (!is_stable_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Abnormal status: " + device_status_);
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
  }

  status.add("Product Name", product_name_);
  status.add("Firmware Version", firmware_version_);
  status.add("Device ID", device_id_);
  status.add("Computed Latency", system_latency_.seconds());
  status.add("User Time Offset", user_latency_.seconds());
  status.add("Device Status", device_status_);
  status.add("Sensor Status", sensor_status_);
  status.add("Scan Retrieve Error Count", error_count_);
  status.add("Scan Retrieve Total Error Count", total_error_count_);
  status.add("Reconnection Count", reconnect_count_);
}

// スキャンスレッドの開始
void UrgNode2::start_thread(void)
{
  // スレッド終了フラグのクリア
  close_thread_ = false;
  scan_thread_ = std::thread(std::bind(&UrgNode2::scan_thread, this));
}

// スキャンスレッドの停止
void UrgNode2::stop_thread(void)
{
  // スレッド終了フラグのセット
  close_thread_ = true;
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }
}

// Diagnosticsの開始
void UrgNode2::start_diagnostics(void)
{
  // Diagnostics設定
  diagnostic_updater_.reset(new diagnostic_updater::Updater(this));
  diagnostic_updater_->setHardwareID(device_id_);
  diagnostic_updater_->add("Hardware Status", this, &UrgNode2::populate_diagnostics_status);

  // Diagnosticsトピック設定
  diagnostics_freq_ = 1.0 / (scan_period_ * (skip_ + 1));
  if (use_multiecho_) {
    echo_freq_.reset(
      new diagnostic_updater::HeaderlessTopicDiagnostic(
        "Laser Echoes",
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(
          &diagnostics_freq_, &diagnostics_freq_, diagnostics_tolerance_,
          diagnostics_window_time_)));
  } else {
    scan_freq_.reset(
      new diagnostic_updater::HeaderlessTopicDiagnostic(
        "Laser Scan",
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(
          &diagnostics_freq_, &diagnostics_freq_, diagnostics_tolerance_,
          diagnostics_window_time_)));
  }
}

// Diagnosticsの停止
void UrgNode2::stop_diagnostics(void)
{
  // Diagnostics解放
  diagnostic_updater_.reset();
  if (use_multiecho_) {
    echo_freq_.reset();
  } else {
    scan_freq_.reset();
  }

  // 暫定対応
  // Diagnosticsが追加したパラメータの解放
  // diagnostic_updaterを再び生成するとdeclare_parameterが呼ばれエラーになる
  //   https://github.com/ros/diagnostics/pull/227
  if (has_parameter("diagnostic_updater.period")) {
    try {
      undeclare_parameter("diagnostic_updater.period");
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(get_logger(), "undeclare_parameter failed: %s", e.what());
    }
  }
}

// 接続先LiDARが強度出力に対応しているかどうか
bool UrgNode2::is_intensity_supported(void)
{
  // 計測は停止している必要がある
  if (is_measurement_started_) {
    return false;
  }

  if (urg_start_measurement(&urg_, URG_DISTANCE_INTENSITY, 0, 0, 0) < 0) {
    RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n%s", urg_error(&urg_));
    return false;
  }
  is_measurement_started_ = true;
  int ret = urg_get_distance_intensity(&urg_, &distance_[0], &intensity_[0], NULL);
  if (ret <= 0) {
    // 強度出力非対応
    return false;
  }

  urg_stop_measurement(&urg_);
  is_measurement_started_ = false;
  return true;
}

// 接続先LiDARがマルチエコー出力に対応しているかどうか
bool UrgNode2::is_multiecho_supported(void)
{
  // 計測は停止している必要がある
  if (is_measurement_started_) {
    return false;
  }

  if (urg_start_measurement(&urg_, URG_MULTIECHO_INTENSITY, 0, 0, 0) < 0) {
    RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n%s", urg_error(&urg_));
    return false;
  }
  is_measurement_started_ = true;
  int ret = urg_get_multiecho(&urg_, &distance_[0], NULL);
  if (ret <= 0) {
    // マルチエコー出力非対応
    return false;
  }

  urg_stop_measurement(&urg_);
  is_measurement_started_ = false;
  return true;
}

// システムレイテンシの計測
void UrgNode2::calibrate_system_latency(size_t num_measurements)
{
  if (!is_connected_) {
    RCLCPP_WARN(get_logger(), "Unable to calibrate time offset. Not Ready.");
    return;
  }

  try {
    RCLCPP_INFO(get_logger(), "Starting calibration. This will take a few seconds.");
    RCLCPP_INFO(get_logger(), "Time calibration is still experimental.");

    system_latency_ = rclcpp::Duration(0ns);

    rclcpp::Duration start_offset = get_native_clock_offset(1);
    rclcpp::Duration prev_offset(0ns);

    std::vector<rclcpp::Duration> time_offsets;
    for (size_t i = 0; i < num_measurements; i++) {
      rclcpp::Duration scan_offset = get_time_stamp_offset(1);
      rclcpp::Duration post_offset = get_native_clock_offset(1);
      rclcpp::Duration adjusted_scan_offset = scan_offset - start_offset;
      rclcpp::Duration adjusted_post_offset = post_offset - start_offset;
      rclcpp::Duration average_offset = rclcpp::Duration::from_seconds(
        (adjusted_post_offset.seconds() + prev_offset.seconds()) / 2.0);
      time_offsets.push_back(adjusted_scan_offset - average_offset);
      prev_offset = adjusted_post_offset;
    }

    // 格納した差をソートし中央値を返す
    std::nth_element(
      time_offsets.begin(),
      time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
    system_latency_ = time_offsets[time_offsets.size() / 2];

    // 開始角度までの旋回時間を加算
    system_latency_ = system_latency_ + get_angular_time_offset();

    RCLCPP_INFO(
      get_logger(), "Calibration finished. Latency is: %.4f sec.",
      (double)(system_latency_.nanoseconds() * 1e-9));

  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(get_logger(), "Could not calibrate time offset: %s", e.what());
    system_latency_ = rclcpp::Duration(0ns);
  }
}

// ROS時刻とLiDAR時刻の差の計算
rclcpp::Duration UrgNode2::get_native_clock_offset(size_t num_measurements)
{
  // すでに計測開始されていた場合エラー
  if (is_measurement_started_) {
    std::stringstream ss;
    ss << "Cannot get native clock offset while started.";
    throw std::runtime_error(ss.str());
  }

  if (urg_start_time_stamp_mode(&urg_) < 0) {
    std::stringstream ss;
    ss << "Cannot start time stamp mode.";
    throw std::runtime_error(ss.str());
  }

  std::vector<rclcpp::Duration> time_offsets;
  for (size_t i = 0; i < num_measurements; i++) {
    // chronoライブラリでLiDAR時刻取得直前の時刻を取得
    rclcpp::Time request_time(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    // LiDARから時刻を取得
    double urg_timestamp = urg_time_stamp(&urg_);
    rclcpp::Time lidar_time(1e6 * urg_timestamp);
    // LiDAR時刻取得後の時刻を取得
    rclcpp::Time response_time(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    // 直前時刻と直後時刻の平均値を取得時刻とみなす
    rclcpp::Time average_time((response_time.nanoseconds() + request_time.nanoseconds()) / 2.0);
    // chronoライブラリで取得した時刻とLiDAR時刻との差を格納
    time_offsets.push_back(lidar_time - average_time);
  }

  if (urg_stop_time_stamp_mode(&urg_) < 0) {
    std::stringstream ss;
    ss << "Cannot stop time stamp mode.";
    throw std::runtime_error(ss.str());
  }

  // 格納した差をソートし中央値を返す
  std::nth_element(
    time_offsets.begin(),
    time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
  return time_offsets[time_offsets.size() / 2];
}

// システム時刻とLiDAR時刻の差の計算
rclcpp::Duration UrgNode2::get_time_stamp_offset(size_t num_measurements)
{
  // すでに計測開始されていた場合"0"を返す
  if (is_measurement_started_) {
    std::stringstream ss;
    ss << "Cannot get time stamp offset while started.";
    throw std::runtime_error(ss.str());
  }

  // 計測開始
  int ret = urg_start_measurement(&urg_, measurement_type_, 0, skip_, 0);
  if (ret < 0) {
    std::stringstream ss;
    ss << "Could not start Hokuyo measurement.\n";
    ss << urg_error(&urg_);
    throw std::runtime_error(ss.str());
  }
  is_measurement_started_ = true;

  std::vector<rclcpp::Duration> time_offsets;
  for (size_t i = 0; i < num_measurements; i++) {
    long time_stamp;
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    int ret = 0;
    long system_time_stamp = system_clock.now().nanoseconds();

    // データ取得時のシステム時刻とLiDAR時刻を取得
    if (measurement_type_ == URG_DISTANCE) {
      ret = urg_get_distance(&urg_, &distance_[0], &time_stamp);
    } else if (measurement_type_ == URG_DISTANCE_INTENSITY) {
      ret = urg_get_distance_intensity(
        &urg_, &distance_[0], &intensity_[0], &time_stamp);
    } else if (measurement_type_ == URG_MULTIECHO) {
      ret = urg_get_multiecho(&urg_, &distance_[0], &time_stamp);
    } else if (measurement_type_ == URG_MULTIECHO_INTENSITY) {
      ret = urg_get_multiecho_intensity(
        &urg_, &distance_[0], &intensity_[0], &time_stamp);
    }

    if (ret <= 0) {
      std::stringstream ss;
      ss << "Cannot get scan to measure time stamp offset.";
      throw std::runtime_error(ss.str());
    }

    rclcpp::Time lidar_timestamp(1e6 * time_stamp);
    rclcpp::Time system_timestamp(system_time_stamp);
    time_offsets.push_back(lidar_timestamp - system_timestamp);
  }

  // 計測停止
  urg_stop_measurement(&urg_);
  is_measurement_started_ = false;

  // 格納した差をソートし中央値を返す
  std::nth_element(
    time_offsets.begin(),
    time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
  return time_offsets[time_offsets.size() / 2];
}

// 指数移動平均による動的補正
rclcpp::Time UrgNode2::get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp)
{
  rclcpp::Time stamp = system_time_stamp;

  const uint32_t t1 = static_cast<uint32_t>(time_stamp);
  const uint32_t t0 = static_cast<uint32_t>(last_hardware_time_stamp_);
  const uint32_t mask = 0x00ffffff;
  double delta = static_cast<double>(mask & (t1 - t0)) / 1000.0;
  hardware_clock_ += delta;
  double cur_adj = stamp.seconds() - hardware_clock_;
  if (adj_count_ > 0) {
    hardware_clock_adj_ = adj_alpha_ * cur_adj + (1.0 - adj_alpha_) * hardware_clock_adj_;
  } else {
    // 指数移動平均の初期化
    hardware_clock_adj_ = cur_adj;
  }
  adj_count_++;
  last_hardware_time_stamp_ = time_stamp;

  // ズレのチェック
  if (adj_count_ > 100) {
    stamp = rclcpp::Time((hardware_clock_ + hardware_clock_adj_) * 1e9);
    // ズレが大きすぎると指数移動平均の初期化
    if (fabs((stamp - system_time_stamp).seconds()) > 0.1) {
      adj_count_ = 0;
      hardware_clock_ = 0.0;
      last_hardware_time_stamp_ = 0;
      stamp = system_time_stamp;
      RCLCPP_WARN(get_logger(), "%s: detected clock warp, reset EMA", __func__);
    }
  }
  return stamp;
}

// 開始角度位置移動までのオフセット計算
rclcpp::Duration UrgNode2::get_angular_time_offset(void)
{
  // スキャンデータのタイムスタンプは真後ろのものなので開始角度位置までの時間をオフセットとして加算する必要がある
  double circle_fraction = 0.0;
  if (first_step_ == 0 && last_step_ == 0) {
    int min_step, max_step;
    urg_step_min_max(&urg_, &min_step, &max_step);
    circle_fraction = (urg_step2rad(&urg_, min_step) + M_PI) / (2.0 * M_PI);
  } else {
    circle_fraction = (urg_step2rad(&urg_, first_step_) + M_PI) / (2.0 * M_PI);
  }
  return rclcpp::Duration::from_seconds(circle_fraction * scan_period_);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node2::UrgNode2)
