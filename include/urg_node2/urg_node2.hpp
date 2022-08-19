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

/**
 * @file urg_node2.hpp
 * @brief ROS2対応LiDARドライバ
 */

#ifndef URG_NODE2_URG_NODE2_HPP_
#define URG_NODE2_URG_NODE2_HPP_

#include <chrono>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <memory>
#include <thread>
#include <functional>
#include <limits>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "laser_proc/laser_publisher.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "urg_sensor.h"
#include "urg_utils.h"

using namespace std::chrono_literals;

/** @def
 * 受信可能なスキャンデータの最大サイズ
 */
#define URG_NODE2_MAX_DATA_SIZE 5000  // FIXME

/** @def
 * 調整モード時のシステムレイテンシ計測における通信実施回数
 */
#define URG_NODE2_CALIBRATION_MEASUREMENT_TIME 10

namespace urg_node2
{

class UrgNode2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief コンストラクタ
   * @details クラス内メンバ変数の初期化、パラメータの宣言及び"SIGPIPE"の無効化設定を行う
   */
  explicit UrgNode2(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief デストラクタ
   * @details スキャンスレッドの終了処理を行う
   */
  ~UrgNode2();

  /**
   * @brief Lifecycle制御におけるUnconfiguredからInactiveへの遷移時の処理
   * @details パラメータの取得や内部の初期化およびLiDAR接続処理、スレッドの作成を実施する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   * @retval CallbackReturn::FAILURE LiDAR未接続
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Lifecycle制御におけるInactiveからActiveへの遷移時の処理
   * @details LiDARが接続されている場合はDiagnosticsを有効化する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   * @retval CallbackReturn::ERROR LiDAR未接続
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Lifecycle制御におけるActiveからInactiveへの遷移時の処理
   * @details Diagnosticsの資源を解放し、LiDARが接続されているかの判定を行う
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   * @retval CallbackReturn::ERROR LiDAR未接続
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Lifecycle制御におけるInactiveからUnconfiguredへの遷移時の処理
   * @details スレッドの停止、publisherなどの資源の解放およびLiDAR切断処理を実施する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Lifecycle制御における各状態からFinalizedへの遷移時の処理
   * @details スレッドを停止させ、publisherとDiagnosticsの資源の解放およびLiDARとの接続を解除する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Lifecycle制御における各状態からErrorへの遷移時の処理
   * @details スレッドの停止、publisherとDiagnosticsの資源の解放およびLiDAR切断処理を実施する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  /**
   * @brief 初期化
   * @details 各種パラメータの設定、内部変数の初期化およびスキャンデータのpublisherの設定を行う
   */
  void initialize(void);

  /**
   * @brief LiDAR接続
   * @details LiDARとの接続処理とLiDAR情報の取得を行う
   * @retval true 接続成功
   * @retval false 接続失敗
   */
  bool connect(void);

  /**
   * @brief スキャン設定
   * @details 受信用のデータ領域確保やLiDARに対してスキャンの設定を行う
   */
  void set_scan_parameter(void);

  /**
   * @brief LiDAR切断
   * @details LiDARからの切断処理を行う
   */
  void disconnect(void);

  /**
   * @brief LiDAR再接続
   * @details LiDARからの切断処理および接続処理を行う
   */
  void reconnect(void);

  /**
   * @brief スキャンスレッド
   * @details LiDARからスキャンデータを受信しトピックとして配信を行う
   */
  void scan_thread(void);

  /**
   * @brief スキャントピック作成
   * @details LiDARから取得したスキャン情報のトピックへの変換を行う
   * @param[out] msg スキャンデータメッセージ
   * @retval true 正常終了
   * @retval false 取得失敗
   */
  bool create_scan_message(sensor_msgs::msg::LaserScan & msg);

  /**
   * @brief スキャントピック作成（マルチエコー）
   * @details LiDARから取得したマルチエコースキャン情報のトピックへの変換を行う
   * @param[out] msg マルチエコースキャンデータメッセージ
   * @retval true 正常終了
   * @retval false 取得失敗
   */
  bool create_scan_message(sensor_msgs::msg::MultiEchoLaserScan & msg);

  /**
   * @brief システムレイテンシの計算
   * @details 調整モード（calibrate_time_==true）時に実行、内部変数system_latency_を設定する
   * @param[in] num_measurements 計測回数
   */
  void calibrate_system_latency(size_t num_measurements);

  /**
   * @brief ROS時刻とLiDAR時刻の差の計算
   * @details ROS時刻とLiDAR時刻の差を指定回数取得し中央値を返す
   * @param[in] num_measurements 計測回数
   * @return 計測結果
   */
  rclcpp::Duration get_native_clock_offset(size_t num_measurements);

  /**
   * @brief システム時刻とLiDAR時刻の差の計算
   * @details システム時刻とLiDAR時刻の差を指定回数取得し中央値を返す
   * @param[in] num_measurements 計測回数
   * @return 計測結果
   */
  rclcpp::Duration get_time_stamp_offset(size_t num_measurements);

  /**
   * @brief タイムスタンプの動的補正
   * @details システム時刻とLiDAR時刻の指数移動平均で補正する
   * @param[in] time_stamp LiDAR時刻
   * @param[in] system_time_stamp システム時刻
   * @return 補正後タイムスタンプ
   */
  rclcpp::Time get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp);

  /**
   * @brief 強度モード対応確認
   * @details 接続先のLiDARが強度モードに対応しているかを確認する
   * @retval true 強度モード対応
   * @retval false 強度モード非対応
   */
  bool is_intensity_supported(void);

  /**
   * @brief マルチエコーモード対応確認
   * @details 接続先のLiDARがマルチエコーモードに対応しているかを確認する
   * @retval true マルチエコーモード対応
   * @retval false マルチエコーモード非対応
   */
  bool is_multiecho_supported(void);

  /**
   * @brief 開始角度位置移動までのオフセット計算
   * @details 回転体が真後ろから開始角度に移動するまでの時間を計算する
   * @return オフセット時間
   */
  rclcpp::Duration get_angular_time_offset(void);

  /**
   * @brief 診断情報の作成
   * @details Diagnosticsで出力するHardware status情報を作成を行う
   */
  void populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief スキャンスレッドの開始
   * @details LiDARとの通信を行うスキャンスレッドの開始を行う
   */
  void start_thread(void);

  /**
   * @brief スキャンスレッドの停止
   * @details LiDARとの通信を行うスキャンスレッドの停止および停止待機を行う
   */
  void stop_thread(void);

  /**
   * @brief Diagnosticsの開始
   * @details Diagnosticsの設定を行う
   */
  void start_diagnostics(void);

  /**
   * @brief Diagnosticsの停止
   * @details Diagnosticsの資源解放と登録されたパラメータの削除を行う
   */
  void stop_diagnostics(void);

  /** スキャンデータのpublisher */
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>> scan_pub_;
  /** マルチエコースキャンデータのpublisher */
  std::unique_ptr<laser_proc::LaserPublisher> echo_pub_;

  /** Diagnositcs Updater */
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  /** スキャンデータトピック診断用設定 */
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> scan_freq_;
  /** マルチエコースキャンデータトピック診断用設定 */
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> echo_freq_;

  /** スレッドの終了フラグ */
  bool close_thread_;
  /** スキャンスレッドのスレッド変数 */
  std::thread scan_thread_;

  /** LiDAR管理構造体 */
  urg_t urg_;

  /** パラメータ"ip_address" : 接続先IPアドレス */
  std::string ip_address_;
  /** パラメータ"ip_port" : 接続ポート */
  int ip_port_;
  /** パラメータ"serial_port" : 接続先シリアルデバイスパス */
  std::string serial_port_;
  /** パラメータ"serial_baud" : 接続ボーレート */
  int serial_baud_;
  /** パラメータ"frame_id" : スキャンデータのframe_id */
  std::string frame_id_;
  /** パラメータ"calibrate_time" : 調整モード */
  bool calibrate_time_;
  /** パラメータ"synchronize_time" : 同期モード */
  bool synchronize_time_;
  /** パラメータ"publish_intensity" : 強度出力モード */
  bool publish_intensity_;
  /** パラメータ"publish_multiecho" : マルチエコーモード */
  bool publish_multiecho_;
  /** パラメータ"error_limit" : 再接続を行うエラー回数 */
  int error_limit_;
  /** パラメータ"error_reset_period" : エラーをリセットする期間 */
  double error_reset_period_;
  /** パラメータ"diagnostics_tollerance" : Diagnositcs許容範囲 */
  double diagnostics_tolerance_;
  /** パラメータ"diagnostics_window_time" : Diagnositcsウィンドウタイム */
  double diagnostics_window_time_;
  /** パラメータ"time_offset" : ユーザレイテンシ[sec] */
  double time_offset_;
  /** パラメータ"angle_min" : 出力範囲の開始角度[rad] */
  double angle_min_;
  /** パラメータ"angle_max" : 出力範囲の終了角度[rad] */
  double angle_max_;
  /** パラメータ"skip" : トピック出力間引き */
  int skip_;
  /** パラメータ"cluster" : グルーピング設定 */
  int cluster_;

  /** デバイス状態 : urg_sensor_status()の値を格納 */
  std::string device_status_;
  /** センサ状態 : urg_sensor_state()の値を格納 */
  std::string sensor_status_;
  /** 製品名 : urg_sensor_product_type()の値を格納 */
  std::string product_name_;
  /** ファームウェアバージョン : urg_sensor_firmware_version()の値を格納 */
  std::string firmware_version_;
  /** デバイスID : urg_sensor_serial_id()の値を格納 */
  std::string device_id_;
  /** スキャン時間(sec) : urg_scan_usec()の値(usec)をsecに変換したものを格納 */
  double scan_period_;

  /** 計測モード [URG_DISTANCE or URG_DISTANCE_INTENSITY or URG_MULTIECHO or URG_MULTIECHO_INTENSITY] */
  urg_measurement_type_t measurement_type_;

  /** 通信エラーカウンタ（再接続時にリセット） */
  int error_count_;
  /** 通信エラー合計カウンタ（Active遷移時にリセット） */
  int total_error_count_;
  /** 再接続カウンタ（Active&Inactive時） */
  int reconnect_count_;

  /** LiDAR接続状態 */
  bool is_connected_;
  /** LiDAR計測状態 */
  bool is_measurement_started_;
  /** LiDAR安定状態 */
  bool is_stable_;
  /** 強度モードが使用できるかどうか */
  bool use_intensity_;
  /** マルチエコーモードが使用できるかどうか */
  bool use_multiecho_;
  /** Diagnosticsのtarget frequency */
  double diagnostics_freq_;

  /** トピックのframe_id設定 */
  std::string header_frame_id_;

  /** システムレイテンシ */
  rclcpp::Duration system_latency_;
  /** ユーザレイテンシ[ns] */
  rclcpp::Duration user_latency_;

  /** 同期モード用変数 */
  double hardware_clock_;
  /** 同期モード用変数 */
  long int last_hardware_time_stamp_;
  /** 同期モード用変数 */
  double hardware_clock_adj_;
  /** 同期モード用変数 */
  const double adj_alpha_ = 0.01;
  /** 同期モード用変数 */
  int adj_count_;

  /** LiDARに設定された最小ステップ範囲（範囲クリップ&丸め実施）[exp:-540] */
  int first_step_;
  /** LiDARに設定された最大ステップ範囲（範囲クリップ&丸め実施）[exp:540] */
  int last_step_;

  /** スキャンデータ受信領域 */
  std::vector<long> distance_;
  /** 強度データ受信領域 */
  std::vector<unsigned short> intensity_;

  /** トピック設定用angle_min */
  double topic_angle_min_;
  /** トピック設定用angle_max */
  double topic_angle_max_;
  /** トピック設定用angle_increment */
  double topic_angle_increment_;
  /** トピック設定用time_increment */
  double topic_time_increment_;
  /** トピック設定用range_min */
  double topic_range_min_;
  /** トピック設定用range_max */
  double topic_range_max_;
};

}
#endif  // URG_NODE2_URG_NODE2_HPP_
