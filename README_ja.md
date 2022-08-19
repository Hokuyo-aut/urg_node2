# 概要
ROS2で動作する北陽電機製 2D測域センサ(LiDAR)のドライバです。

- 特徴
  - イーサネットおよびUSB接続に対応
  - スキャンデータ出力
  - マルチエコースキャンデータ出力（トピックの配信には`laser_proc`パッケージ使用）
  - ハードウェア診断情報の出力（診断情報の配信には`diagnostic_updater`パッケージ使用）
  - コンポーネント実装
  - [ライフサイクル制御](http://design.ros2.org/articles/node_lifecycle.html)対応

インタフェースや一部の処理は、ROS1でよく利用されている[urg_nodeパッケージ](http://wiki.ros.org/urg_node)を参考にしています。

LiDARとの通信には[urg_library ver.1.2.5](https://github.com/UrgNetworks/urg_library/tree/ver.1.2.5)のC言語APIを使用します。

## ライフサイクル制御
ライフサイクルの各状態での動作は以下のようになります。
- Unconfigured  
  起動状態
- Inactive  
  LiDAR接続状態（スキャンデータと診断情報は配信されません）
- Active  
  LiDARデータ配信状態（スキャンデータと診断情報が配信されます）
- Finalized  
  終了状態

# 対応機種
北陽電機製のSCIP 2.2に対応した測域センサ  
動作確認済み機種：`UTM-30LX-EW`, `UST-10LX`, `UTM-30LX`, `URG-04LX-UG01`

動作確認済み環境：`foxy`, `galactic`

# ライセンス
本パッケージのライセンスは`Apache License 2.0`です。  
urg_libray C言語APIのライセンスは`Simplified BSD License`です。

# Publish
- /scan (sensor_msgs::msg::LaserScan)  
  LiDARスキャンデータ（パラメータ`publish_multiecho`=false時出力）
- /echoes (sensor_msgs::msg::MultiEchoLaserScan)  
  LiDARマルチエコースキャンデータ（パラメータ`publish_multiecho`=true時出力）
- /first (sensor_msgs::msg::LaserScan)  
  最も近いスキャンデータ（パラメータ`publish_multiecho`=true時出力）
- /last (sensor_msgs::msg::LaserScan)  
  最も遠いスキャンデータ（パラメータ`publish_multiecho`=true時出力）
- /most_intense (sensor_msgs::msg::LaserScan)  
  最も強度が高いスキャンデータ（パラメータ`publish_multiecho`=true時出力）
- /diagnostics (diagnostics_msgs::msg::DiagnosticArray)  
  診断情報

# パラメータ
- ip_address (string, default: "")  
  LiDARにイーサネット接続する場合のIPアドレス（"XX.XX.XX.XX"の形式で指定します）  
  ※指定する文字列が空（empty）の場合はシリアル接続になります。
- ip_port (int, default: 10940)  
  LiDARにイーサネット接続する場合のポート番号
- serial_port (string, default: "/dev/ttyACM0")  
  接続先シリアルデバイスパス
- serial_baud (int, default: 115200)  
  接続シリアルボーレート
- frame_id (string, default: "laser")  
  スキャンデータのframe_id  
  スキャンデータのメッセージヘッダ"frame_id"には、パラメータ`frame_id`が設定されます。  
- calibrate_time (bool, default: false)  
  調整モードのフラグ  
  このフラグがtrueの場合、スキャン開始時にLiDARの時刻とシステムの時刻のズレを計測しレイテンシとしてスキャンデータのtimestampに加算します。
- synchronize_time (bool, default: false)  
  同期モードのフラグ  
  このフラグがtrueの場合、スキャンデータのtimestampの基準となるシステム時刻についてLiDARの時刻とのズレを用いて動的補正します。
- publish_intensity (bool, default: false)  
  強度出力モードフラグ  
  このフラグがtrueの場合、スキャンデータの強度データ（intensities）が出力され、falseの場合、強度データは空（empty）で出力されます。  
  LiDARが強度出力に対応していない場合は、false設定と同様に動作します。
- publish_multiecho (bool, default: false)  
  マルチエコーモードフラグ  
  このフラグがtrueの場合、マルチエコースキャンデータ（/echoes, /first, /last, /most_intense）が出力され、falseの場合、スキャンデータ（/scan）が出力されます。  
  LiDARがマルチエコーに対応していない場合は、false設定と同様に動作します。
- error_limit (int, default: 4 [回])  
  再接続を実施するエラー回数  
  データ取得の際に発生したエラー回数がerror_limitより大きくなった場合にLiDARとの接続を再接続します。  
- error_reset_period (double, default: 5.0 [sec])  
  エラーのリセット周期  
  データ取得の際に発生したエラー回数を周期的にリセットします。  
  ※散発的なエラー発生における再接続を防止するためです。
- diagnostics_tolerance (double, default: 0.05)  
  スキャンデータの配信頻度に関する診断情報（Diagnostics）の目標配信頻度に対しての許容割合  
  ※例えばdiagnostics_toleranceが0.05であれば、目標配信頻度の105%～95%の範囲が正常範囲となります。
- diagnostics_window_time (double, default: 5.0 [sec])  
  スキャンデータの配信頻度に関する診断情報（Diagnostics）の出力配信頻度を計測する期間
- time_offset (double, default: 0.0 [sec])  
  スキャンデータのtimestampに加算するユーザレイテンシ
- angle_min (double, default：-pi [rad], 範囲：-pi～pi)  
  スキャンデータの出力範囲の最小角度
- angle_max (double, default：pi [rad], 範囲：-pi～pi)  
  スキャンデータの出力範囲の最大角度
- skip (int, default: 0 [回], 範囲: 0～9)  
  スキャンデータの出力間引き設定  
  スキャンデータの配信頻度が1/(skip+1)倍になります。
- cluster (int, default: 1 [個], 範囲: 1～99)  
  スキャンデータのグルーピング設定  
  スキャンデータのデータ数が1/cluster倍になります。

# ビルド方法

1. ソースコードの取得

```
$ cd <ROS2_workspace>/src
$ git clone --recursive https://github.com/UrgNetworks/urg_node2.git
```

2. 関連パッケージのインストール

```
$ rosdep update
$ rosdep install -i --from-paths urg_node2
```

3. ビルド

```
$ cd <ROS2_workspace>
$ colcon build --symlink-install
```

4. テスト（オプション）

`UTM-30LX-EW`を対象としたテストが実行されますので、`UTM-30LX-EW`を接続して電源ON状態で実行してください。

```
$ cd <ROS2_workspace>
$ colcon test
```

# 使用例

## 起動

1. LiDARを接続  
   イーサネットもしくはUSBで接続します。  
1. 接続先（パラメータ）を設定  
   config/params_ether.yaml（イーサネット接続の場合）を編集して接続先の設定を行います。  
   ※USB接続を使用する場合はconfig/params_serial.yamlを編集し`launch/urg_node2.launch.py`のパラメータファイルの指定部分をparams_serial.yamlに変更してください。
1. ノードの起動  
   以下のコマンドを実行すると、自動でActive状態に遷移しスキャンデータの配信が開始されます。

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py
   ```

   自動でActive状態へ遷移させたくない場合は以下のコマンドを実行してください。（起動後、Unconfigured状態になります）

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py auto_start:=false
   ```

   ※`urg_node2.launch.py`ではurg_node2を（コンポーネントとしてではなく）ライフサイクルノードとしてスタンドアロン起動しています。これはコンポーネントかつライフサイクルノードとして起動した場合に、launchでのライフサイクル制御インタフェースがないためです（現時点でのROS2では未実装）。

## パラメータの変更

パラメータの変更を行う場合は以下の手順を実行してください。

1. ノードの終了  
   ノードがすでに起動している場合は\<Ctrl-C\>でノードを終了させます。
1. パラメータを再設定  
   使用しているパラメータファイル（config/params_ether.yamlなど）を編集します。
1. ノードの再起動  
   以下のコマンドでノードを起動します。

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py
   ```

## 異常時のリセット

スキャンデータが配信されないなどの異常が発生した場合は以下の手順を実行してください。

1. ノードの終了  
   発生しているノードを\<Ctrl-C\>で終了させます。
1. 接続確認  
   LiDARの電源や接続が正常かどうか確認します。
1. ノードの再起動  
   以下のコマンドでノードを起動します。

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py
   ```

# 制限事項

- Galacticでは2回目以降のInactive->Active状態遷移が失敗します  
diagnostic_updaterを生成するとdeclare_parameterが必ず実行されます。ROS2では、定義済みパラメータに対してdeclare_parameterを実行するとエラーになるため、状態遷移時のdiagnostic_updater生成に失敗して状態遷移ができません。  
下記のプルリクエストが反映されれば本制限は解消されます。  
  https://github.com/ros/diagnostics/pull/227

