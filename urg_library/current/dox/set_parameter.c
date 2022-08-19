#include "urg_sensor.h"
#include "urg_utils.h"

int main(void)
{
const char connect_device[] = "/dev/ttyACM0";
const long connect_baudrate = 115200;
urg_t urg;
int first_step;
int last_step;
int skip_step;
int scan_times;
int skip_scan;
int ret;
// \~japanese 計測パラメータの設定
// \~english Configures measurement parameters

// \~japanese センサに対して接続を行う。
// \~japanese 接続を行うと、計測パラメータの設定は初期化される
// \~english Connects to the sensor
// \~english Upon connection, measurement parameters are initialized (default values)
ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
// \todo check error code

// \~japanese 計測範囲を指定する
// \~japanese センサ正面方向の 90 [deg] 範囲のデータ取得を行い、ステップ間引きを行わない例
// \~english Defines the measurement scope (start, end steps)
// \~english Defines a measurement scope of 90 [deg] at the front of the sensor, and no step grouping in this example
first_step = urg_rad2step(&urg, -45);
last_step = urg_rad2step(&urg, +45);
skip_step = 0;
ret = urg_set_scanning_parameter(&urg, first_step, last_step, skip_step);
// \todo check error code

// \~japanese 計測回数と計測の間引きを指定して、計測を開始する
// \~japanese 123 回の計測を指示し、スキャンの間引きを行わない例
// \~english Defines the number of scans
// \~english 123 scans are requested, and no scan skipping in this example
scan_times = 123;
skip_scan = 0;
ret = urg_start_measurement(&urg, URG_DISTANCE, scan_times, skip_scan);
// \todo check error code
return 0;
}
