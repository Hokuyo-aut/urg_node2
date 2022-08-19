// \~japanese シリアル接続でのセンサとの接続と距離データの取得
// \~english Connects to the sensor via serial interface and gets range data

#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>


int main(void)
{
    urg_t urg;
    int ret;
    long *length_data;
    int length_data_size;

    // \~japanese "COM1" は、センサが認識されているデバイス名にする必要がある
    // \~english "COM1" is, in this case, the device name detected for the sensor
    const char connect_device[] = "COM1";
    const long connect_baudrate = 115200;

    // \~japanese センサに対して接続を行う。
    // \~english Connects to the sensor
    ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
    // \todo check error code

    // \~japanese データ受信のための領域を確保する
    // \~english Allocates memory to hold received measurement data
    length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
    // \todo check length_data is not NULL

    // \~japanese 距離データの計測開始。
    // \~english Starts range data measurement
    ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
    // \todo check error code

    // \~japanese センサから距離データを取得する。
    // \~english Receives the measurement data
    length_data_size = urg_get_distance(&urg, length_data, NULL);
    // \todo process length_data array

    // \~japanese センサとの接続を閉じる。
    // \~english Disconnects from the sensor
    urg_close(&urg);

    return 0;
}
