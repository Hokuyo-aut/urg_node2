#include "urg_sensor.h"


int main(void)
{
    urg_t urg;
    int ret;
// \~japanese イーサーネット接続でのセンサとの接続と距離データの取得
// \~english Connects to the sensor via Ethernet and receives range data

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;

// \~japanese センサに対して接続を行う。
// \~english Connects to the sensor
ret = urg_open(&urg, URG_ETHERNET, connect_address, connect_port);
// \todo check error code
return 0;
}
