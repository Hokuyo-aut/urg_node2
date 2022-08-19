/*!
  \~japanese
  \example sync_time_stamp.cpp センサと PC のタイムスタンプを同期する
  \~english
  \example sync_time_stamp.cpp Timestamp synchronization between PC and sensor
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include "ticks.h"
#include <iostream>

using namespace qrk;
using namespace std;


namespace
{
    void print_timestamp(Urg_driver& urg)
    {
        enum { Print_times = 3 };
        urg.start_time_stamp_mode();

        for (int i = 0; i < Print_times; ++i) {
            cout << ticks() << ", " << urg.get_sensor_time_stamp() << endl;
        }

        urg.stop_time_stamp_mode();
    }
}


int main(int argc, char *argv[])
{
    Connection_information information(argc, argv);

    // \~japanese 接続
    // \~english Connects to the sensor
    Urg_driver urg;
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        cout << "Urg_driver::open(): "
             << information.device_or_ip_name() << ": " << urg.what() << endl;
        return 1;
    }

    cout << "# pc,\tsensor" << endl;

    // \~japanese 比較用に PC とセンサのタイムスタンプを表示する
    // \~english Just to compare, shows the current PC timestamp and sensor timestamp
    print_timestamp(urg);
    cout << endl;

    // \~japanese センサに PC のタイムスタンプを設定し、
    // \~japanese 距離データを取得したときに得られるタイムスタンプが、
    // \~japanese PC から得られるタイムスタンプと同じになるようにする
    // \~english Configures the PC timestamp into the sensor
    // \~english The timestamp value which comes in the measurement data
    // \~english will match the timestamp value from the PC
    urg.set_sensor_time_stamp(ticks());

    // \~japanese 設定後に PC とセンサのタイムスタンプを表示する
    // \~english Displays the PC timestamp and sensor timestamp after configuration
    print_timestamp(urg);

    return 0;
}
