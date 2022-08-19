/*!
  \~japanese
  \example get_distance_intensity.cpp 距離・強度データを取得する
  \~english
  \example get_distance_intensity.cpp Obtains distance and intensity data
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include <iostream>

using namespace qrk;
using namespace std;


namespace
{
    void print_data(const Urg_driver& urg,
                    const vector<long>& data,
                    const vector<unsigned short>& intensity,
                    long time_stamp)
    {
#if 1
        // \~japanese 前方のデータのみを表示
        // \~english Shows only the front step
        int front_index = urg.step2index(0);
        cout << data[front_index] << " [mm], "
             << intensity[front_index] << " [1], ("
             << time_stamp << " [msec])" << endl;

#else
        static_cast<void>(urg);

        // \~japanese 全てのデータを表示
        // \~english Prints the range/intensity values for all the measurement points
        size_t data_n = data.size();
        cout << "# n = " << data_n << ", timestamp = " << time_stamp << endl;
        for (size_t i = 0; i < data_n; ++i) {
            cout << i << ", " << data[i] << ", " << intensity[i] << endl;
        }
#endif
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

    // \~japanese データ取得
    // \~english Gets measurement data
    enum { Capture_times = 10 };
    urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
    for (int i = 0; i < Capture_times; ++i) {
        vector<long> data;
        vector<unsigned short> intensity;
        long time_stamp = 0;

        if (!urg.get_distance_intensity(data, intensity, &time_stamp)) {
            cout << "Urg_driver::get_distance(): " << urg.what() << endl;
            return 1;
        }
        print_data(urg, data, intensity, time_stamp);
    }

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
