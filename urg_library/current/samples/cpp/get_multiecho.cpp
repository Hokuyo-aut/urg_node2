/*!
  \~japanese
  \example get_multiecho.cpp 距離データ(マルチエコー)を取得する
  \~english
  \example get_multiecho.cpp Obtains multiecho distance data
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
    void print_echo_data(const vector<long>& data, int index,
                         int max_echo_size)
    {
        // [mm]
        for (int i = 0; i < max_echo_size; ++i) {
            cout << data[(max_echo_size * index) + i] << ", ";
        }
    }


    void print_data(const Urg_driver& urg,
                    const vector<long>& data, long time_stamp)
    {
#if 1
        // \~japanese 前方のデータのみを表示
        // \~english Shows only the front step
        int front_index = urg.step2index(0);
        print_echo_data(data, front_index, urg.max_echo_size());
        cout << time_stamp << endl;

#else
        static_cast<void>(urg);

        // \~japanese 全てのデータを表示
        // \~english Prints the multiecho distance for all the measurement points
        size_t data_n = data.size();
        cout << "# n = " << data_n << ", timestamp = " << time_stamp << endl;

        int max_echo_size = urg.max_echo_size();
        for (size_t i = 0; i < data_n; ++i) {
            print_echo_data(data, i, max_echo_size);
            cout << endl;
        }
        cout << endl;
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
    urg.start_measurement(Urg_driver::Multiecho, Urg_driver::Infinity_times, 0);
    for (int i = 0; i < Capture_times; ++i) {
        vector<long> data;
        long time_stamp = 0;

        if (!urg.get_multiecho(data, &time_stamp)) {
            cout << "Urg_driver::get_distance(): " << urg.what() << endl;
            return 1;
        }
        print_data(urg, data, time_stamp);
    }

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
