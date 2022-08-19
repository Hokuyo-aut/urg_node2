#ifndef QRK_URG_DRIVER_H
#define QRK_URG_DRIVER_H

/*!
  \file
  \~japanese
  \brief URG ドライバ
  \~english
  \brief URG driver
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include <memory>
#include <string>
#include "Lidar.h"

namespace qrk
{
    //! \~japanese URG ドライバ  \~english URG driver
    class Urg_driver : public Lidar
    {
    public:
        enum {
            Default_baudrate = 115200,
            Default_port = 10940,
            Infinity_times = -1,
        };

        Urg_driver(void);
        virtual ~Urg_driver(void);

        static std::vector<std::string> find_ports(void);
        static std::vector<std::string> find_ports(std::vector<int>&
                                                   is_urg_ports);
        const char* what(void) const;

        bool open(const char* device_name, long baudrate = Default_baudrate,
                  connection_type_t type = Serial);
        void close(void);
        bool is_open(void) const;

        void set_timeout_msec(int msec);

        bool laser_on(void);
        bool laser_off(void);

        bool reboot(void);

        void sleep(void);
        void wakeup(void);
        bool is_stable(void);

        //! \~japanese データ取得の開始  \~english Starts data measurement process
        bool start_measurement(measurement_type_t type = Distance,
                               int scan_times = Infinity_times,
                               int skip_scan = 0);

        //! \~japanese 受信データの受け取り  \~english Receives measurement data
        bool get_distance(std::vector<long>& data, long *time_stamp = NULL);
        bool get_distance_intensity(std::vector<long>& data,
                                    std::vector<unsigned short>& intensity,
                                    long *time_stamp = NULL);

        bool get_multiecho(std::vector<long>& data_multi,
                           long* time_stamp = NULL);

        bool get_multiecho_intensity(std::vector<long>& data_multiecho,
                                     std::vector<unsigned short>&
                                     intensity_multiecho,
                                     long* time_stamp = NULL);

        bool set_scanning_parameter(int first_step, int last_step,
                                    int skip_step = 1);

        //! \~japanese データ取得の中断  \~english Stops data measurement process
        void stop_measurement(void);

        //! \~japanese タイムスタンプの同期  \~english Synchronization of timestamps
        bool start_time_stamp_mode(void);
        bool stop_time_stamp_mode(void);
        bool set_sensor_time_stamp(long time_stamp);
        long get_sensor_time_stamp(void);

        //! \~japanese 角度変換  \~english Angle conversion functions
        double index2rad(int index) const;
        double index2deg(int index) const;
        int rad2index(double radian) const;
        int deg2index(double degree) const;
        int rad2step(double radian) const;
        int deg2step(double degree) const;
        double step2rad(int step) const;
        double step2deg(int step) const;
        int step2index(int step) const;

        int min_step(void) const;
        int max_step(void) const;
        long min_distance(void) const;
        long max_distance(void) const;
        long scan_usec(void) const;
        int max_data_size(void) const;
        int max_echo_size(void) const;

        const char* product_type(void) const;
        const char* firmware_version(void) const;
        const char* serial_id(void) const;
        const char* status(void) const;
        const char* state(void) const;

        int raw_write(const char* data, size_t data_size);
        int raw_read(char* data, size_t max_data_size, int timeout);
        int raw_readline(char* data, size_t max_data_size, int timeout);
        void* raw_urg(void);
        void set_measurement_type(measurement_type_t type);

    private:
        Urg_driver(const Urg_driver& rhs);
        Urg_driver& operator = (const Urg_driver& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif /* !QRK_URG_DRIVER_H */
