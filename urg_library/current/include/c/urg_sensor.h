#ifndef URG_SENSOR_H
#define URG_SENSOR_H

/*!
  \file
  \~japanese
  \brief URG センサ制御

  URG 用の基本的な関数を提供します。

  \~english
  \brief URG sensor control

  Provides the basic functions for URG

  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_connection.h"

    /*!
      \~japanese
      \brief 計測タイプ
      \~english
      \brief Measurement types
    */
    typedef enum {
        URG_DISTANCE,           //!< \~japanese 距離  \~english Distance (range)
        URG_DISTANCE_INTENSITY, //!< \~japanese 距離 + 強度  \~english Distance (range) and intensity (strength)
        URG_MULTIECHO,          //!< \~japanese マルチエコーの距離  \~english Multiecho distance
        URG_MULTIECHO_INTENSITY, //!< \~japanese マルチエコーの(距離 + 強度)  \~english Multiecho distance and intensity
        URG_STOP,                //!< \~japanese 計測の停止  \~english Stop measurement
        URG_UNKNOWN,             //!< \~japanese 不明  \~english Unknown measurement type
    } urg_measurement_type_t;

    /*!
      \~japanese
      \brief 距離を何 byte で表現するかの指定
      \~english
      \brief Distance data encoding types (number of bytes)
    */
    typedef enum {
        URG_COMMUNICATION_3_BYTE, //!< \~japanese 距離を 3 byte で表現する  \~english Use 3-bytes encoding for distance
        URG_COMMUNICATION_2_BYTE, //!< \~japanese 距離を 2 byte で表現する  \~english Use 2-bytes encoding for distance
    } urg_range_data_byte_t;


    enum {
        URG_SCAN_INFINITY = 0,  //!< \~japanese 無限回のデータ取得  \~english Continuous data scanning
        URG_MAX_ECHO = 3, //!< \~japanese マルチエコーの最大エコー数  \~english Maximum number of echoes
    };


    /*!
       \~japanese
       \brief エラーハンドラ
       \~english
       \brief Error handler
    */
    typedef urg_measurement_type_t
    (*urg_error_handler)(const char *status, void *urg);


    /*!
      \~japanese
      \brief URG センサ管理

      \~english
      \brief URG sensor control structure
    */
    typedef struct
    {
        int is_active;
        int last_errno;
        urg_connection_t connection;

        int first_data_index;
        int last_data_index;
        int front_data_index;
        int area_resolution;
        long scan_usec;
        int min_distance;
        int max_distance;
        int scanning_first_step;
        int scanning_last_step;
        int scanning_skip_step;
        int scanning_skip_scan;
        urg_range_data_byte_t range_data_byte;

        int timeout;
        int specified_scan_times;
        int scanning_remain_times;
        int is_laser_on;

        int received_first_index;
        int received_last_index;
        int received_skip_step;
        urg_range_data_byte_t received_range_data_byte;
        int is_sending;

        urg_error_handler error_handler;

        char return_buffer[80];
    } urg_t;

    /*!
      \~japanese
      \brief urg_t構造体の初期化

      URG センサ管理構造体(urg_t)を初期化します。

      \param[in,out] urg URG センサ管理

      \attention この関数はurg_open()の初めに実行されます。任意にセンサ管理構造体(urg_t)を初期化したい時はこの関数を呼び出してください。
      \see urg_open()

      \~english
      \brief URG control structure (urg_t) initialization

      Initialize URG control structure(urg_t)

      \param[in,out] urg URG control structure

      \attention
      This function is executed at the start of urg_open ().
      Call this function if you want to initialize the URG control structure (urg_t) arbitrarily.

      \see urg_open()
      \~
    */
    void urg_t_initialize(urg_t *urg);

    /*!
      \~japanese
      \brief 接続

      指定したデバイスに接続し、距離を計測できるようにする。

      \param[in,out] urg URG センサ管理
      \param[in] connection_type 通信タイプ
      \param[in] device_or_address 接続デバイス名
      \param[in] baudrate_or_port 接続ボーレート [bps] / TCP/IP ポート

      \retval 0 正常
      \retval <0 エラー

      connection_type には、以下の項目が指定できます。

      - #URG_SERIAL
      - シリアル、USB 接続

      - #URG_ETHERNET
      - イーサーネット接続

      \~english
      \brief Connect

      Connects to the given device and enables measurement

      \param[in,out] urg URG control structure
      \param[in] connection_type Type of the connection
      \param[in] device_or_address Name of the device
      \param[in] baudrate_or_port Connection baudrate [bps] or TCP/IP port number

      \retval 0 Successful
      \retval <0 Error

      The following values can be used in connection_type:

      - #URG_SERIAL
      - Serial, USB connection

      - #URG_ETHERNET
      - Ethernet connection
      \~
      Example
      \code
      urg_t urg;

      if (urg_open(&urg, URG_SERIAL, "/dev/ttyACM0", 115200) < 0) {
      return 1;
      }

      ...

      urg_close(&urg); \endcode

      \~japanese
      \attention URG C ライブラリの他の関数を呼び出す前に、この関数を呼び出す必要があります。
      \~english
      \attention Call this function before using any other function on the URG library.

      \~
      \see urg_close()
    */
    extern int urg_open(urg_t *urg, urg_connection_type_t connection_type,
                        const char *device_or_address,
                        long baudrate_or_port);


    /*!
      \~japanese
      \brief 切断

      レーザを消灯し、URG との接続を切断します。

      \param[in,out] urg URG センサ管理

      \~english
      \brief Disconnection

      Turns off the laser and closes the connection with the URG sensor.

      \param[in,out] urg URG control structure
      \~
      \see urg_open()
    */
    extern void urg_close(urg_t *urg);


    /*!
      \~japanese
      \brief タイムアウト時間の設定

      \param[in,out] urg URG センサ管理
      \param[in] msec タイムアウトする時間 [msec]

      \attention urg_open() を呼び出すと timeout の設定値はデフォルト値に初期化されるため、この関数は urg_open() 後に呼び出すこと。
      \~english
      \brief Defines the timeout value to use during communication

      \param[in,out] urg URG control structure
      \param[in] msec Timeout value [msec]

      \attention The urg_open() function always sets the timeout value to its default, if necessary call this function after urg_open().
    */
    extern void urg_set_timeout_msec(urg_t *urg, int msec);


    /*!
       \~japanese
       \brief タイムスタンプモードの開始
       \~english
       \brief Starts the timestamp mode (time adjustment state)
    */
    extern int urg_start_time_stamp_mode(urg_t *urg);


    /*!
      \~japanese
      \brief タイムスタンプの取得

      \param[in,out] urg URG センサ管理

      \retval >=0 タイムスタンプ [msec]
      \retval <0 エラー

      \~english
      \brief Read timestamp data

      \param[in,out] urg URG control structure

      \retval >=0 Timestamp value [msec]
      \retval <0 Error

      \~
      Example
      \code
      urg_start_time_stamp_mode(&urg);

      before_ticks = get_pc_msec_function();
      time_stamp = urg_time_stamp(&urg);
      after_ticks = get_pc_msec_function();

      \~japanese
      // タイムスタンプについての計算
      \~english
      // Processing of timestamp data
      \~
      ...

      urg_stop_time_stamp_mode(&urg); \endcode

      \~japanese
      詳しくは \ref sync_time_stamp.c を参照して下さい。
      \~english
      For a detailed use consult the \ref sync_time_stamp.c example
    */
    extern long urg_time_stamp(urg_t *urg);


    /*!
       \~japanese
       \brief タイムスタンプモードの終了
       \~english
       \brief Stops the timestamp mode (returning to idle state)
    */
    extern int urg_stop_time_stamp_mode(urg_t *urg);


    /*!
      \~japanese
      \brief 距離データの取得を開始

      距離データの取得を開始します。実際のデータは urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity() で取得できます。

      \param[in,out] urg URG センサ管理
      \param[in] type データ・タイプ
      \param[in] scan_times データの取得回数
      \param[in] skip_scan データの取得間隔

      \retval 0 正常
      \retval <0 エラー

      type には取得するデータの種類を指定します。

      - #URG_DISTANCE ... 距離データ
      - #URG_DISTANCE_INTENSITY ... 距離データと強度データ
      - #URG_MULTIECHO ... マルチエコー版の距離データ
      - #URG_MULTIECHO_INTENSITY ... マルチエコー版の(距離データと強度データ)

      scan_times は何回のデータを取得するかを 0 以上の数で指定します。ただし、0 または #URG_SCAN_INFINITY を指定した場合は、無限回のデータを取得します。\n
      開始した計測を中断するには urg_stop_measurement() を使います。

      skip_scan はミラーの回転数のうち、１回のスキャン後に何回スキャンしないかを指定します。skip_scan に指定できる範囲は [0, 9] です。

      \image html skip_scan_image.png 何回に１回だけ計測するか

      たとえば、ミラーの１回転が 100 [msec] のセンサで skip_scan に 1 を指定した場合、データの取得間隔は 200 [msec] になります。

      \~english
      \brief Start getting distance measurement data

      Starts measurement data acquisition. The actual data can be retrieved using urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity().

      \param[in,out] urg URG control structure
      \param[in] type Measurement type
      \param[in] scan_times Number of scans to request
      \param[in] skip_scan Interval between scans

      \retval 0 Successful
      \retval <0 Error

      The following values are possible for the type argument

      - #URG_DISTANCE ... Distance (range) data
      - #URG_DISTANCE_INTENSITY ... Distance (range) and intensity (strength) data
      - #URG_MULTIECHO ... Multiecho distance data
      - #URG_MULTIECHO_INTENSITY ... Multiecho distance and intensity data

      scan_times defines how many scans to capture from the sensor: >0 means a fixed number of scans, 0 or #URG_SCAN_INFINITY means continuous (infinite) scanning.
      To interrupt measurement at any time use urg_stop_measurement().

      skip_scan means, after obtaining one scan, skip measurement for the following X mirror (motor) revolutions.
      The values for skip_scan are from the range [0, 9].

      \image html skip_scan_image.png shows scan skip

      For example, for a sensor with one mirror (motor) revolution is 100 [msec] and skip_scan is set to 1, measurement data will be obtained with an interval of 200 [msec].

      \~
      Example
      \code
      enum { CAPTURE_TIMES = 10 };
      urg_start_measurement(&urg, URG_DISTANCE, CAPTURE_TIMES, 0);

      for (i = 0; i < CAPTURE_TIMES; ++i) {
      int n = urg_get_distance(&urg, data, &time_stamp);

      \~japanese
      // 受信したデータの利用
      \~english
      // Processing of obtained data
      \~
      ...
      } \endcode

      \~
      \see urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity(), urg_stop_measurement()
    */
    extern int urg_start_measurement(urg_t *urg, urg_measurement_type_t type,
                                     int scan_times, int skip_scan);


    /*!
      \~japanese
      \brief 距離データの取得

      センサから距離データを取得します。事前に urg_start_measurement() を #URG_DISTANCE 指定で呼び出しておく必要があります。

      \param[in,out] urg URG センサ管理
      \param[out] data 距離データ [mm]
      \param[out] time_stamp タイムスタンプ [msec]

      \retval >=0 受信したデータ個数
      \retval <0 エラー

      data には、センサから取得した距離データが格納されます。data はデータを格納するのサイズを確保しておく必要があります。data に格納されるデータ数は urg_max_data_size() で取得できます。

      time_stamp には、センサ内部のタイムスタンプが格納されます。time_stamp を取得したくない場合 NULL を指定して下さい。

      \~english
      \brief Gets distance data

      Receives distance data from the sensor. The urg_start_measurement() function was called beforehand with #URG_DISTANCE as type argument.

      \param[in,out] urg URG control structure
      \param[out] data Distance data array [mm]
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Distance data received from the sensor are stored in data array. data array should be previously allocated to hold all the data points requested from the sensor. To know how many data points are received, use the urg_max_data_size() function.

      time_stamp will hold the timestamp value stored on the sensor. When not necessary just pass NULL as argument.

      \~
      Example
      \code
      long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));

      ...

      \~japanese
      // データのみ取得する
      \~english
      // Gets only measurement data
      urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
      int n = urg_get_distance(&urg, data, NULL);

      ...

      \~japanese
      // データとタイムスタンプを取得する
      \~english
      // Gets measurement data and timestamp
      long time_stamp;
      urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
      n = urg_get_distance(&urg, data, &time_stamp); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance(urg_t *urg, long data[], long *time_stamp);


    /*!
      \~japanese
      \brief 距離と強度データの取得

      urg_get_distance() に加え、強度データの取得ができる関数です。事前に urg_start_measurement() を #URG_DISTANCE_INTENSITY 指定で呼び出しておく必要があります。

      \param[in,out] urg URG センサ管理
      \param[out] data 距離データ [mm]
      \param[out] intensity 強度データ
      \param[out] time_stamp タイムスタンプ [msec]

      \retval >=0 受信したデータ個数
      \retval <0 エラー

      強度データとは、距離計算に使った波形の反射強度であり、センサのシリーズ毎に特性が異なります。 強度データを使うことで、物体の反射率や環境の大まかな濃淡を推測できます。

      data, time_stamp については urg_get_distance() と同じです。

      intensity には、センサから取得した強度データが格納されます。intensity はデータを格納するのサイズを確保しておく必要があります。intensity に格納されるデータ数は urg_max_data_size() で取得できます。

      \~english
      \brief Gets distance and intensity data

      This is an extension to urg_get_distance() which allows to obtain also the intensity (strength) data. The urg_start_measurement() function was called beforehand with #URG_DISTANCE_INTENSITY as type argument.

      \param[in,out] urg URG control structure
      \param[out] data Distance data array [mm]
      \param[out] intensity Intensity data array
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Intensity data corresponds to the laser strength received during distance calculation. The characteristics of this value changes with sensor series. With some limitations, this property can be used to guess the reflectivity and color shade of an object.

      Regarding data and time_stamp arguments, refer to urg_get_distance().

      Intensity data received from the sensor are stored in intensity array. intensity array should be previously allocated to hold all the data points requested from the sensor. To know how many data points are received, use the urg_max_data_size() function.

      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
      long *data = malloc(data_size * sizeof(long));
      long *intensity = malloc(data_size * sizeof(unsigned short));

      ...

      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
      int n = urg_get_distance_intensity(&urg, data, intesnity, NULLL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance_intensity(urg_t *urg, long data[],
                                          unsigned short intensity[],
                                          long *time_stamp);


    /*!
      \~japanese
      \brief 距離データの取得 (マルチエコー版)

      マルチエコー版の距離データ取得関数です。事前に urg_start_measurement() を #URG_MULTIECHO 指定で呼び出しておく必要があります。

      \param[in,out] urg URG センサ管理
      \param[out] data_multi 距離データ [mm]
      \param[out] time_stamp タイムスタンプ [msec]

      \retval >=0 受信したデータ個数
      \retval <0 エラー

      マルチエコーとは複数の距離データです。 マルチエコーは、１つのレーザ発光において複数の距離データが得られたときに得られます。

      \image html multiecho_image.png マルチエコーのイメージ図

      time_stamp については urg_get_distance() と同じです。

      data_multi には、センサから取得した距離データが１つの step あたり最大で #URG_MAX_ECHO (3 つ)格納されます。マルチエコーが存在しない項目のデータ値は -1 が格納されています。

      \verbatim
      data_multi[0] ... step n の距離データ (1 つめ)
      data_multi[1] ... step n の距離データ (2 つめ)
      data_multi[2] ... step n の距離データ (3 つめ)
      data_multi[3] ... step (n + 1) の 距離データ (1 つめ)
      data_multi[4] ... step (n + 1) の 距離データ (2 つめ)
      data_multi[5] ... step (n + 1) の 距離データ (3 つめ)
      ... \endverbatim

      格納順は、各 step において urg_get_distance() のときと同じ距離のデータが (3n + 0) の位置に格納され、それ以外のデータが (3n + 1), (3n + 2) の位置に降順に格納されます。\n
      つまり data_multi[3n + 1] >= data_multi[3n + 2] になることは保証されますが data_multi[3n + 0] と data_multi[3n + 1] の関係は未定義です。(data_multi[3n + 1] == data_multi[3n + 2] が成り立つのはデータ値が -1 のとき。)

      \~english
      \brief Gets distance data (multiecho mode)

      Receives multiecho distance data from the sensor. The urg_start_measurement() function was called beforehand with #URG_MULTIECHO as type argument.

      \param[in,out] urg URG control structure
      \param[out] data_multi Distance data array [mm]
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Multiecho means multiple range responses (echoes). For a single laser beam, multiple laser returns reflected from different targets may be received, and thus multiple range values are calculated.

      \image html multiecho_image.png shows multiecho measurement

      time_stamp will hold the timestamp value stored on the sensor, same as with urg_get_distance().

      The array data_multi will hold the multiecho range data, up to a maximum of #URG_MAX_ECHO (3) echoes per step. In case the echo does not exists then -1 will be stored on the array.

      \verbatim
      data_multi[0] ... step n range data (1st echo)
      data_multi[1] ... step n range data (2nd echo)
      data_multi[2] ... step n range data (3rd echo)
      data_multi[3] ... step (n + 1) range data (1st echo)
      data_multi[4] ... step (n + 1) range data (2nd echo)
      data_multi[5] ... step (n + 1) range data (3rd echo)
      ... \endverbatim

      In the array, the cells numbered (3n + 0) will hold the range data for first echo (same data as for the urg_get_distance() function), for the other cells (3n + 1) and (3n + 2) data is stored in descending order. \n
      This is, the order data_multi[3n + 1] >= data_multi[3n + 2] is assured, however the relation between data_multi[3n + 0] and data_multi[3n + 1] is not defined. (When data_multi[3n + 1] == data_multi[3n + 2] it means the echo does not exists and the stored value is -1.)

      \~
      Example
      \code
      long *data_multi = malloc(3 * urg_max_data_size(&urg) * sizeof(long));

      ...

      urg_start_measurement(&urg, URG_MULTIECHO, 1, 0);
      int n = urg_get_distance_intensity(&urg, data_multi, NULLL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_multiecho(urg_t *urg, long data_multi[], long *time_stamp);


    /*!
      \~japanese
      \brief 距離と強度データの取得 (マルチエコー版)

      urg_get_multiecho() に加え、強度データの取得できる関数です。事前に urg_start_measurement() を #URG_MULTIECHO_INTENSITY 指定で呼び出しておく必要があります。

      \param[in,out] urg URG センサ管理
      \param[out] data_multi 距離データ [mm]
      \param[out] intensity_multi 強度データ
      \param[out] time_stamp タイムスタンプ [msec]

      \retval >=0 受信したデータ個数
      \retval <0 エラー

      data_multi, time_stamp については urg_get_multiecho() と同じです。

      intensity_multi のデータの並びは data_multi と対応したものになります。intensity_multi に格納されるデータ数は urg_max_data_size() で取得できます。

      \~english
      \brief Gets distance and intensity data (multiecho mode)

      This is an extension to urg_get_multiecho() which allows to obtain also the intensity (strength) data for multiple echoes. The urg_start_measurement() function was called beforehand with #URG_MULTIECHO_INTENSITY as type argument.

      \param[in,out] urg URG control structure
      \param[out] data_multi Distance data array [mm]
      \param[out] intensity_multi Intensity data array
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      data_multi and time_stamp are as described in urg_get_multiecho() function.

      The order of data in the array intensity_multi is defined by how the data_multi array was sorted. The size of the intensity_multi can be obtained using urg_max_data_size().
      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
      long *data_multi = malloc(3 * data_size * sizeof(long));
      long *intensity_multi = malloc(3 * data_size * sizeof(unsigned short));

      ...

      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
      int n = urg_get_multiecho_intensity(&urg, data_multi,
      intesnity_multi, NULLL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_multiecho_intensity(urg_t *urg, long data_multi[],
                                           unsigned short intensity_multi[],
                                           long *time_stamp);


    /*!
      \~japanese
      \brief 計測を中断し、レーザを消灯させます

      \ref urg_start_measurement() の計測を中断します。

      \param[in,out] urg URG センサ管理

      \retval 0 正常
      \retval <0 エラー

      \~english
      \brief Stops measurement process and turns off the laser.

      It stops the measurement started with \ref urg_start_measurement() function.

      \param[in,out] urg URG control structure

      \retval 0 Successful
      \retval <0 Error

      \~
      Example
      \code
      urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
      for (int i = 0; i < 10; ++i) {
      urg_get_distance(&urg, data, NULL);
      }
      urg_stop_measurement(&urg); \endcode

      \~
      \see urg_start_measurement()
    */
    extern int urg_stop_measurement(urg_t *urg);


    /*!
      \~japanese
      \brief 計測範囲を設定します

      センサが計測する範囲を step 値で指定します。urg_get_distance() などの距離データ取得の関数で返されるデータ数は、ここで指定した範囲で制限されます。

      \param[in,out] urg URG センサ管理
      \param[in] first_step 計測の開始 step
      \param[in] last_step 計測の終了 step
      \param[in] skip_step 計測データをグルーピングする個数

      \retval 0 正常
      \retval <0 エラー

      センサの step は、センサ正面を 0 とし、センサ上部から見て反時計まわりの向きが正の値となる順に割り振られます。

      \image html sensor_angle_image.png センサと step の関係

      step の間隔と、最大値、最小値はセンサ依存です。step 値の最大値、最小値は urg_step_min_max() で取得できます。\n

      first_step, last_step でデータの計測範囲を指定します。計測範囲は [first_step, last_step] となります。

      skip_step は、計測データをグルーピングする個数を指定します。指定できる値は [0, 99] です。\n
      skip_step は、指定された数のデータを 1 つにまとめることで、センサから受信するデータ量を減らし、距離取得を行う関数の応答性を高めるときに使います。ただし、データをまとめるため、得られるデータの分解能は減ります。

      例えば以下のような距離データが得られる場合に
      \verbatim
      100, 101, 102, 103, 104, 105, 106, 107, 108, 109
      \endverbatim

      skip_step に 2 を指定すると、得られるデータは
      \verbatim
      100, 102, 104, 106, 108
      \endverbatim

      データは、まとめるデータのうち、一番小さな値のデータが用いられます。

      \~english
      \brief Configure measurement parameters

      This function allows definining the scope (start and end steps) for measurement. The number of measurement data (steps) returned by urg_get_distance() and similar is defined here.

      \param[in,out] urg URG control structure
      \param[in] first_step start step number
      \param[in] last_step end step number
      \param[in] skip_step step grouping factor

      \retval 0 Successful
      \retval <0 Error

      Observing the sensor from the top, the step 0 corresponds to the very front of the sensor, steps at the left side (counter clockwise) of step 0 are positive numbers and those to the right side (clockwise) are negative numbers.

      \image html sensor_angle_image.png shows the relation between sensor and steps

      The spacing between steps, the minimum and maximum step numbers depend on the sensor. Use urg_step_min_max() to get the minimum and maximum step values.\n

      first_step and last_step define the data measurement scope ([first_step, last_step])。

      skip_step allows setting a step grouping factor, where valid values are [0, 99].\n
      With the skip_step parameter, several adjacent steps are grouped and combined into 1 single step, thus the amount of data transmitted from the sensor is reduced and so the response time of measurement data adquisition functions. Of course, grouping several steps into one means the measurement resolution is reduced.

      For example, for the following range data obtained in the sensor:
      \verbatim
      100, 101, 102, 103, 104, 105, 106, 107, 108, 109
      \endverbatim

      And setting skip_step to 2, the range data returned is:
      \verbatim
      100, 102, 104, 106, 108
      \endverbatim

      for each group, the smallest range value is returned.
      \~
      Example
      \code
      urg_set_scanning_parameter(&urg, urg_deg2step(&urg, -45),
      urg_deg2step(&urg, +45), 1);
      urg_start_measurement(&urg, URG_DISTANCE, 0);
      int n = urg_get_distance(&urg, data, NULL);
      for (int i = 0; i < n; ++i) {
      printf("%d [mm], %d [deg]\n", data[i], urg_index2deg(&urg, i));
      } \endcode

      \~
      \see urg_step_min_max(), urg_rad2step(), urg_deg2step()
    */
    extern int urg_set_scanning_parameter(urg_t *urg, int first_step,
                                          int last_step, int skip_step);

    /*!
      \~japanese
      \brief 通信データのサイズ変更

      距離データをセンサから受信の際のデータサイズを変更します。

      \param[in,out] urg URG センサ管理
      \param[in] data_byte 距離値を表現するデータのバイト数

      \retval 0 成功
      \retval <0 エラー

      data_byte には

      - URG_COMMUNICATION_3_BYTE ... 距離を 3 byte で表現する
      - URG_COMMUNICATION_2_BYTE ... 距離を 2 byte で表現する

      を指定できます。\n
      初期状態では距離を 3 byte で表現するようになっています。この設定を 2 byte に設定することで、センサから受信するデータ数は 2/3 になります。ただし、取得できる距離の最大値が 4095 になるため、観測したい対象が 4 [m] 以内の範囲に存在する場合のみ利用して下さい。

      \~english
      \brief Change the size (number of bytes) of measurement data used during communications.

      When receiving data from the sensor, changes the number of bytes used to represent measurement data.

      \param[in,out] urg URG control structure
      \param[in] data_byte Number of bytes used to represent measurement data

      \retval 0 Successful
      \retval <0 Error

      data_byte can be:

      - URG_COMMUNICATION_3_BYTE ... to represent data in 3 bytes
      - URG_COMMUNICATION_2_BYTE ... to represent data in 2 bytes

       \n
      The initial (default) data size is 3 bytes. If the number of bytes is changed to 2, the actual received message length becomes around 2/3 of the original length. However, using 2 bytes means the maximum measurement range is 4095, therefore use it only when measurement targets are 4 [m] from the sensor.
      \~
    */
    extern int urg_set_measurement_data_size(urg_t *urg,
                                               urg_range_data_byte_t data_byte);


    /*!
       \~japanese
       \brief レーザを発光させる
       \~english
       \brief Turns on the laser
    */
    extern int urg_laser_on(urg_t *urg);


    /*!
       \~japanese
       \brief レーザを消灯する
       \~english
       \brief Turns off the laser
    */
    extern int urg_laser_off(urg_t *urg);


    /*!
       \~japanese
       \brief センサを再起動する
       \~english
       \brief Reboots the sensor
    */
    extern int urg_reboot(urg_t *urg);


    /*!
      \~japanese
      \brief センサを低消費電力の状態に遷移させる

      低消費電力のモードでは、スキャナの回転が停止し計測も中断されます。

      - 低消費電力のモード
        - レーザが消灯して計測が中断される。
        - スキャナの回転が停止する。

      低消費電力のモードから抜けるためには \ref urg_wakeup() 関数を呼び出して下さい。

      \~english
      \brief Sets the sensor into low power mode (sleep state)

      During low power mode, the scanner motor stops and so measurement is interrupted.

      - Low power mode
        - Laser is turned off and so measurement is stopped
        - The scanner motor is stopped.

      To recover from low power mode call the function \ref urg_wakeup()
      \~
      \see urg_wakeup()
    */
    extern void urg_sleep(urg_t *urg);


    /*!
      \~japanese
      \brief センサを低消費電力のモードから通常の状態に遷移させる
      \~english
      \brief Returns from the low power mode (sleep state) to the normal mode (idle state)
      \~
      \see urg_sleep()
    */
    extern void urg_wakeup(urg_t *urg);

    /*!
      \~japanese
      \brief センサが計測できる状態かを返す

      \retval 1 センサが計測できる状態にある
      \retval 0 センサが計測できる状態にない

      起動直後でスキャナの回転が安定していない場合や、何らかのエラーで計測できない場合、この関数は 0 を返します。

      \~english
      \brief Returns whether the sensor is stable to perform measurement or not

      \retval 1 The sensor can do measurement
      \retval 0 The sensor cannot do measurement

      Right after power on the motor rotation is not yet stable for measurement, or if any failure condition was detected,
      0 is returned.
    */
    extern int urg_is_stable(urg_t *urg);


    /*!
      \~japanese
      \brief センサ型式を文字列で返す

      センサの型式を文字列で返す。返される文字列はセンサ依存となる。

      \param[in] urg URG センサ管理
      \return センサ型式の文字列

      \~english
      \brief Returns the sensor model string

      Returns the string message corresponding to the sensor model. This message is sensor dependent.

      \param[in] urg URG control structure
      \return sensor model string
    */
    extern const char *urg_sensor_product_type(urg_t *urg);


    /*!
      \~japanese
      \brief センサのシリアル ID 文字列を返す

      センサのシリアル ID 文字列を返す。返される文字列はセンサ依存となる。

      \param[in] urg URG センサ管理
      \return シリアル ID 文字列

      \~english
      \brief Returns the sensor serial number string

      Returns the string message corresponding to the sensor serial number. This message is sensor dependent.

      \param[in] urg URG control structure
      \return serial number string
    */
    extern const char *urg_sensor_serial_id(urg_t *urg);


    /*!
      \~japanese
      \brief センサのバージョン文字列を返す

      センサのソフトウェア・バージョン文字列を返す。返される文字列はセンサ依存となる。

      \param[in] urg URG センサ管理
      \return バージョン文字列

      \~english
      \brief Returns the current sensor firmware version string

      Returns the string message corresponding to the current sensor firmware version. This message is sensor dependent.

      \param[in] urg URG control structure
      \return firmware version string
    */
    extern const char *urg_sensor_firmware_version(urg_t *urg);


    /*!
      \~japanese
      \brief センサのステータス文字列を返す

      センサのステータス文字列を返す。返される文字列はセンサ依存となる。

      \param[in] urg URG センサ管理
      \return ステータス文字列

      \~english
      \brief Returns the current sensor status string

      Returns the string message corresponding to the current sensor status. This message is sensor dependent.

      \param[in] urg URG control structure
      \return current sensor status string
    */
    extern const char *urg_sensor_status(urg_t *urg);


    /*!
      \~japanese
      \brief センサの状態を返す

      センサのステータス文字列を返す。返される文字列はセンサ依存となる。

      \param[in] urg URG センサ管理
      \return 状態を示す文字列

      \attention 状態については SCIP の通信仕様書を参照のこと。

      \~english
      \brief Returns the current sensor state string

      Returns the string message corresponding to the current sensor state. This message is sensor dependent.

      \param[in] urg URG control structure
      \return current sensor state string

      \attention For details, please refer to the SCIP communication protocol specification.
    */
    extern const char *urg_sensor_state(urg_t *urg);


    /*!
      \~japanese
      \brief 計測用のエラーハンドラを登録する

      エラーハンドラは Gx, Mx 系のコマンドの応答が "00" か "99" 以外のときに呼び出される。

      \~english
      \brief Registers an error handler for measurement functions

      The error handler will be called for the Gx, Mx commands when the response code is not "00" or "99".
    */
    extern void urg_set_error_handler(urg_t *urg, urg_error_handler handler);


    /*!
      \~japanese
      \brief SCIP 文字列のデコードを行う

      \param[in] data SCIP 文字列
      \param[in] size data の byte サイズ

      \retval デコード後の数値

      \~english
      \brief Decodes a SCIP message

      \param[in] data the SCIP message to decode
      \param[in] size the data encoding types (number of bytes for encoding)

      \retval Value after decoding
    */
    extern long urg_scip_decode(const char data[], int size);


#ifdef __cplusplus
}
#endif

#endif /* !URG_SENSOR_H */
