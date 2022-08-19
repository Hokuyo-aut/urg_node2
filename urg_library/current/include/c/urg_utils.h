#ifndef URG_UTILS_H
#define URG_UTILS_H

/*!
  \file
  \~japanese
  \brief URG センサ用の補助関数

  \~english
  \brief URG sensor utility

  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_sensor.h"


    /*!
      \~japanese
      \brief URG のエラーを示す文字列を返す

      \param[in] urg URG センサ管理

      \retval URG のエラーを示す文字列

      \~english
      \brief Returns the string message for the last URG error

      \param[in] urg URG control structure

      \retval String message for the last URG error

      \~
      Example
      \code
      if (!urg_open(&urg, "/dev/ttyACM0", 115200, URG_SERIAL)) {
      printf("urg_open: %s\n", urg_error(&urg));
      return -1;
      } \endcode
    */
    extern const char *urg_error(const urg_t *urg);


    /*!
      \~japanese
      \brief センサが返す距離の最大値、最小値を返す

      センサが返す距離を [最小値, 最大値] で返します。

      \param[in] urg URG センサ管理
      \param[out] min_distance 最小値 [mm]
      \param[out] max_distance 最大値 [mm]

      \~english
      \brief Obtains the minimum and maximum distance values from sensor measurements

      \param[in] urg URG control structure
      \param[out] min_distance minimum distance [mm]
      \param[out] max_distance maximum distance [mm]

      \~
      Example
      \code
      long min_distance, max_distance;
      urg_distance_min_max(&urg, &min_distance, &max_distance);

      for (int i = 0; i < n; ++i) {
      long distance = data[i];
      if ((distance < min_distance) || (distance > max_distance)) {
      continue;
      }
      ...
      } \endcode
    */
    extern void urg_distance_min_max(const urg_t *urg,
                                     long *min_distance, long *max_distance);


    /*!
      \~japanese
      \brief 計測 step の最大値、最小値を返す

      urg_set_scanning_parameter() で指定できる範囲を [最小値, 最大値] で返す。

      \param[in] urg URG センサ管理
      \param[out] min_step 最小値
      \param[out] max_step 最大値

      step はセンサ正面が 0 であり、センサ上部から見た場合の反時計まわりの方向が正、時計まわりの方向が負の step 値となる。

      \image html sensor_step_image.png センサと step の関係

      min_step, max_step の値はセンサによって異なる。

      \~english
      \brief Gets the minimum and maximum step numbers

      Returns the minimum step and maximum step values as configured using urg_set_scanning_parameter()

      \param[in] urg URG control structure
      \param[out] min_step minimum step
      \param[out] max_step maximum step

      As seen from the top of the sensor: the frontal step is 0, going counter-clockwise are positive values, going clockwise are negative values.

      \image html sensor_step_image.png shows the relation between sensor and steps

      The actual values for min_step, max_step change with sensor type/series.

      \~
      Example
      \code
      urg_step_min_max(&urg, &min_step, &max_step);

      printf("range first: %d [deg]\n", urg_step2deg(&urg, min_step));
      printf("range last : %d [deg]\n", urg_step2deg(&urg, max_step)); \endcode

      \see urg_set_scanning_parameter(), urg_step2rad(), urg_step2deg()
    */
    extern void urg_step_min_max(const urg_t *urg, int *min_step, int *max_step);


    /*!
       \~japanese
       \brief １スキャンにかかる時間 [usec] を返す
       \~english
       \brief Returns the time [usec] for 1 scan
    */
    extern long urg_scan_usec(const urg_t *urg);


    /*!
       \~japanese
       \brief 取得データ数の最大値を返す
       \~english
       \brief Returns the maximum size of data received from the sensor
    */
    extern int urg_max_data_size(const urg_t *urg);


    /*!
      \~japanese
      \brief インデックスと角度(radian)の変換を行う

      インデックとは urg_get_distance() などの距離データ取得関数が返したデータ配列についての値である。この関数は、最後に行った距離データ取得関数のデータ配列について有効となる。

      \param[in] urg URG センサ管理
      \param[in] index インデックス

      \return 角度 [radian]

      index は、取得した計測データについての値であり step や角度との関係は取得設定により異なる。

      \image html sensor_index_image.png センサの計測範囲とインデックスの関係

      \~english
      \brief Converts index to angle in radians

      Index is the position of each measurement data in the array returned using urg_get_distance().
      This function applies to the last array of measurement data read from the sensor.

      \param[in] urg URG control structure
      \param[in] index index value

      \return Angle [radian]

      The index value depends on the start/end steps configuration used for measurement.

      \image html sensor_index_image.png shows the relation between start/end step configuration and index

      \~
      Example
      \code
      int n = urg_get_distance(&urg, data, NULL);
      for (int i = 0; i < n; ++i) {
      long distance = data[i];
      double radian = urg_index2rad(i);
      double x = distance * cos(radian);
      double y = distance * sin(radian);
      printf("%.1f, %.1f\n", x, y);
      } \endcode

      \see urg_index2deg(), urg_rad2index(), urg_deg2index()
    */
    extern double urg_index2rad(const urg_t *urg, int index);


    /*!
       \~japanese
       \brief インデックスと角度(degree)の変換を行う
       \~english
       \brief Converts index to angle in degrees
    */
    extern double urg_index2deg(const urg_t *urg, int index);


    /*!
       \~japanese
       \brief 角度(radian)とインデックスの変換を行う
       \~english
       \brief Converts angle in radians to index
    */
    extern int urg_rad2index(const urg_t *urg, double radian);


    /*!
       \~japanese
       \brief 角度(degree)とインデックスの変換を行う
       \~english
       \brief Converts angle in degrees to index
    */
    extern int urg_deg2index(const urg_t *urg, double degree);


    /*!
      \~japanese
      \brief 角度(radian)と step の変換を行う

      urg_step_min_max() で定義されている step について、角度(radian)と step の変換を行う。

      \param[in] urg URG センサ管理
      \param[in] radian 角度 [radian]

      \return step

      \image html sensor_angle_image.png センサの step と角度との関係

      角度から step へ変換した結果が整数でない場合、結果は 0 の方向に切り捨てられた値となる。

      \~english
      \brief Converts angle in radians to step number

      Conversion to angle (radian) is performed according to the min/max step definition using urg_step_min_max().

      \param[in] urg URG control structure
      \param[in] radian angle [radian]

      \return step value

      \image html sensor_angle_image.png shows the relation between steps and angles

      When the conversion from angle to step results on a fractional number, the value is rounded down towards zero (floor).

      \~
      \see urg_step_min_max(), urg_deg2step(), urg_step2rad(), urg_step2deg()
    */
    extern int urg_rad2step(const urg_t *urg, double radian);


    /*!
       \~japanese
       \brief 角度(degree)と step の変換を行う
       \~english
       \brief Converts angle in degrees to step number
    */
    extern int urg_deg2step(const urg_t *urg, double degree);


    /*!
       \~japanese
       \brief step と 角度(radian)の変換を行う
       \~english
       \brief Converts step number to angle in radians
    */
    extern double urg_step2rad(const urg_t *urg, int step);


    /*!
       \~japanese
       \brief step と 角度(degree)の変換を行う
       \~english
       \brief Converts step number to angle in degrees
    */
    extern double urg_step2deg(const urg_t *urg, int step);

    /*!
       \~japanese
       \brief step とインデックスの変換を行う
       \~english
       \brief Converts step number to index
    */
    extern int urg_step2index(const urg_t *urg, int step);

    /*!
       \~japanese
       \brief 指定した時間待つ
       \~english
       \brief Wait at the specified time
    */
    extern void urg_delay(int delay_msec);
#ifdef __cplusplus
}
#endif

#endif /* !URG_UTILS_H */
