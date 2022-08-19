#ifndef URG_DEBUG_H
#define URG_DEBUG_H

/*!
  \file
  \brief URG debugging functions

  \author Satofumi KAMIMURA

  \~japanese
  \attention 使う必要はありません。

  \~english
  \attention It is not necessary to use these functions.

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_sensor.h"


    /*!
     \~japanese
     \brief センサにデータを直接送信する
     \~english
     \brief Directly send raw data to the sensor
    */
    extern int urg_raw_write(urg_t *urg, const char *data, int data_size);


    /*!
     \~japanese
     \brief センサからデータを直接受信する
     \~english
     \brief Directly get raw data from the sensor
    */
    extern int urg_raw_read(urg_t *urg, char *data, int max_data_size,
                            int timeout);

    /*!
     \~japanese
     \brief センサから改行までのデータを直接受信する
     \~english
     \brief Directly get raw data from the sensor until end-of-line
     */
    extern int urg_raw_readline(urg_t *urg,char *data, int max_data_size,
                                int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG_DEBUG_H */
