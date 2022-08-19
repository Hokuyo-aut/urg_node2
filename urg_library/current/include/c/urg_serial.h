#ifndef URG_SERIAL_H
#define URG_SERIAL_H

/*!
  \file
  \~japanese
  \brief シリアル通信
  \~english
  \brief Serial communications
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_detect_os.h"

#if defined(URG_WINDOWS_OS)
#include <windows.h>
#else
#include <termios.h>
#endif
#include "urg_ring_buffer.h"


enum {
    RING_BUFFER_SIZE_SHIFT = 7,
    RING_BUFFER_SIZE = 1 << RING_BUFFER_SIZE_SHIFT,

    ERROR_MESSAGE_SIZE = 256,
};


//! \~japanese シリアル通信用  \~english Control information for serial connection
typedef struct
{
#if defined(URG_WINDOWS_OS)
    HANDLE hCom;                //!< \~japanese 接続リソース  \~english Connection resource
    int current_timeout;        //!< \~japanese タイムアウトの設定時間 [msec]  \~english Timeout configuration value
#else
    int fd;                     //!< \~japanese ファイルディスクリプタ  \~english File descriptor
    struct termios sio;         //!< \~japanese 通信設定  \~english Connection configuration
#endif

    ring_buffer_t ring;         //!< \~japanese リングバッファ  \~english Ring buffer structure
    char buffer[RING_BUFFER_SIZE]; //!< \~japanese バッファ領域  \~english Data buffer
    char has_last_ch;          //!< \~japanese 書き戻した文字があるかのフラグ  \~english Whether the last character was received or not
    char last_ch;              //!< \~japanese 書き戻した１文字  \~english Last character received
} urg_serial_t;


//! \~japanese 接続を開く  \~english Opens the connection
extern int serial_open(urg_serial_t *serial, const char *device, long baudrate);


//! \~japanese 接続を閉じる  \~english Closes the connection
extern void serial_close(urg_serial_t *serial);


//! \~japanese ボーレートを設定する  \~english Configures the baudrate
extern int serial_set_baudrate(urg_serial_t *serial, long baudrate);


//! \~japanese データを送信する  \~english Sends data over serial connection
extern int serial_write(urg_serial_t *serial, const char *data, int size);


//! \~japanese データを受信する  \~english Gets data from serial connection
extern int serial_read(urg_serial_t *serial,
                       char *data, int max_size, int timeout);


//! \~japanese 改行までのデータを受信する  \~english Gets data from serial connection until end-of-line
extern int serial_readline(urg_serial_t *serial,
                           char *data, int max_size, int timeout);


//! \~japanese エラー文字列を格納して返す  \~english Stores the serial error message
extern int serial_error(urg_serial_t *serial,
                        char *error_message, int max_size);

#ifdef __cplusplus
}
#endif

#endif /* !URG_SERIAL_H */
