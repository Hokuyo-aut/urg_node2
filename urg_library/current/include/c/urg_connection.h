#ifndef URG_CONNECTION_H
#define URG_CONNECTION_H

/*!
  \file
  \~japanese
  \brief 通信の処理
  \~english
  \brief Process communications
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_serial.h"
#include "urg_tcpclient.h"


/*!
  \~japanese
  \brief 定数定義
  \~english
  \brief Defines constants
*/
enum {
    URG_CONNECTION_TIMEOUT = -1, //!< \~japanese タイムアウトが発生したときの戻り値  \~english Return value in case of timeout
};


/*!
  \~japanese
  \brief 通信タイプ
  \~english
  \brief Connection type
*/
typedef enum {
    URG_SERIAL,                 //!< \~japanese シリアル, USB 接続  \~english Serial/USB connection
    URG_ETHERNET,               //!< \~japanese イーサーネット接続  \~english Ethernet connection
} urg_connection_type_t;


/*!
  \~japanese
  \brief 通信リソース
  \~english
  \brief Connection resources
*/
typedef struct
{
    urg_connection_type_t type; //!< \~japanese 接続タイプ  \~english Type of connection
    urg_serial_t serial;        //!< \~japanese シリアル接続 \~english Serial connection
    urg_tcpclient_t tcpclient;  //!< \~japanese イーサーネット接続 \~english Ethernet connection
} urg_connection_t;


/*!
  \~japanese
  \brief 接続

  指定されたデバイスに接続する。

  \param[in,out] connection 通信リソース
  \param[in] connection_type 接続タイプ
  \param[in] device 接続名
  \param[in] baudrate_or_port ボーレート / ポート番号

  \retval 0 正常
  \retval <0 エラー

  connection_type には

  - URG_SERIAL ... シリアル通信
  - URG_ETHERNET .. イーサーネット通信

  を指定する。

  device, baudrate_or_port の指定は connection_type により指定できる値が異なる。
  例えば、シリアル通信の場合は以下のようになる。

  \~english
  \brief Connection

  Connects to the specified device

  \param[in,out] connection Connection resource
  \param[in] connection_type Connection type
  \param[in] device Device name
  \param[in] baudrate_or_port Baudrate or port number

  \retval 0 Success
  \retval <0 Error

  The connection_type is either of:

  - URG_SERIAL ... Serial connection
  - URG_ETHERNET .. Ethernet connection

  device and baudrate_or_port arguments are defined according to connection_type
  For example, in case of serial connection:

  \~
  Example
  \code
  connection_t connection;
  if (! connection_open(&connection, URG_SERIAL, "COM1", 115200)) {
      return 1;
  } \endcode

  And, in case of ethernet connection:

  \~
  Example
  \code
  connection_t connection;
  if (! connection_open(&connection, URG_ETHERNET, "192.168.0.10", 10940)) {
      return 1;
  } \endcode

  \~
  \see connection_close()
*/
extern int connection_open(urg_connection_t *connection,
                           urg_connection_type_t connection_type,
                           const char *device, long baudrate_or_port);


/*!
  \~japanese
  \brief 切断

  デバイスとの接続を切断する。

  \param[in,out] connection 通信リソース
  \~english
  \brief Disconnection

  Closes the connection with the device

  \param[in,out] connection Connection resource
  \~
  \code
  connection_close(&connection); \endcode
  \~
  \see connection_open()
*/
extern void connection_close(urg_connection_t *connection);


/*!
  \~japanese
  \brief ボーレートを設定する
  \~english
  \brief Configures the baudrate
*/
extern int connection_set_baudrate(urg_connection_t *connection, long baudrate);


/*!
  \~japanese
  \brief 送信

  データを送信する。

  \param[in,out] connection 通信リソース
  \param[in] data 送信データ
  \param[in] size 送信バイト数

  \retval >=0 送信データ数
  \retval <0 エラー

  \~english
  \brief Send

  Writes data over the communication channel

  \param[in,out] connection Connection resource
  \param[in] data Data to send
  \param[in] size Number of bytes to send

  \retval >=0 Number of bytes sent
  \retval <0 Error
  \~
  Example
  \code
  n = connection_write(&connection, "QT\n", 3); \endcode

  \~
  \see connection_read(), connection_readline()
*/
extern int connection_write(urg_connection_t *connection,
                            const char *data, int size);


/*!
  \~japanese
  \brief 受信

  データを受信する。

  \param[in,out] connection 通信リソース
  \param[in] data 受信データを格納するバッファ
  \param[in] max_size 受信データを格納できるバイト数
  \param[in] timeout タイムアウト時間 [msec]

  \retval >=0 受信データ数
  \retval <0 エラー

  timeout に負の値を指定した場合、タイムアウトは発生しない。

  1 文字も受信しなかったときは #URG_CONNECTION_TIMEOUT を返す。

  \~english
  \brief Receive

  Reads data from the communication channel

  \param[in,out] connection Connection resource
  \param[in] data Buffer to store received data
  \param[in] max_size Maximum size of the buffer
  \param[in] timeout Timeout [msec]

  \retval >=0 Number of bytes received
  \retval <0 Error

  If timeout argument is negative then the function waits until some data is received

  In case no data is received #URG_CONNECTION_TIMEOUT is returned.
  \~
  Example
  \code
enum {
    BUFFER_SIZE = 256,
    TIMEOUT_MSEC = 1000,
};
char buffer[BUFFER_SIZE];
n = connection_read(&connection, buffer, BUFFER_SIZE, TIMEOUT_MSEC); \endcode

  \~
  \see connection_write(), connection_readline()
*/
extern int connection_read(urg_connection_t *connection,
                           char *data, int max_size, int timeout);


/*!
  \~japanese
  \brief 改行文字までの受信

  改行文字までのデータを受信する。

  \param[in,out] connection 通信リソース
  \param[in] data 受信データを格納するバッファ
  \param[in] max_size 受信データを格納できるバイト数
  \param[in] timeout タイムアウト時間 [msec]

  \retval >=0 受信データ数
  \retval <0 エラー

  data には、'\\0' 終端された文字列が max_size を越えないバイト数だけ格納される。 つまり、受信できる文字のバイト数は、最大で max_size - 1 となる。

  改行文字は '\\r' または '\\n' とする。

  受信した最初の文字が改行の場合は、0 を返し、1 文字も受信しなかったときは #URG_CONNECTION_TIMEOUT を返す。

  \~english
  \brief Receive until end-of-line

  Reads data until the end-of-line character is detected.

  \param[in,out] connection Connection resource
  \param[in] data Buffer to store received data
  \param[in] max_size Maximum size of the buffer
  \param[in] timeout Timeout [msec]

  \retval >=0 Number of bytes received
  \retval <0 Error

  If timeout argument is negative then the function waits until some data is received

  The null terminator character '\\0' is used at the end of data so that the number of bytes does not exceed max_size.
  This is, the maximum number of received characters is max_size - 1.

  The end-of-line character is either '\\r' or '\\n'

  In case no end-of-line is received then returns 0, if no data is received #URG_CONNECTION_TIMEOUT is returned.
  \~
  \see connection_write(), connection_read()
*/
extern int connection_readline(urg_connection_t *connection,
                               char *data, int max_size, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG_CONNECTION_H */
