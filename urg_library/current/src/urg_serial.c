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

#include "urg_serial.h"


enum {
    False = 0,
    True,
};


#if defined(URG_WINDOWS_OS)
#include "urg_serial_windows.c"
#else
#include "urg_serial_linux.c"
#endif


// \~japanese 改行かどうかの判定
// \~english Checks wheter is is a EOL character
static int is_linefeed(const char ch)
{
    return ((ch == '\r') || (ch == '\n')) ? 1 : 0;
}


static void serial_ungetc(urg_serial_t *serial, char ch)
{
    serial->has_last_ch = True;
    serial->last_ch = ch;
}


int serial_readline(urg_serial_t *serial, char *data, int max_size, int timeout)
{
    /* \~japanese １文字ずつ読み出して評価する */
    /* \~english Reads and evaluates 1 character at a time */
    int filled = 0;
    int is_timeout = 0;

    while (filled < max_size) {
        char recv_ch;
        int n = serial_read(serial, &recv_ch, 1, timeout);
        if (n <= 0) {
            is_timeout = 1;
            break;
        } else if (is_linefeed(recv_ch)) {
            break;
        }
        data[filled++] = recv_ch;
    }
    if (filled >= max_size) {
        --filled;
        serial_ungetc(serial, data[filled]);
    }
    data[filled] = '\0';

    if ((filled == 0) && is_timeout) {
        return -1;
    } else {
        //fprintf(stderr, "%s\n", data);
        return filled;
    }
}
