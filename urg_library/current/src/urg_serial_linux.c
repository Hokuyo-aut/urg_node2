/*!
  \file
  \~japanese 
  \brief シリアル通信
  \~english Serial communications in Linux
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_ring_buffer.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>


enum {
    INVALID_FD = -1,
};


static void serial_initialize(urg_serial_t *serial)
{
    serial->fd = INVALID_FD;
    serial->has_last_ch = False;

    ring_initialize(&serial->ring, serial->buffer, RING_BUFFER_SIZE_SHIFT);
}


static void serial_clear(urg_serial_t* serial)
{
    tcdrain(serial->fd);
    tcflush(serial->fd, TCIOFLUSH);
    ring_clear(&serial->ring);
    serial->has_last_ch = False;
}


int serial_open(urg_serial_t *serial, const char *device, long baudrate)
{
    int flags = 0;
    int ret = 0;

    serial_initialize(serial);

#ifndef URG_MAC_OS
    enum { O_EXLOCK = 0x0 }; /* \~japanese Linux では使えないのでダミーを作成しておく \~english Not used in Linux, used as dummy */
#endif
    serial->fd = open(device, O_RDWR | O_EXLOCK | O_NONBLOCK | O_NOCTTY);
    if (serial->fd < 0) {
        /* \~japanese 接続に失敗 \~english Connection failed */
        //strerror_r(errno, serial->error_string, ERROR_MESSAGE_SIZE);
        return -1;
    }

    flags = fcntl(serial->fd, F_GETFL, 0);
    fcntl(serial->fd, F_SETFL, flags & ~O_NONBLOCK);

    /* \~japanese シリアル通信の初期化 \~english Initializes serial communication  */
    tcgetattr(serial->fd, &serial->sio);
    serial->sio.c_iflag = 0;
    serial->sio.c_oflag = 0;
    serial->sio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    serial->sio.c_cflag |= CS8 | CREAD | CLOCAL;
    serial->sio.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);

    serial->sio.c_cc[VMIN] = 0;
    serial->sio.c_cc[VTIME] = 0;

    /* \~japanese ボーレートの変更 ~\english Changes the baudrate */
    ret = serial_set_baudrate(serial, baudrate);
    if (ret < 0) {
        return ret;
    }

    /* \~japanese シリアル制御構造体の初期化 \~english Initializes serial control structures */
    serial->has_last_ch = False;

    return 0;
}


void serial_close(urg_serial_t *serial)
{
    if (serial->fd >= 0) {
        close(serial->fd);
        serial->fd = INVALID_FD;
    }
}


int serial_set_baudrate(urg_serial_t *serial, long baudrate)
{
    long baudrate_value = -1;

    switch (baudrate) {
    case 4800:
        baudrate_value = B4800;
        break;

    case 9600:
        baudrate_value = B9600;
        break;

    case 19200:
        baudrate_value = B19200;
        break;

    case 38400:
        baudrate_value = B38400;
        break;

    case 57600:
        baudrate_value = B57600;
        break;

    case 115200:
        baudrate_value = B115200;
        break;

    default:
        return -1;
    }

    /* \~japanese ボーレート変更 \~english Changes the baudrate */
    cfsetospeed(&serial->sio, baudrate_value);
    cfsetispeed(&serial->sio, baudrate_value);
    tcsetattr(serial->fd, TCSADRAIN, &serial->sio);
    serial_clear(serial);

    return 0;
}


int serial_write(urg_serial_t *serial, const char *data, int size)
{
    if (serial->fd == INVALID_FD) {
        return -1;
    }
    return write(serial->fd, data, size);
}


static int wait_receive(urg_serial_t* serial, int timeout)
{
    fd_set rfds;
    struct timeval tv;

    // \~japanese タイムアウト設定
    // \~english Configures the timeout
    FD_ZERO(&rfds);
    FD_SET(serial->fd, &rfds);

    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    if (select(serial->fd + 1, &rfds, NULL, NULL,
               (timeout < 0) ? NULL : &tv) <= 0) {
        /* \~japanese タイムアウト発生 \~english Timeout occurred */
        return 0;
    }
    return 1;
}


static int internal_receive(char data[], int data_size_max,
                            urg_serial_t* serial, int timeout)
{
    int filled = 0;

    if (data_size_max <= 0) {
        return 0;
    }

    while (filled < data_size_max) {
        int require_n;
        int read_n;

        if (! wait_receive(serial, timeout)) {
            break;
        }

        require_n = data_size_max - filled;
        read_n = read(serial->fd, &data[filled], require_n);
        if (read_n <= 0) {
            /* \~japanese 読み出しエラー。現在までの受信内容で戻る \~english Read error, returns all the data up to now */
            break;
        }
        filled += read_n;
    }
    return filled;
}


int serial_read(urg_serial_t *serial, char *data, int max_size, int timeout)
{
    int buffer_size;
    int read_n;
    int filled = 0;

    if (max_size <= 0) {
        return 0;
    }

    /* \~japanese 書き戻した１文字があれば、書き出す  \~english If there is a single character return it */
    if (serial->has_last_ch != False) {
        data[0] = serial->last_ch;
        serial->has_last_ch = False;
        ++filled;
    }
    if (serial->fd == INVALID_FD) {
        if (filled > 0) {
            return filled;
        } else {
            return -1;
        }
    }

    buffer_size = ring_size(&serial->ring);
    read_n = max_size - filled;
    if (buffer_size < read_n) {
        // \~japanese リングバッファ内のデータで足りなければ、データを読み足す
        // \~english Reads data if there is space in the ring buffer
        char buffer[RING_BUFFER_SIZE];
        int n = internal_receive(buffer,
                                 ring_capacity(&serial->ring) - buffer_size,
                                 serial, 0);
        if (n > 0) {
            ring_write(&serial->ring, buffer, n);
            buffer_size += n;
        }
    }

    // \~japanese リングバッファ内のデータを返す
    // \~english Returns the data stored in the ring buffer
    if (read_n > buffer_size) {
        read_n = buffer_size;
    }
    if (read_n > 0) {
        ring_read(&serial->ring, &data[filled], read_n);
        filled += read_n;
    }

    // \~japanese データをタイムアウト付きで読み出す
    // \~english Reads data within the given timeout
    filled += internal_receive(&data[filled], max_size - filled,
                               serial, timeout);
    return filled;
}
