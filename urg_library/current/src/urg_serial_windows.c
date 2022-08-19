/*!
  \file
  \~japanese 
  \brief シリアル通信
  \~english 
  \brief Serial communications on Windows
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_serial.h"
#include <stdio.h>

#if defined(URG_MSC)
#define False 0
#endif


static void serial_initialize(urg_serial_t *serial)
{
    serial->hCom = INVALID_HANDLE_VALUE;
    serial->has_last_ch = False;

    ring_initialize(&serial->ring, serial->buffer, RING_BUFFER_SIZE_SHIFT);
}


static void set_timeout(urg_serial_t *serial, int timeout)
{
    COMMTIMEOUTS timeouts;
    GetCommTimeouts(serial->hCom, &timeouts);

    timeouts.ReadIntervalTimeout = (timeout == 0) ? MAXDWORD : 0;
    timeouts.ReadTotalTimeoutConstant = timeout;
    timeouts.ReadTotalTimeoutMultiplier = 0;

    SetCommTimeouts(serial->hCom, &timeouts);
}


int serial_open(urg_serial_t *serial, const char *device, long baudrate)
{
    // \~japanese COM10 以降への対応用
    // \~english To deal with port names over COM10
    enum { NameLength = 11 };
    char adjusted_device[NameLength];

    serial_initialize(serial);

    /* \~japanese COM ポートを開く \~english Opens the COM port */
    _snprintf(adjusted_device, NameLength, "\\\\.\\%s", device);
    serial->hCom = CreateFileA(adjusted_device, GENERIC_READ | GENERIC_WRITE,
                               0, NULL, OPEN_EXISTING,
                               FILE_ATTRIBUTE_NORMAL, NULL);

    if (serial->hCom == INVALID_HANDLE_VALUE) {
        // !!! store error_message buffer
        //printf("open failed: %s\n", device);
        return -1;
    }

    /* \~japanese 通信サイズの更新  \~english Configures the transmission size */
    SetupComm(serial->hCom, 4096 * 8, 4096);

    /* \~japanese ボーレートの変更 ~\english Changes the baudrate */
    serial_set_baudrate(serial, baudrate);

    /* \~japanese シリアル制御構造体の初期化 \~english Initializes serial control structures */
    serial->has_last_ch = False;

    /* \~japanese タイムアウトの設定  \~english Configures the timeout */
    serial->current_timeout = 0;
    set_timeout(serial, serial->current_timeout);

    return 0;
}


void serial_close(urg_serial_t *serial)
{
    if (serial->hCom != INVALID_HANDLE_VALUE) {
        CloseHandle(serial->hCom);
        serial->hCom = INVALID_HANDLE_VALUE;
    }
}


int serial_set_baudrate(urg_serial_t *serial, long baudrate)
{
    long baudrate_value;
    DCB dcb;

    switch (baudrate) {

    case 4800:
        baudrate_value = CBR_4800;
        break;

    case 9600:
        baudrate_value = CBR_9600;
        break;

    case 19200:
        baudrate_value = CBR_19200;
        break;

    case 38400:
        baudrate_value = CBR_38400;
        break;

    case 57600:
        baudrate_value = CBR_57600;
        break;

    case 115200:
        baudrate_value = CBR_115200;
        break;

    default:
        baudrate_value = baudrate;
    }

    GetCommState(serial->hCom, &dcb);
    dcb.BaudRate = baudrate_value;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.fParity = FALSE;
    dcb.StopBits = ONESTOPBIT;

    dcb.fBinary = TRUE; //If this member is TRUE, binary mode is enabled. Windows does not support nonbinary mode transfers, so this member must be TRUE.
    dcb.fInX = FALSE;
    dcb.fOutX = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fNull = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    /*
    if (SetCommState(serial->hCom, &dcb) == 0) {
        flush();
        return -1;
    } else {
        return 0;
    }
    */
    SetCommState(serial->hCom, &dcb);
    return 0;
}


int serial_write(urg_serial_t *serial, const char *data, int size)
{
    DWORD n;

    if (size < 0) {
        return 0;
    }

    if (serial->hCom == INVALID_HANDLE_VALUE) {
        return -1;
    }

    WriteFile(serial->hCom, data, (DWORD)size, &n, NULL);
    return n;
}


static int internal_receive(char data[], int max_size,
                            urg_serial_t* serial, int timeout)
{
    int filled = 0;
    DWORD n;

    if (timeout != serial->current_timeout) {
        set_timeout(serial, timeout);
        serial->current_timeout = timeout;
    }

    ReadFile(serial->hCom, &data[filled], (DWORD)max_size - filled, &n, NULL);

    return filled + n;
}


int serial_read(urg_serial_t *serial, char *data, int max_size, int timeout)
{
    int filled = 0;
    int buffer_size;
    int read_n;

    if (max_size <= 0) {
        return 0;
    }

    /* \~japanese 書き戻した１文字があれば、書き出す  \~english If there is a single character return it */
    if (serial->has_last_ch) {
        data[0] = serial->last_ch;
        serial->has_last_ch = False;
        ++filled;
    }

    if (serial->hCom == INVALID_HANDLE_VALUE) {
        if (filled > 0) {
            return filled;
        }
        return -1;
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
        ring_write(&serial->ring, buffer, n);
    }
    buffer_size = ring_size(&serial->ring);

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
    filled += internal_receive(&data[filled],
                               max_size - filled, serial, timeout);
    return filled;
}
