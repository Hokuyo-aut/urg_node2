/*!
  \file
  \~japanese TCP/IP 読み込み/書き込み　関数 
  \brief
  \~english
  \brief TCP/IP read/write functions
  \~
  \author Katsumi Kimoto

  $Id$
*/

// http://www.ne.jp/asahi/hishidama/home/tech/lang/socket.html

#include "urg_detect_os.h"
#include <string.h>
#if defined(URG_WINDOWS_OS)
#else
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#endif
#include "urg_tcpclient.h"

#include <stdio.h>

enum {
    Invalid_desc = -1,
};

// \~japanese 改行かどうかの判定
// \~english Checks wheter is is a EOL character
static int is_linefeed(const char ch)
{
    return ((ch == '\r') || (ch == '\n')) ? 1 : 0;
}


static void tcpclient_buffer_init(urg_tcpclient_t* cli)
{
    ring_initialize(&cli->rb, cli->buf, RB_BITSHIFT);
}


// get number of data in buffer.
static int tcpclient_buffer_data_num(urg_tcpclient_t* cli)
{
    return ring_size(&cli->rb);
}


static int tcpclient_buffer_write(urg_tcpclient_t* cli,
                                  const char* data, int size)
{
    return ring_write(&cli->rb, data, size);
}


static int tcpclient_buffer_read(urg_tcpclient_t* cli, char* data, int size)
{
    return ring_read(&cli->rb, data, size);
}


static void set_block_mode(urg_tcpclient_t* cli)
{
#if defined(URG_WINDOWS_OS)
    u_long flag = 0;
    ioctlsocket(cli->sock_desc, FIONBIO, &flag);
#else
    int flag = 0;
    fcntl(cli->sock_desc, F_SETFL, flag);
#endif
}


int tcpclient_open(urg_tcpclient_t* cli, const char* ip_str, int port_num)
{
    enum { Connect_timeout_second = 2 };
    fd_set rmask, wmask;
    struct timeval tv = { Connect_timeout_second, 0 };
#if defined(URG_WINDOWS_OS)
    u_long flag;
#else
    int flag;
    int sock_optval = -1;
    int sock_optval_size = sizeof(sock_optval);
#endif
    int ret;

    cli->sock_desc = Invalid_desc;
    cli->pushed_back = -1; // no pushed back char.

#if defined(URG_WINDOWS_OS)
    {
        static int is_initialized = 0;
        WORD wVersionRequested = 0x0202;
        WSADATA WSAData;
        int err;
        if (!is_initialized) {
            err = WSAStartup(wVersionRequested, &WSAData);
            if (err != 0) {
                return -1;
            }
            is_initialized = 1;
        }
    }
#endif

    tcpclient_buffer_init(cli);

    cli->sock_addr_size = sizeof (struct sockaddr_in);
    if ((cli->sock_desc = (int)socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        return -1;
    }

    memset((char*)&(cli->server_addr), 0, sizeof(cli->sock_addr_size));
    cli->server_addr.sin_family = AF_INET;
    cli->server_addr.sin_port = htons(port_num);

    if (!strcmp(ip_str, "localhost")) {
        ip_str = "127.0.0.1";
    }

    /* bind is not required, and port number is dynamic */
    if ((cli->server_addr.sin_addr.s_addr = inet_addr(ip_str)) == INADDR_NONE) {
        return -1;
    }

#if defined(URG_WINDOWS_OS)
    // \~japanese ノンブロックに変更
    // \~english Configures non-blocking mode
    flag = 1;
    ioctlsocket(cli->sock_desc, FIONBIO, &flag);

    if (connect(cli->sock_desc, (const struct sockaddr *)&(cli->server_addr),
                cli->sock_addr_size) == SOCKET_ERROR) {
        int error_number = WSAGetLastError();
        if (error_number != WSAEWOULDBLOCK) {
            tcpclient_close(cli);
            return -1;
        }

        FD_ZERO(&rmask);
        FD_SET((SOCKET)cli->sock_desc, &rmask);
        wmask = rmask;

        ret = select((int)cli->sock_desc + 1, &rmask, &wmask, NULL, &tv);
        if (ret == 0) {
            // \~japanese タイムアウト
            // \~english Operation timed out
            tcpclient_close(cli);
            return -2;
        }
    }
    // \~japanese ブロックモードにする
    // \~english Returns to blocking mode
    set_block_mode(cli);

#else
    // \~japanese ノンブロックに変更
    // \~english Configures non-blocking mode
    flag = fcntl(cli->sock_desc, F_GETFL, 0);
    fcntl(cli->sock_desc, F_SETFL, flag | O_NONBLOCK);

    if (connect(cli->sock_desc, (const struct sockaddr *)&(cli->server_addr),
                cli->sock_addr_size) < 0) {
        if (errno != EINPROGRESS) {
            tcpclient_close(cli);
            return -1;
        }

        // \~japanese EINPROGRESS:コネクション要求は始まったが、まだ完了していない
        // \~english EINPROGRESS: a connection request was already received and not completed yet
        FD_ZERO(&rmask);
        FD_SET(cli->sock_desc, &rmask);
        wmask = rmask;

        ret = select(cli->sock_desc + 1, &rmask, &wmask, NULL, &tv);
        if (ret <= 0) {
            // \~japanese タイムアウト処理
            // \~english Operation timed out
            tcpclient_close(cli);
            return -2;
        }

        if (getsockopt(cli->sock_desc, SOL_SOCKET, SO_ERROR, (int*)&sock_optval,
                       (socklen_t*)&sock_optval_size) != 0) {
            // \~japanese 接続に失敗
            // \~english Connection failed
            tcpclient_close(cli);
            return -3;
        }

        if (sock_optval != 0) {
            // \~japanese 接続に失敗
            // \~english Connection failed
            tcpclient_close(cli);
            return -4;
        }
        // \~japanese ブロックモードにする
        // \~english Returns to blocking mode
        set_block_mode(cli);
    }
#endif

    return 0;
}


void tcpclient_close(urg_tcpclient_t* cli)
{
    if (cli->sock_desc != Invalid_desc) {
#if defined(URG_WINDOWS_OS)
        closesocket(cli->sock_desc);
        //WSACleanup();
#else
        close(cli->sock_desc);
#endif
        cli->sock_desc = Invalid_desc;
    }
}


int tcpclient_read(urg_tcpclient_t* cli,
                   char* userbuf, int req_size, int timeout)
{
    // number of data in buffer.
    int num_in_buf = tcpclient_buffer_data_num(cli);
    int sock       = cli->sock_desc;
    int rem_size   = req_size;  // remaining size to be sent back.
    int n;

    // copy data in buffer to user buffer and return with requested size.
    if (num_in_buf > 0) {
        n = tcpclient_buffer_read(cli, userbuf, req_size);
        rem_size = req_size - n;  // lacking size.
        if (rem_size <= 0) {
            return req_size;
        }

        num_in_buf = tcpclient_buffer_data_num(cli);
    }

    // data in buffer was not enough, read from socket to fill buffer,
    // without blocking, i.e. read from system's buffer.
    {
        char tmpbuf[BUFSIZE];
        // receive with non-blocking mode.
#if defined(URG_WINDOWS_OS)
        int no_timeout = 1;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&no_timeout, sizeof(struct timeval));
        n = recv(sock, tmpbuf, BUFSIZE - num_in_buf, 0);
#else
        n = recv(sock, tmpbuf, BUFSIZE - num_in_buf, MSG_DONTWAIT);
#endif
        if (n > 0) {
            tcpclient_buffer_write(cli, tmpbuf, n); // copy socket to my buffer
        }

        n = tcpclient_buffer_read(cli, &userbuf[req_size-rem_size], rem_size);
        // n never be greater than rem_size
        rem_size -= n;
        if (rem_size <= 0) {
            return req_size;
        }
    }

    //  lastly recv with blocking but with time out to read necessary size.
    {
#if defined(URG_WINDOWS_OS)
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   (const char *)&timeout, sizeof(struct timeval));
#else
        struct timeval tv;
        tv.tv_sec = timeout / 1000; // millisecond to seccond
        tv.tv_usec = (timeout % 1000) * 1000; // millisecond to microsecond
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
#endif
        //4th arg 0:no flag
        n = recv(sock, &userbuf[req_size-rem_size], rem_size, 0);
        // n never be greater than rem_size
        if (n > 0) {
            rem_size -= n;
        }
    }

    return (req_size - rem_size); // last return may be less than req_size;
}


int tcpclient_write(urg_tcpclient_t* cli, const char* buf, int size)
{
    // blocking if data size is larger than system's buffer.
    return send(cli->sock_desc, buf, size, 0);  //4th arg 0: no flag
}


int tcpclient_error(urg_tcpclient_t* cli, char* error_message, int max_size)
{
    (void)cli;
    (void)error_message;
    (void)max_size;

    // not implemented yet.

    return -1;
}


int tcpclient_readline(urg_tcpclient_t* cli,
                       char* userbuf, int buf_size, int timeout)
{
    int n = 0;
    int i = 0;

    if (cli->pushed_back > 0) {
        userbuf[i] = cli->pushed_back;
        i++;
        cli->pushed_back = -1;
    }
    for (; i < buf_size; ++i) {
        char ch;
        n = tcpclient_read(cli, &ch, 1, timeout);
        if (n <= 0) {
            break; // error
        }
        if (is_linefeed(ch)) {
            break; // success
        }
        userbuf[i] = ch;
    }

    if (i >= buf_size) { // No CR or LF found.
        --i;
        cli->pushed_back = userbuf[buf_size - 1] & 0xff;
        userbuf[buf_size - 1] = '\0';
    }
    userbuf[i] = '\0';

    if (i == 0 && n <= 0) { // error
        return -1;
    }

    return i; // the number of characters filled into user buffer.
}
