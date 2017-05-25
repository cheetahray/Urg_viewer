/*!
  \file
  \brief TCP/IP 通信

  \author Satofumi Kamimura

  $Id$
*/

#include <cstdio>
#include <windows.h>
#include <string>
#include <sys/types.h>
#include "detect_os.h"
#include "Tcpip.h"
#include "Ring_buffer.hpp"

using namespace hrk;
using namespace std;

#if defined(VISUAL_CPP)
#define snprintf _snprintf
#endif


enum {
    Invalid_socket = -1,
};


struct Tcpip::pImpl
{
    string error_message_;
    int socket_;
    Ring_buffer<char> ring_buffer_;


    pImpl(void)
        : error_message_("not opened."), socket_(Invalid_socket)
    {
    }


    pImpl(void* socket, void* socket_set)
    {
        (void)socket;
        (void)socket_set;
        // !!!
    }


    void set_block_mode(void)
    {
        u_long flag = 0;
        ioctlsocket(socket_, FIONBIO, &flag);
    }


    void set_noblock_mode(void)
    {
        u_long flag = 1;
        ioctlsocket(socket_, FIONBIO, &flag);
    }


    void set_connect_fail_error(const char* address, long port,
                                const char* message = NULL)
    {
        enum { Buffer_size = 13 };
        char buffer[Buffer_size];
        snprintf(buffer, Buffer_size, "%ld", port);
        error_message_ = string(address) + ", " + buffer + " : ";
        if (message) {
            error_message_ += message;
        } else {
            error_message_ += strerror(errno);
        }
    }


    bool open(const char* address, long port)
    {
        close();

        static bool is_initialized = false;
        WORD wVersionRequested = 0x0202;
        WSADATA WSAData;
        int err;
        if (!is_initialized) {
            err = WSAStartup(wVersionRequested, &WSAData);
            if (err != 0) {
                set_connect_fail_error(address, port, "WinSock startup fail.");
                return false;
            }
            is_initialized = true;
        }

        int sockaddr_in_size = sizeof(struct sockaddr_in);
        if ((socket_ = (int)socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            set_connect_fail_error(address, port, "socket create fail.");
            return false;
        }

        struct sockaddr_in server;
        memset((char*)&(server), 0, sockaddr_in_size);
        server.sin_family = AF_INET;
        server.sin_port = htons(port);

        if (!strcmp(address, "localhost")) {
            address = "127.0.0.1";
        }

        server.sin_addr.s_addr = inet_addr(address);
        if (server.sin_addr.s_addr == INADDR_NONE) {
            set_connect_fail_error(address, port, "inet_addr fail.");
            return false;
        }

        // ノンブロックモードに変更
        set_noblock_mode();

        if (connect(socket_, (const struct sockaddr *)&server,
                    sockaddr_in_size) == SOCKET_ERROR) {
            int error_number = WSAGetLastError();
            if (error_number != WSAEWOULDBLOCK) {
                set_connect_fail_error(address, port, "connect fail.");
                close();
                return false;
            }

            // コネクション要求は始まったが、まだ完了していない
            fd_set rmask, wmask;
            FD_ZERO(&rmask);
            FD_SET((SOCKET)socket_, &rmask);
            wmask = rmask;

            enum { Connect_timeout_second = 2 };
            struct timeval tv = { Connect_timeout_second, 0 };
            int ret = select((int)socket_ + 1,
                             &rmask, &wmask, NULL, &tv);
            if (ret == 0) {
                // タイムアウト
                set_connect_fail_error(address, port, "connect timeout fail.");
                close();
                return false;
            }
        }

        // ブロックモードに戻す
        set_block_mode();

        ring_buffer_.clear();
        error_message_ = "no error.";
        return true;
    }


    bool is_open(void)
    {
        return (socket_ == Invalid_socket) ? false : true;
    }


    void close(void)
    {
        if (socket_ != Invalid_socket) {
            closesocket(socket_);
            socket_ = Invalid_socket;
        }
    }


    int read(char* data, size_t max_data_size, int timeout)
    {
        if (max_data_size <= 0) {
            return 0;
        }

        int buffer_size = ring_buffer_.size();
        int read_size = max_data_size;
        int filled_size = 0;
        if (buffer_size < read_size) {
            // リングバッファ内のデータで足りなければ、データを読み足す
            enum { Buffer_size = 4096 };
            char buffer[Buffer_size];
            int n = internal_receive(buffer, Buffer_size, 0);
            if (n > 0) {
                buffer_size += ring_buffer_.push(buffer, n);
            }
        }

        // リングバッファ内のデータを返す
        read_size = min(read_size, buffer_size);
        if (read_size > 0) {
            filled_size += ring_buffer_.pop(&data[filled_size], read_size);
        }

        // データをタイムアウト付きで読み出す
        filled_size += internal_receive(&data[filled_size],
                                        max_data_size - filled_size, timeout);
        return filled_size;
    }


    int internal_receive(char data[], int data_size_max, int timeout)
    {
        if (data_size_max <= 0) {
            return 0;
        }

        int filled_size = 0;
        while (filled_size < data_size_max) {
            int require_n;
            int read_n;

            if (!wait_receive(timeout)) {
                break;
            }

            require_n = data_size_max - filled_size;
            read_n = recv(socket_, &data[filled_size], require_n, 0);
            if (read_n <= 0) {
                // 読み出しエラー。現在までの受信内容で戻る
                error_message_ = strerror(errno);
                close();
                break;
            }
            filled_size += read_n;
        }
        return filled_size;
    }


    int wait_receive(int timeout)
    {
        // タイムアウト設定
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(socket_, &rfds);

        struct timeval tv;
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO,
                   (const char*)&tv, sizeof(struct timeval));

        if (select(socket_ + 1, &rfds, NULL, NULL,
                   (timeout < 0) ? NULL : &tv) <= 0) {
            // タイムアウト発生
            return 0;
        }
        return 1;
    }
};


Tcpip::Tcpip(void) : pimpl(new pImpl)
{
}


Tcpip::Tcpip(void* socket, void* socket_set)
    : pimpl(new pImpl(socket, socket_set))
{
}


Tcpip::~Tcpip(void)
{
    close();
}


bool Tcpip::open(const std::string& address, long port)
{
    return pimpl->open(address.c_str(), port);
}


const char* Tcpip::what(void) const
{
    return pimpl->error_message_.c_str();
}


bool Tcpip::change_baudrate(long baudrate)
{
    static_cast<void>(baudrate);

    // 常に成功することにする
    return true;
}


bool Tcpip::is_open(void) const
{
    return pimpl->is_open();
}


void Tcpip::close(void)
{
    pimpl->close();
}


int Tcpip::write(const char* data, size_t data_size)
{
    if (!is_open()) {
        pimpl->error_message_ = "not opened.";
        return -1;
    }

    return send(pimpl->socket_, data, data_size, 0);
}


int Tcpip::read(char* data, size_t max_data_size, int timeout)
{
    if (!is_open()) {
        pimpl->error_message_ = "not opened.";
        return -1;
    }

    return pimpl->read(data, max_data_size, timeout);
}


void Tcpip::ungetc(int ch)
{
    if (!is_open()) {
        pimpl->error_message_ = "not opened.";
        return;
    }

    pimpl->ring_buffer_.ungetc(ch);
}


void Tcpip::set_socket_set(void* socket_set)
{
    (void)socket_set;
    // !!!
}
