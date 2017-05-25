/*!
  \file
  \brief RS-232 シリアル通信 (Linux, Mac)

  \author Satofumi Kamimura

  $Id$
*/

#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include "Ring_buffer.hpp"
#include "Serial.h"

using namespace hrk;
using namespace std;


namespace
{
    enum {
        Invalid_fd = -1,
    };
}


struct Serial::pImpl
{
    string error_message_;
    int fd_;
    struct termios sio_;
    Ring_buffer<char> ring_buffer_;


    pImpl(void) : error_message_("no error."), fd_(Invalid_fd)
    {
    }


    bool open(const char* device_name, long baudrate)
    {
        close();

#ifndef MAC_OS
        // Linux では使えないのでダミーを作成する
        enum { O_EXLOCK = 0x0 };
#endif
        fd_ = ::open(device_name, O_RDWR | O_EXLOCK | O_NONBLOCK | O_NOCTTY);
        if (fd_ < 0) {
            // 接続に失敗
            error_message_ = string(device_name) + ": " + strerror(errno);
            return false;
        }

        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

        // シリアル通信の初期化
        tcgetattr(fd_, &sio_);
        sio_.c_iflag = 0;
        sio_.c_oflag = 0;
        sio_.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
        sio_.c_cflag |= CS8 | CREAD | CLOCAL;
        sio_.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);

        sio_.c_cc[VMIN] = 0;
        sio_.c_cc[VTIME] = 0;

        // ボーレートの変更
        return set_baudrate(baudrate);
    }


    void close(void)
    {
        if (is_open()) {
            ::close(fd_);
            fd_ = Invalid_fd;
        }
    }


    bool is_open(void) const
    {
        return (fd_ == Invalid_fd) ? false : true;
    }


    bool set_baudrate(long baudrate)
    {
        typedef struct
        {
            long baudrate;
            long value;
        } baudrate_table_t;

        baudrate_table_t table[] = {
            { 4800, B4800 },
            { 9600, B9600 },
            { 19200, B19200 },
            { 38400, B38400 },
            { 57600, B57600 },
            { 115200, B115200 },
        };

        enum { Invalid_value = -1 };
        long baudrate_value = Invalid_value;
        size_t table_size = sizeof(table) / sizeof(table[0]);
        for (size_t i = 0; i < table_size; ++i) {
            if (table[i].baudrate == baudrate) {
                baudrate_value = table[i].value;
                break;
            }
        }
        if (baudrate_value == Invalid_value) {
            return false;
        }

        // ボーレート変更
        cfsetospeed(&sio_, baudrate_value);
        cfsetispeed(&sio_, baudrate_value);
        tcsetattr(fd_, TCSADRAIN, &sio_);

        clear();

        return true;
    }


    void clear(void)
    {
        tcdrain(fd_);
        tcflush(fd_, TCIOFLUSH);
        ring_buffer_.clear();
    }


    int read(char* data, size_t max_data_size, int timeout)
    {
        if (!is_open()) {
            error_message_ = "not opened.";
            return -1;
        }

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
                ring_buffer_.push(buffer, n);
                buffer_size += n;
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
        int filled_size = 0;

        if (data_size_max <= 0) {
            return 0;
        }

        while (filled_size < data_size_max) {
            int require_n;
            int read_n;

            if (!wait_receive(timeout)) {
                break;
            }

            require_n = data_size_max - filled_size;
            read_n = ::read(fd_, &data[filled_size], require_n);
            if (read_n <= 0) {
                // 読み出しエラー。現在までの受信内容で戻る
                break;
            }
            filled_size += read_n;
        }
        return filled_size;
    }


    bool wait_receive(int timeout)
    {
        // タイムアウト設定
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        struct timeval tv;
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;

        if (select(fd_ + 1, &rfds, NULL, NULL,
                   (timeout < 0) ? NULL : &tv) <= 0) {
            // タイムアウト発生
            return false;
        }
        return true;
    }
};


Serial::Serial(void) : pimpl(new pImpl)
{
}


Serial::~Serial(void)
{
    close();
}


std::vector<std::string> Serial::find_ports(void)
{
    // find "/dev/ttyUSB\d" or "/dev/ttyACM\d" file
    typedef struct
    {
        const char* dir_name;
        const char* file_basename;
    } find_pattern_t;

    find_pattern_t patterns[] = {
        { "/dev", "ttyUSB" },
        { "/dev", "ttyACM" },
        { "/dev", "tty.usbmodem" },
    };

    vector<string> found_files;
    size_t patterns_size = sizeof(patterns) / sizeof(patterns[0]);
    for (size_t i = 0; i < patterns_size; ++i) {
        const char* dir_name = patterns[i].dir_name;
        DIR* dir = opendir(dir_name);
        if (!dir) {
            continue;
        }

        // マッチしたファイル名を登録する
        const char* basename = patterns[i].file_basename;
        size_t basename_size = strlen(basename);
        struct dirent* entry;
        while ((entry = readdir(dir))) {
            string file_name = entry->d_name;
            if (!file_name.compare(0, basename_size,
                                   basename, basename_size)) {
                found_files.push_back(dir_name + ("/" + file_name));
            }
        }
        closedir(dir);
    }

    return found_files;
}


std::string Serial::port_driver_name(const string& port_name)
{
    static_cast<void>(port_name);
    // 実装しない
    return "";
}


const char* Serial::what(void) const
{
    return pimpl->error_message_.c_str();
}


bool Serial::open(const std::string& device_name, long baudrate)
{
    return pimpl->open(device_name.c_str(), baudrate);
}


bool Serial::change_baudrate(long baudrate)
{
    return pimpl->set_baudrate(baudrate);
}


bool Serial::is_open(void) const
{
    return pimpl->is_open();
}


void Serial::close(void)
{
    pimpl->close();
}


int Serial::write(const char* data, size_t data_size)
{
    if (!is_open()) {
        pimpl->error_message_ = "not opened.";
        return -1;
    }
    return ::write(pimpl->fd_, data, data_size);
}


int Serial::read(char* data, size_t max_data_size, int timeout)
{
    return pimpl->read(data, max_data_size, timeout);
}


void Serial::ungetc(int ch)
{
    pimpl->ring_buffer_.ungetc(ch);
}
