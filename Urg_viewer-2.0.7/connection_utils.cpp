/*!
  \file
  \brief Connection 用の補助関数

  \author Satofumi Kamimura

  $Id$
*/

#include <algorithm>
#include "connection_utils.h"
#include "Connection.h"

using namespace std;


namespace
{
    // 改行かどうかの判定
    int is_linefeed(const char ch)
    {
        return ((ch == '\r') || (ch == '\n')) ? 1 : 0;
    }
}


void hrk::ignore(hrk::Connection* connection, int timeout, int size)
{
    enum { Buffer_size = 256 };
    char buffer[Buffer_size];

    int left_size = size;
    while (true) {
        int read_size = (size == Connection_utils_infinity) ?
            Buffer_size : min(left_size, static_cast<int>(Buffer_size));
        int n = connection->read(buffer, read_size, timeout);
        if (n <= 0) {
            return;
        }
        left_size -= n;
    }
}


bool hrk::ignore_to(Connection* connection, int timeout, char ch)
{
    while (true) {
        char buffer;

        int n = connection->read(&buffer, 1, timeout);
        if (n <= 0) {
            return false;
        }

        if (buffer == ch) {
            return true;
        }
    }
}


int hrk::readline(Connection* connection,
                  char* data, int max_data_size, int timeout)
{
    bool is_timeout = false;

    int filled = 0;
    while (filled < max_data_size) {
        // １文字ずつ読み出して評価する
        char recv_ch;
        int n = connection->read(&recv_ch, 1, timeout);
        if (n <= 0) {
            is_timeout = true;
            break;

        } else if (is_linefeed(recv_ch)) {
            break;
        }
        data[filled++] = recv_ch;
    }
    if (filled >= max_data_size) {
        --filled;
        if (filled < 0) {
            return 0;
        }
        connection->ungetc(data[filled]);
    }
    data[filled] = '\0';

    if ((filled == 0) && is_timeout) {
        enum { Timeout = -1 };
        return Timeout;
    } else {
        return filled;
    }

}
