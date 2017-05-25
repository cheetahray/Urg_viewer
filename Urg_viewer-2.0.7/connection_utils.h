#ifndef HRK_CONNECTION_UTILS_H
#define HRK_CONNECTION_UTILS_H

/*!
  \file
  \brief Connection 用の補助関数

  \author Satofumi Kamimura

  $Id$
*/

#include <cstdio>


namespace hrk
{
    class Connection;

    enum { Connection_utils_infinity = -1 };

    extern void ignore(Connection* connection, int timeout,
                       int size = Connection_utils_infinity);
    extern bool ignore_to(Connection* connection, int timeout, char ch);

    extern int readline(Connection* connection,
                        char* data, int max_data_size, int timeout);
}

#endif
