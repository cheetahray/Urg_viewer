#ifndef HRK_TCPIP_H
#define HRK_TCPIP_H

/*!
  \file
  \brief TCP/IP 通信

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "Connection.h"


namespace hrk
{
    //! TCP/IP 接続クラス
    class Tcpip : public Connection
    {
    public:
        friend class Accept_server;
        friend class Socket_set;

        Tcpip(void);
        ~Tcpip(void);

        /*!
          \brief 接続を開く

          \param[in] address IP アドレス
          \param[in] port 接続ポート

          \retval true 成功
          \retval false エラー
        */
        bool open(const std::string& address, long port);

        const char* what(void) const;
        bool change_baudrate(long baudrate);
        bool is_open(void) const;
        void close(void);
        int write(const char* data, size_t data_size);
        int read(char* data, size_t max_data_size, int timeout);
        void ungetc(int ch);

    private:
        Tcpip(void* socket, void* socket_set = NULL);
        void set_socket_set(void* socket_set);

        Tcpip(const Tcpip& rhs);
        Tcpip& operator = (const Tcpip& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
