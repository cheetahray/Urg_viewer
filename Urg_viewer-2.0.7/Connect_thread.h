#ifndef CONNECT_THREAD_H
#define CONNECT_THREAD_H

/*!
  \file
  \brief 接続用のスレッド

  \author Satofumi Kamimura

  $Id$
*/

#include <QThread>
#include "Urg_driver.h"

namespace hrk
{
    class Connection;
}


class Connect_thread : public QThread
{
    Q_OBJECT;

public:
    Connect_thread(hrk::Urg_driver& urg);
    ~Connect_thread(void);

    void run(void);

    void set_open_setting(hrk::Connection* connection);
    void set_open_setting(const std::string& device_or_address,
                          long baudrate_or_port,
                          hrk::Urg_driver::connection_t type);

signals:
    void connected(void);
    void connect_failed(const QString& error_message);

private:
    Connect_thread(void);
    Connect_thread(const Connect_thread& rhs);
    Connect_thread& operator = (const Connect_thread& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
