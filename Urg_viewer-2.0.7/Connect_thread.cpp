/*!
  \file
  \brief 接続用のスレッド

  \author Satofumi Kamimura

  $Id$
*/

#include "Connect_thread.h"
#include "Connection.h"

using namespace hrk;
using namespace std;


struct Connect_thread::pImpl
{
    Connect_thread* thread_;
    hrk::Urg_driver& urg_;
    Connection* connection_;
    string device_or_address_;
    long baudrate_or_port_;
    Urg_driver::connection_t type_;


    pImpl(Connect_thread* thread, Urg_driver& urg)
        : thread_(thread), urg_(urg)
    {
    }
};


Connect_thread::Connect_thread(hrk::Urg_driver& urg)
    : pimpl(new pImpl(this, urg))
{
}


Connect_thread::~Connect_thread(void)
{
}


void Connect_thread::run(void)
{
    bool ret = false;
    if (pimpl->connection_) {
        ret = pimpl->urg_.open(pimpl->connection_);

    } else if (!pimpl->device_or_address_.empty()) {
        ret = pimpl->urg_.open(pimpl->device_or_address_.c_str(),
                               pimpl->baudrate_or_port_, pimpl->type_);
    }

    if (ret) {
        emit connected();
    } else {
        emit connect_failed(pimpl->urg_.what());
    }
}


void Connect_thread::set_open_setting(hrk::Connection* connection)
{
    pimpl->connection_ = connection;

    pimpl->device_or_address_.clear();
}


void Connect_thread::set_open_setting(const std::string& device_or_address,
                                      long baudrate_or_port,
                                      hrk::Urg_driver::connection_t type)
{
    pimpl->device_or_address_ = device_or_address;
    pimpl->baudrate_or_port_ = baudrate_or_port;
    pimpl->type_ = type;

    pimpl->connection_ = NULL;
}
