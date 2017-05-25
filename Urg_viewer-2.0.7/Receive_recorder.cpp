/*!
  \file
  \brie 受信データを記録する Connection クラス

  \author Satofumi Kamimura

  $Id$
*/

#include <string>
#include <fstream>
#include "Receive_recorder.h"

using namespace hrk;
using namespace std;


struct Receive_recorder::pImpl
{
    Connection* connection_;
    string error_message_;
    ofstream* fout_;


    pImpl(void) : connection_(NULL), fout_(NULL)
    {
    }


    bool open_file(const char* file_path)
    {
        fout_ = new ofstream(file_path, ios_base::binary);
        if (!fout_->is_open()) {
            error_message_ = string("could not open file: ") + file_path;
            return false;
        }

        return true;
    }


    int read(char* data, size_t max_data_size, int timeout)
    {
        int n = connection_->read(data, max_data_size, timeout);
        if (n > 0) {
            fout_->write(data, n);
        }
        return n;
    }
};


Receive_recorder::Receive_recorder(void) : pimpl(new pImpl)
{
}


Receive_recorder::~Receive_recorder(void)
{
}


bool Receive_recorder::open(Connection* connection, const char* file_path)
{
    if (!connection) {
        pimpl->error_message_ = "connection is NULL.";
        return false;
    }

    bool ret = pimpl->open_file(file_path);
    if (!ret) {
        return false;
    }

    pimpl->connection_ = connection;
    return true;
}


const char* Receive_recorder::what(void) const
{
    if (!pimpl->connection_) {
        return "connection is NULL.";
    }

    if (pimpl->error_message_.empty()) {
        return pimpl->error_message_.c_str();
    } else {
        return pimpl->connection_->what();
    }
}


bool Receive_recorder::change_baudrate(long baudrate)
{
    if (!pimpl->connection_) {
        return false;
    }

    return pimpl->connection_->change_baudrate(baudrate);
}


bool Receive_recorder::is_open(void) const
{
    if (!pimpl->connection_) {
        return false;
    }

    return pimpl->connection_->is_open();
}


void Receive_recorder::close(void)
{
    if (!pimpl->connection_) {
        return;
    }

    pimpl->connection_->close();
    if (pimpl->fout_) {
        delete pimpl->fout_;
        pimpl->fout_ = NULL;
    }
}


int Receive_recorder::write(const char* data, size_t data_size)
{
    if (!pimpl->connection_) {
        return false;
    }

    return pimpl->connection_->write(data, data_size);
}


int Receive_recorder::read(char* data, size_t max_data_size, int timeout)
{
    if (!pimpl->connection_) {
        return -1;
    }

    return pimpl->read(data, max_data_size, timeout);
}


void Receive_recorder::ungetc(int ch)
{
    if (!pimpl->connection_) {
        return;
    }

    pimpl->connection_->ungetc(ch);
}
