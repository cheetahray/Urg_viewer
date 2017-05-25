#ifndef HRK_RECEIVE_RECORDER_H
#define HRK_RECEIVE_RECORDER_H

/*!
  \file
  \brie 受信データを記録する Connection クラス

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "Connection.h"


namespace hrk
{
    class Receive_recorder : public Connection
    {
    public:
        Receive_recorder(void);
        ~Receive_recorder(void);

        /*!
          \brief 記録対象を登録する

          \param[in] connection 記録対象
          \param[in] file_path 保存するファイル名

          \retval true 成功
          \retval false 失敗
        */
        bool open(Connection* connection, const char* file_path);

        const char* what(void) const;
        bool change_baudrate(long baudrate);
        bool is_open(void) const;
        void close(void);
        int write(const char* data, size_t data_size);
        int read(char* data, size_t max_data_size, int timeout);
        void ungetc(int ch);

    private:
        Receive_recorder(const Receive_recorder& rhs);
        Receive_recorder& operator = (const Receive_recorder& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
