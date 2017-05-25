#ifndef HRK_SERIAL_H
#define HRK_SERIAL_H

/*!
  \file
  \brief RS-232 シリアル通信

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <string>
#include <vector>
#include "Connection.h"


namespace hrk
{
    //! シリアル接続クラス
    class Serial : public Connection
    {
    public:
        Serial(void);
        ~Serial(void);

        /*!
          \brief 認識されているポート一覧を返す

          \return ポート名の一覧
        */
        static std::vector<std::string> find_ports(void);

        static std::string port_driver_name(const std::string& port_name);

        /*!
          \brief 接続を開く

          \param[in] device_name デバイス名
          \param[in] baudrate ボーレート

          \retval true 成功
          \retval false エラー
        */
        bool open(const std::string& device_name, long baudrate);

        const char* what(void) const;
        bool change_baudrate(long baudrate);
        bool is_open(void) const;
        void close(void);
        int write(const char* data, size_t data_size);
        int read(char* data, size_t max_data_size, int timeout);
        void ungetc(int ch);

    private:
        Serial(const Serial& rhs);
        Serial& operator = (const Serial& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
