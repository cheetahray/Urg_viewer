#ifndef HRK_STREAM_H
#define HRK_STREAM_H

/*!
  \file
  \brief ストリーム操作

  \author Satofumi Kamimura

  $Id$
*/

#include <cstddef>


namespace hrk
{
    class Stream
    {
    public:
        virtual ~Stream(void)
        {
        }

        /*!
          \brief 受信

          \retval true 接続中
          \retval false 接続していない
        */
        virtual bool is_open(void) const = 0;

        /*!
          \brief 接続を閉じる

          接続されてないときに呼び出されたときは、何もしない。
        */
        virtual void close(void) = 0;

        /*!
          \brief データの送信

          \param[in] data 送信するデータ
          \param[in] data_size 送信するデータの byte 数

          \retval >=0 送信した byte 数
          \retval <0 エラー ID
        */
        virtual int write(const char* data, size_t data_size) = 0;

        /*!
          \brief データの受信

          \param[in] data 受信データ用のバッファ
          \param[in] max_data_size 受信できるデータの最大 byte 数
          \param[in] timeout 受信を待つ時間 [msec]。無限に待つ場合には Timeout_infinity を指定する。

          \retval >=0 送信した byte 数
          \retval <0 エラー ID
        */
        virtual int read(char* data, size_t max_data_size, int timeout) = 0;
    };
}

#endif
