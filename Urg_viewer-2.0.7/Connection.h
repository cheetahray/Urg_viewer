#ifndef HRK_CONNECTION_H
#define HRK_CONNECTION_H

/*!
  \file
  \brief 接続のインターフェース

  \author Satofumi Kamimura

  $Id$

  \todo timeout を double 型にする。
*/

#include "Stream.h"


namespace hrk
{
    /*!
      \brief 接続のインターフェース

      アクセス用メソッドを定義する。ただし open() は扱うデバイスによって引数の数が変わる可能性があるため、この実装を行うインターフェースが提供ことにする。
    */
    class Connection : public Stream
    {
    public:
        enum {
            Timeout_infinity = -1, //!< read() で無限に受信を待つ場合に利用する
        };


        virtual ~Connection(void)
        {
        }

        /*!
          \brief 状態を示すメッセージを返す

          \return メッセージ文字列
        */
        virtual const char* what(void) const = 0;

        /*!
          \brief ボーレートの変更

          \param[in] baudrate ボーレート [bps]

          \retval true 成功
          \retval false エラー
        */
        virtual bool change_baudrate(long baudrate) = 0;

        /*!
          \brief １文字だけ受信バッファに書き戻す
        */
        virtual void ungetc(int ch) = 0;
    };
}

#endif
