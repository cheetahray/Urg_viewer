/*!
  \file
  \brief カウンタ用の補助関数

  \author Satofumi Kamimura

  $Id$
*/

#include "counter_utils.h"


QString counter_text(long left_second)
{
    int second = left_second % 60;
    int left_minute = left_second / 60;
    int minute = left_minute % 60;
    int hour = left_minute / 60;

    QChar zero('0');
    QString text = QString("%1:%2:%3").
        arg(hour, 2, 10, zero).
        arg(minute, 2, 10, zero).
        arg(second, 2, 10, zero);

    return text;
}
