#ifndef PRODUCT_UTILS_H
#define PRODUCT_UTILS_H

/*!
  \file
  \brief 製品毎の対処用の補助関数

  \author Satofumi Kamimura

  $Id$
*/


namespace hrk
{
    class Urg_driver;
}

extern double product_timestamp_unit(const hrk::Urg_driver& urg);

#endif
