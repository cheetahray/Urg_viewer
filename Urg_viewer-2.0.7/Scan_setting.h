#ifndef SCAN_SETTING_H
#define SCAN_SETTING_H

/*!
  \file
  \brief 計測の設定

  \author Satofumi Kamimura

  $Id$
*/

#include "Lidar.h"


class Scan_setting
{
 public:
    int first_step;
    int last_step;
    int group_steps;
    bool with_intensity;
    bool is_multiecho;
};

#endif
