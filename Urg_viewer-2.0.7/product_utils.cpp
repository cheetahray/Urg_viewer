/*!
  \file
  \brief 製品毎の対処用の補助関数

  \author Satofumi Kamimura

  $Id$
*/

#include "product_utils.h"
#include "Urg_driver.h"

using namespace hrk;
using namespace std;


double product_timestamp_unit(const hrk::Urg_driver& urg)
{
    const string product_type = urg.sensor_product_type();
    double timestamp_unit;
    if ((product_type == "UJM-30LX-EA1") || (product_type == "UJM-30LX-EA0")) {
        timestamp_unit = 1000.0;
    } else {
        timestamp_unit = 1.0;
    }
    return timestamp_unit;
}
