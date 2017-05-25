/*!
  \file
  \brief 色

  \author Satofumi Kamimura

  $Id$
*/

#include "Color.h"

using namespace hrk;


Color::Color(double red, double green, double blue, double alpha)
    : red_(red), green_(green), blue_(blue), alpha_(alpha)
{
}


Color::Color(void)
    : red_(1.0), green_(1.0), blue_(1.0), alpha_(1.0)
{
}


void Color::set_red(double red)
{
    red_ = red;
}


double Color::red(void) const
{
    return red_;
}


void Color::set_green(double green)
{
    green_ = green;
}


double Color::green(void) const
{
    return green_;
}


void Color::set_blue(double blue)
{
    blue_ = blue;
}


double Color::blue(void) const
{
    return blue_;
}


void Color::set_alpha(double alpha)
{
    alpha_ = alpha;
}


double Color::alpha(void) const
{
    return alpha_;
}
