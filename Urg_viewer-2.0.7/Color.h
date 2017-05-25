#ifndef HRK_COLOR_H
#define HRK_COLOR_H

/*!
  \file
  \brief 色

  \author Satofumi Kamimura

  $Id$
*/

namespace hrk
{
    //! 色
    class Color
    {
    public:
        /*!
          \param[in] red 赤 [0.0, 1.0]
          \param[in] green 緑 [0.0, 1.0]
          \param[in] blue 青 [0.0, 1.0]
          \param[in] alpha 透明値 [0.0, 1.0]
        */
        explicit Color(double red, double green, double blue,
                       double alpha = 1.0);

        explicit Color(void);

        //! 赤の値を設定する
        void set_red(double red);

        //! 赤の値を取得する
        double red(void) const;

        //! 緑の値を設定する
        void set_green(double red);

        //! 緑の値を取得する
        double green(void) const;

        //! 青の値を設定する
        void set_blue(double red);

        //! 青の値を取得する
        double blue(void) const;

        //! 透明値を設定する
        void set_alpha(double red);

        //! 透明値を取得する
        double alpha(void) const;

    private:
        double red_;
        double green_;
        double blue_;
        double alpha_;
    };
}

#endif
