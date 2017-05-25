#ifndef HRK_CONVERT_PATH_CODEC_H
#define HRK_CONVERT_PATH_CODEC_H

/*!
  \file
  \brief 文字コードの変換

  \author Satofumi Kamimura

  $Id$
*/

#include <QString>
#include <string>


namespace hrk
{
    extern const std::string std_string_path(const QString& path);
    extern const QString qt_qstring_path(const std::string& path);
}

#endif
