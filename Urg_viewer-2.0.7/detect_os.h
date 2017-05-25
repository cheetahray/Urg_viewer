#ifndef HRK_DETECT_OS_H
#define HRK_DETECT_OS_H

/*!
  \file
  \brief OS の検出

  \author Satofumi Kamimura

  $Id$
*/

#if defined _MSC_VER || defined __CYGWIN__ || defined __MINGW32__
#define WINDOWS_OS

#if defined _MSC_VER
#define VISUAL_CPP
#elif defined __CYGWIN__
#define CYGWIN_GCC
#elif defined __MINGW32__
#define MINGW_GCC
#endif

#elif defined __linux__
#define LINUX_OS

#else
// 検出できないときは Mac として扱う
#define MAC_OS
#endif

#endif
