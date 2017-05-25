/*!
  \file
  \brief RS-232 シリアル通信

  \author Satofumi Kamimura

  $Id$
*/

#include "detect_os.h"


#if defined(WINDOWS_OS)
#include "Serial_windows.cpp"
#else
#include "Serial_linux.cpp"
#endif
