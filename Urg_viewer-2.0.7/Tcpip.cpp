/*!
  \file
  \brief TCP/IP 通信

  \author Satofumi Kamimura

  $Id$
*/

#include "detect_os.h"


#if defined(WINDOWS_OS)
#include "Tcpip_windows.cpp"
#else
#include "Tcpip_linux.cpp"
#endif
