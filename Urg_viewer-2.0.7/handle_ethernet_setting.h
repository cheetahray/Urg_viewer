#ifndef HRK_HANDLE_ETHERNET_SETTING_H
#define HRK_HANDLE_ETHERNET_SETTING_H

/*!
  \file
  \brief Ethernet_connection_widget 設定の管理

  \author Satofumi KAMIMURA

  $Id$
*/

class QSettings;
class QTextEdit;


namespace hrk
{
    class Ethernet_connection_widget;


    extern void load_ethernet_setting(QSettings& settings,
                                      Ethernet_connection_widget* widget);
    extern void save_ethernet_setting(QSettings& settings,
                                      const Ethernet_connection_widget* widget);
    extern QStringList load_complete_address(const char* completer_file);
    extern bool save_complete_address(const char* completer_file,
                                      const QStringList& completer_address);
}

#endif
