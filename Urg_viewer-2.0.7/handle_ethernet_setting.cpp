/*!
  \file
  \brief Ethernet_connection_widget 設定の管理

  \author Satofumi KAMIMURA

  $Id: connection_setting.cpp,v d887121d7c3a 2011/04/28 17:25:42 satofumi $
*/

#include <string>
#include <QSettings>
#include <QFile>
#include <QTextStream>
#include <QCompleter>
#include <QLineEdit>
#include "handle_ethernet_setting.h"
#include "Ethernet_connection_widget.h"

using namespace hrk;
using namespace std;


namespace
{
    const char* Default_ip_address = "192.168.0.10";
}


void hrk::load_ethernet_setting(QSettings& settings,
                                Ethernet_connection_widget* widget)
{
    string ip_address =
        settings.value("connection_ip_address",
                       Default_ip_address).toString().toStdString();
    if (ip_address.empty()) {
        ip_address = Default_ip_address;
    }
    widget->set_address(ip_address.c_str());
}


void hrk::save_ethernet_setting(QSettings& settings,
                                const Ethernet_connection_widget* widget)
{
    if (!widget->address().empty()) {
        settings.setValue("connection_ip_address",
                          widget->address().c_str());
    }
}


QStringList hrk::load_complete_address(const char* completer_file)
{
    QStringList completer_address;

    // 入力候補を保存したファイルがあれば、読み出して入力候補として使う
    QFile file(completer_file);
    if (!file.open(QFile::ReadOnly)) {
        return completer_address;
    }

    QTextStream fin(&file);
    while (true) {
        QString line = fin.readLine();
        if (line.isNull()) {
            break;
        }
        completer_address << line;
    }

    return completer_address;
}


bool hrk::save_complete_address(const char* completer_file,
                                const QStringList& completer_address)
{
    QFile file(completer_file);
    if (! file.open(QFile::WriteOnly | QFile::Truncate)) {
        return false;
    }

    QTextStream fout(&file);
    size_t n = completer_address.size();
    for (size_t i = 0; i < n; ++i) {
        fout << completer_address.at(i) << endl;
    }

    return true;
}
