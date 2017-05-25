#ifndef HRK_ETHERNET_CONNECTION_WIDGET_H
#define HRK_ETHERNET_CONNECTION_WIDGET_H

/*!
  \file
  \brief Ethernet 接続用の Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <string>
#include <vector>
#include "ui_Ethernet_connection_widget_form.h"


namespace hrk
{
    class Ethernet_connection_widget
        : public QWidget, private Ui::Ethernet_connection_widget_form
    {
        Q_OBJECT;

    public:
        Ethernet_connection_widget(QWidget* parent = NULL);
        ~Ethernet_connection_widget(void);

        void set_completer_address(const QStringList& completer_address);

        void set_address(const std::string& ip_address);
        std::string address(void) const;
        void set_port_number(long port_number);
        void set_port_visible(bool visible);

        void setFocus(void);
        void set_connected(bool connected = true);

    signals:
        void connect_clicked(const std::string& device, long baudrate);
        void disconnect_clicked(void);

    private slots:
        void connect_button_clicked(void);
        void address_edited(void);
        void enter_pressed(void);

    private:
        Ethernet_connection_widget(const Ethernet_connection_widget& rhs);
        Ethernet_connection_widget&
        operator = (const Ethernet_connection_widget& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
