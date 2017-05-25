/*!
  \file
  \brief Ethernet 接続用の Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <QComboBox>
#include <QCompleter>
#include "Ethernet_connection_widget.h"

using namespace hrk;
using namespace std;


enum {
    Default_baudrate = 115200,
};


struct Ethernet_connection_widget::pImpl
{
    Ethernet_connection_widget* widget_;
    long baudrate_;


    pImpl(Ethernet_connection_widget* widget)
        : widget_(widget), baudrate_(Default_baudrate)
    {
    }


    void initialize_form(void)
    {
        set_disable_if_address_empty();
        widget_->port_spinbox_->setVisible(false);

        // signals
        connect(widget_->connect_button_, SIGNAL(clicked()),
                widget_, SLOT(connect_button_clicked()));
        connect(widget_->address_edit_, SIGNAL(textEdited(const QString&)),
                widget_, SLOT(address_edited()));
        connect(widget_->address_edit_, SIGNAL(returnPressed()),
                widget_, SLOT(enter_pressed()));
    }


    void set_address(const string& ip_address)
    {
        if (ip_address.empty()) {
            return;
        }

        widget_->address_edit_->setText(ip_address.c_str());
        widget_->connect_button_->setEnabled(true);
    }


    bool set_disable_if_address_empty(void)
    {
        QString address = widget_->address_edit_->text().trimmed();
        if (address.isEmpty()) {
            widget_->connect_button_->setEnabled(false);
            return false;
        } else {
            return true;
        }
    }


    void set_edit_enabled(bool enable)
    {
        widget_->address_edit_->setEnabled(enable);
        widget_->port_spinbox_->setEnabled(enable);
    }


    void set_connect_enabled(bool enable)
    {
        widget_->connect_button_->setEnabled(enable);
        if (enable) {
            widget_->connect_button_->setFocus();
        }
    }
};


Ethernet_connection_widget::Ethernet_connection_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
}


Ethernet_connection_widget::~Ethernet_connection_widget(void)
{
}


void Ethernet_connection_widget::set_completer_address(const QStringList&
                                                       completer_address)
{
    QCompleter* completer = new QCompleter(completer_address);
    address_edit_->setCompleter(completer);
}


void Ethernet_connection_widget::set_address(const string& ip_address)
{
    pimpl->set_address(ip_address);
}


std::string Ethernet_connection_widget::address(void) const
{
    return address_edit_->text().toStdString();
}


void Ethernet_connection_widget::set_port_number(long port_number)
{
    port_spinbox_->setValue(port_number);
}


void Ethernet_connection_widget::set_port_visible(bool visible)
{
    port_spinbox_->setVisible(visible);
}


void Ethernet_connection_widget::setFocus(void)
{
    if (pimpl->set_disable_if_address_empty()) {
        connect_button_->setFocus();
    } else {
        address_edit_->setFocus();
    }
}


void Ethernet_connection_widget::set_connected(bool connected)
{
    pimpl->set_connect_enabled(true);

    if (connected) {
        connect_button_->setText(tr("Disconnect"));
    } else {
        connect_button_->setText(tr("Connect"));
    }
    connect_button_->setChecked(connected);
    pimpl->set_edit_enabled(!connected);
}


void Ethernet_connection_widget::connect_button_clicked(void)
{
    connect_button_->setFocus();

    pimpl->set_connect_enabled(false);
    bool is_checked = connect_button_->isChecked();

    pimpl->set_edit_enabled(!is_checked);
    if (is_checked) {
        const string address = address_edit_->text().trimmed().toStdString();
        long port = port_spinbox_->value();
        emit connect_clicked(address, port);
    } else {
        emit disconnect_clicked();
    }
}


void Ethernet_connection_widget::address_edited(void)
{
    if (!pimpl->set_disable_if_address_empty()) {
        return;
    }

    connect_button_->setEnabled(true);
}


void Ethernet_connection_widget::enter_pressed(void)
{
    if (!pimpl->set_disable_if_address_empty()) {
        return;
    }

    connect_button_->setChecked(true);
    connect_button_clicked();
}
