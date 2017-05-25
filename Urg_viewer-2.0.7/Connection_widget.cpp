/*!
  \file
  \brief Serial と Ethernet の Connection_widget のどちらかを選択する Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <QBoxLayout>
#include "Connection_widget.h"
#include "Serial_connection_widget.h"
#include "Ethernet_connection_widget.h"

using namespace hrk;


struct Connection_widget::pImpl
{
    Connection_widget* widget_;
    Serial_connection_widget serial_connection_widget_;
    Ethernet_connection_widget ethernet_connection_widget_;
    connection_t current_connection_;


    pImpl(Connection_widget* widget)
        : widget_(widget), current_connection_(Serial)
    {
    }


    void initialize_form(void)
    {
        // layout
        QLayout* layout = new QBoxLayout(QBoxLayout::LeftToRight);
	layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(&serial_connection_widget_);
        layout->addWidget(&ethernet_connection_widget_);
        widget_->setLayout(layout);
    }
};


Connection_widget::Connection_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    pimpl->initialize_form();
    set_connection_type(Serial);
}


Connection_widget::~Connection_widget(void)
{
}


void Connection_widget::set_connection_type(connection_t type)
{
    switch (type) {
    case Serial:
        pimpl->ethernet_connection_widget_.hide();
        pimpl->serial_connection_widget_.show();
        break;

    case Ethernet:
        pimpl->serial_connection_widget_.hide();
        pimpl->ethernet_connection_widget_.show();
        break;
    }

    pimpl->current_connection_ = type;
}


Connection_widget::connection_t Connection_widget::connection_type(void) const
{
    return pimpl->current_connection_;
}


void Connection_widget::toggle_connection_type(void)
{
    switch (pimpl->current_connection_) {
    case Serial:
        set_connection_type(Ethernet);
        break;

    case Ethernet:
        set_connection_type(Serial);
        break;
    }
}


void Connection_widget::set_connected(bool connected)
{
    switch (pimpl->current_connection_) {
    case Serial:
        pimpl->serial_connection_widget_.set_connected(connected);
        break;

    case Ethernet:
        pimpl->ethernet_connection_widget_.set_connected(connected);
        break;
    }
}


void Connection_widget::setFocus(void)
{
    switch (pimpl->current_connection_) {
    case Serial:
        pimpl->serial_connection_widget_.setFocus();
        break;

    case Ethernet:
        pimpl->ethernet_connection_widget_.setFocus();
        break;
    }
}


Serial_connection_widget* Connection_widget::serial(void)
{
    return &pimpl->serial_connection_widget_;
}


Ethernet_connection_widget* Connection_widget::ethernet(void)
{
    return &pimpl->ethernet_connection_widget_;
}
