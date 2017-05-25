/*!
  \file
  \brief シリアル接続用の Widget

  \author Satofumi Kamimura

  $Id$
*/

#include "Serial_connection_widget.h"

using namespace hrk;
using namespace std;


struct Serial_connection_widget::pImpl
{
    Serial_connection_widget* widget_;


    pImpl(Serial_connection_widget* widget)
        : widget_(widget)
    {
    }


    void initialize_form(void)
    {
        widget_->devices_combobox_->
            setSizeAdjustPolicy(QComboBox::AdjustToContents);

        QStringList baudrates;
        baudrates << "115200"
                  << "38400"
                  << "19200";
        widget_->baudrates_combobox_->addItems(baudrates);
        widget_->set_baudrate_visible(false);

        set_empty_device();

        // signals
        connect(widget_->connect_button_, SIGNAL(clicked()),
                widget_, SLOT(connect_button_clicked()));
        connect(widget_->rescan_button_, SIGNAL(clicked()),
                widget_, SLOT(rescan_button_clicked()));
    }


    void set_empty_device(void)
    {
        QComboBox* combobox = widget_->devices_combobox_;
        combobox->clear();

        combobox->addItem(tr("- press rescan -"));
        set_edit_enabled(false, false);
        widget_->connect_button_->setEnabled(false);
    }


    void set_devices(const vector<string>& device_names)
    {
        QComboBox* combobox = widget_->devices_combobox_;
        combobox->clear();

        for (vector<string>::const_iterator it = device_names.begin();
             it != device_names.end(); ++it) {
            combobox->addItem(it->c_str());
        }

        set_edit_enabled(true, false);
        widget_->connect_button_->setEnabled(true);
    }


    void set_edit_enabled(bool enable, bool with_rescan_button = false)
    {
        widget_->devices_combobox_->setEnabled(enable);
        widget_->baudrates_combobox_->setEnabled(enable);
        if (with_rescan_button) {
            widget_->rescan_button_->setEnabled(enable);
        }
    }


    void set_connect_enabled(bool enable)
    {
        widget_->connect_button_->setEnabled(enable);
        if (enable) {
            widget_->connect_button_->setFocus();
        }
    }
};


Serial_connection_widget::Serial_connection_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
}


Serial_connection_widget::~Serial_connection_widget(void)
{
}


void Serial_connection_widget::set_device_names(const std::vector<std::string>&
                                                device_names)
{
    if (device_names.empty()) {
        pimpl->set_empty_device();
        return;
    }

    pimpl->set_devices(device_names);
}


void Serial_connection_widget::set_baudrate_values(const std::vector<long>&
                                                   baudrates)
{
    if (baudrates.empty()) {
        return;
    }

    baudrates_combobox_->clear();
    for (vector<long>::const_iterator it = baudrates.begin();
         it != baudrates.end(); ++it) {
        baudrates_combobox_->addItem(QString("%1").arg(*it));
    }
}


void Serial_connection_widget::set_baudrate_visible(bool visible)
{
    baudrates_combobox_->setVisible(visible);
}


void Serial_connection_widget::setFocus(void)
{
    if (connect_button_->isEnabled()) {
        connect_button_->setFocus();
    } else {
        rescan_button_->setFocus();
    }
}


void Serial_connection_widget::set_connected(bool connected)
{
    if (connected) {
        connect_button_->setText(tr("Disconnect"));
    } else {
        connect_button_->setText(tr("Connect"));
    }
    connect_button_->setChecked(connected);
    pimpl->set_edit_enabled(!connected, true);

    pimpl->set_connect_enabled(true);
}


void Serial_connection_widget::connect_button_clicked(void)
{
    pimpl->set_connect_enabled(false);
    bool is_checked = connect_button_->isChecked();

    pimpl->set_edit_enabled(!is_checked);
    if (is_checked) {
        const string device = devices_combobox_->currentText().toStdString();
        long baudrate = atol(baudrates_combobox_->
                             currentText().toStdString().c_str());
        emit connect_clicked(device, baudrate);
    } else {
        emit disconnect_clicked();
    }
}


void Serial_connection_widget::rescan_button_clicked(void)
{
    emit rescan_clicked();
    connect_button_->setFocus();
}
