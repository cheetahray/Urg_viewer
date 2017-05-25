#ifndef HRK_SERIAL_CONNECTION_WIDGET_H
#define HRK_SERIAL_CONNECTION_WIDGET_H

/*!
  \file
  \brief シリアル接続用の Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <string>
#include <vector>
#include "ui_Serial_connection_widget_form.h"


namespace hrk
{
    class Serial_connection_widget
        : public QWidget, private Ui::Serial_connection_widget_form
    {
        Q_OBJECT;

    public:
        Serial_connection_widget(QWidget* parent = NULL);
        ~Serial_connection_widget(void);

        void set_device_names(const std::vector<std::string>& device_names);
        void set_baudrate_values(const std::vector<long>& baudrates);
        void set_baudrate_visible(bool visible);

        void setFocus(void);
        void set_connected(bool connected = true);

    signals:
        void connect_clicked(const std::string& device, long baudrate);
        void disconnect_clicked(void);
        void rescan_clicked(void);

    private slots:
        void connect_button_clicked(void);
        void rescan_button_clicked(void);

    private:
        Serial_connection_widget(const Serial_connection_widget& rhs);
        Serial_connection_widget&
        operator = (const Serial_connection_widget& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
