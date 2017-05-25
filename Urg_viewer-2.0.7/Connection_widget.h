#ifndef HRK_CONNECTION_WIDGET_H
#define HRK_CONNECTION_WIDGET_H

/*!
  \file
  \brief Serial と Ethernet の Connection_widget のどちらかを選択する Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <QWidget>


namespace hrk
{
    class Serial_connection_widget;
    class Ethernet_connection_widget;


    class Connection_widget : public QWidget
    {
        Q_OBJECT;

    public:
        typedef enum {
            Serial,
            Ethernet,
        } connection_t;

        Connection_widget(QWidget* parent = NULL);
        ~Connection_widget(void);

        void set_connection_type(connection_t type);
        connection_t connection_type(void) const;
        void toggle_connection_type(void);
        void set_connected(bool connected);
        void setFocus(void);

        Serial_connection_widget* serial(void);
        Ethernet_connection_widget* ethernet(void);

    private:
        Connection_widget(const Connection_widget& rhs);
        Connection_widget& operator = (const Connection_widget& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
