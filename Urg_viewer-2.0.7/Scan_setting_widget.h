#ifndef SCAN_SETTING_WIDGET_H
#define SCAN_SETTING_WIDGET_H

/*!
  \file
  \brief 計測の設定 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "ui_Scan_setting_widget_form.h"

class Scan_setting;

namespace hrk
{
    class Lidar;
}


class Scan_setting_widget
    : public QWidget, private Ui::Scan_setting_widget_form
{
    Q_OBJECT;

 public:
    Scan_setting_widget(QWidget* parent = NULL);
    ~Scan_setting_widget(void);

    void set_control_enabled(bool enable);
    bool load_sensor_setting(hrk::Lidar& lidar);

 signals:
    void close_nortify(void);
    void quit_nortify(void);
    void scan_setting_updated(const Scan_setting& setting, int scan_interval);

 private slots:
    void closeEvent(QCloseEvent* event);
    void quit(void);
    void default_clicked(void);
    void setting_changed(void);
    void apply_clicked(void);

 private:
    Scan_setting_widget(const Scan_setting_widget& rhs);
    Scan_setting_widget& operator = (const Scan_setting_widget& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
