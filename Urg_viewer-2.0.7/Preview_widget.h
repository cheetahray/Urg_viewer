#ifndef PREVIEW_WIDGET_H
#define PREVIEW_WIDGET_H

/*!
  \file
  \brief プレビュー表示

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <QWidget>

class Scan_setting;


class Preview_widget : public QWidget
{
    Q_OBJECT;

 public:
    Preview_widget(QWidget* widget = NULL);
    ~Preview_widget(void);

    void clear_setting(void);
    void set_setting(const Scan_setting& setting, int scan_interval,
                     int front_index, int total_steps);

 protected:
    void paintEvent(QPaintEvent* event);

 private:
    Preview_widget(const Preview_widget& rhs);
    Preview_widget& operator = (const Preview_widget& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
