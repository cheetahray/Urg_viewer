#ifndef RECORDER_WIDGET_H
#define RECORDER_WIDGET_H

/*!
  \file
  \brief データの記録用 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "ui_Recorder_widget_form.h"
#include "State.h"


class Recorder_widget
    : public QWidget, public State, private Ui::Recorder_widget_form
{
    Q_OBJECT;

 public:
    Recorder_widget(QWidget* parent = NULL);
    ~Recorder_widget(void);

    void set_state(state_t state);
    void start_recording(void);
    void stop_recording(void);
    void setFocus(void);

 signals:
    void record_clicked(void);
    void stop_clicked(void);

 private slots:
    void record_button_clicked(void);
    void stop_button_clicked(void);
    void update_timer_timeout(void);

 private:
    Recorder_widget(const Recorder_widget& rhs);
    Recorder_widget& operator = (const Recorder_widget& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
