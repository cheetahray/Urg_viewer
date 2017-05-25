#ifndef PLAYER_WIDGET_H
#define PLAYER_WIDGET_H

/*!
  \file
  \brief データの再生用 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "ui_Player_widget_form.h"
#include "State.h"


class Player_widget
    : public QWidget, public State, private Ui::Player_widget_form
{
    Q_OBJECT;

 public:
    typedef enum {
        Play,
        Pause,
        Stop,
    } button_t;

    Player_widget(QWidget* parent = NULL);
    ~Player_widget(void);

    void set_state(state_t state);
    void set_focus(button_t button);
    void set_pause_enabled(bool enable);
    void set_folder_enabled(bool enable);
    void set_total_play_time(long total_second, long progress_size);
    int slider_value(void) const;
    bool is_folder_enabled(void) const;

 public slots:
    void set_play_time(long second, long progress_index);

 signals:
    void folder_clicked(void);
    void play_clicked(void);
    void stop_clicked(void);
    void pause_clicked(void);
    void slider_changed(int index);

 private slots:
    void slider_value_changed(int value);

 private:
    Player_widget(const Player_widget& rhs);
    Player_widget& operator = (const Player_widget& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
