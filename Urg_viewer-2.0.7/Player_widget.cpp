/*!
  \file
  \brief データの再生用 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include "Player_widget.h"
#include "counter_utils.h"


struct Player_widget::pImpl
{
    Player_widget* widget_;
    int last_set_value_;


    pImpl(Player_widget* widget)
        : widget_(widget), last_set_value_(0)
    {
    }


    void initialize_form(void)
    {
        // signals
        connect(widget_->folder_button_, SIGNAL(clicked()),
                widget_, SIGNAL(folder_clicked()));
        connect(widget_->play_button_, SIGNAL(clicked()),
                widget_, SIGNAL(play_clicked()));
        connect(widget_->stop_button_, SIGNAL(clicked()),
                widget_, SIGNAL(stop_clicked()));
        connect(widget_->pause_button_, SIGNAL(clicked()),
                widget_, SIGNAL(pause_clicked()));
        connect(widget_->progress_slider_, SIGNAL(valueChanged(int)),
                widget_, SLOT(slider_value_changed(int)));
    }


    void set_detail_visible(bool visible)
    {
        widget_->play_button_->setVisible(visible);
        widget_->pause_button_->setVisible(visible);
        widget_->stop_button_->setVisible(visible);
        widget_->progress_slider_->setVisible(visible);
        widget_->current_label_->setVisible(visible);
        widget_->slash_label_->setVisible(visible);
        widget_->total_label_->setVisible(visible);
    }


    void set_visible(bool visible)
    {
        widget_->setVisible(visible);
    }


    void set_folder_enabled(bool enable)
    {
        widget_->folder_button_->setEnabled(enable);
    }


    void set_state(state_t state)
    {
        bool is_detail_enable = false;
        bool is_hide = false;
        bool is_folder_enable = false;

        switch (state) {
        case Playing:
            is_detail_enable = true;
            is_folder_enable = true;
            break;

        case Recording:
            is_hide = true;
            break;

        case Not_connected:
            is_folder_enable = true;
            break;

        case Viewing:
            is_folder_enable = true;
            break;
        }

        set_folder_enabled(is_folder_enable);
        set_detail_visible(is_detail_enable);
        set_visible(!is_hide);
    }
};


Player_widget::Player_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
}


Player_widget::~Player_widget(void)
{
}


void Player_widget::set_state(state_t state)
{
    pimpl->set_state(state);
}


void Player_widget::set_focus(button_t button)
{
    switch (button) {
    case Play:
        pause_button_->hide();
        play_button_->show();
        play_button_->setFocus();
        break;

    case Pause:
        play_button_->hide();
        pause_button_->show();
        pause_button_->setFocus();
        break;

    case Stop:
        stop_button_->setFocus();
        break;
    }
}


void Player_widget::set_pause_enabled(bool enable)
{
    pause_button_->setEnabled(enable);
}


void Player_widget::set_folder_enabled(bool enable)
{
    folder_button_->setEnabled(enable);
}


void Player_widget::set_total_play_time(long total_second, long progress_size)
{
    total_label_->setText(counter_text(total_second));
    progress_slider_->setMaximum(progress_size - 1);
}


int Player_widget::slider_value(void) const
{
    return progress_slider_->value();
}


bool Player_widget::is_folder_enabled(void) const
{
    return folder_button_->isEnabled();
}


void Player_widget::set_play_time(long second, long progress_index)
{
    current_label_->setText(counter_text(second));
    pimpl->last_set_value_ = progress_index;
    progress_slider_->setValue(progress_index);
}


void Player_widget::slider_value_changed(int value)
{
    if (pimpl->last_set_value_ == value) {
        return;
    }

    // 手動による操作のときのみ、シグナルを発行する
    emit slider_changed(value);
}
