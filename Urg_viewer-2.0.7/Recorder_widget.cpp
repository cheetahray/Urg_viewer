/*!
  \file
  \brief データの記録用 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <QTime>
#include <QTimer>
#include "Recorder_widget.h"
#include "counter_utils.h"


struct Recorder_widget::pImpl
{
    Recorder_widget* widget_;
    QTime record_timer_;
    QTimer update_timer_;


    pImpl(Recorder_widget* widget)
        : widget_(widget)
    {
        update_timer_.setInterval(1000);
    }


    void initialize_form(void)
    {
        // signals
        connect(widget_->record_button_, SIGNAL(clicked()),
                widget_, SLOT(record_button_clicked()));
        connect(widget_->stop_button_, SIGNAL(clicked()),
                widget_, SLOT(stop_button_clicked()));
        connect(&update_timer_, SIGNAL(timeout()),
                widget_, SLOT(update_timer_timeout()));
    }


    void set_detail_visible(bool enable)
    {
        widget_->stop_button_->setVisible(enable);
        widget_->progress_label_->setVisible(enable);
    }


    void set_state(state_t state)
    {
        bool is_detail_enable = false;
        bool is_record_enable = false;
        bool is_stop_enable = false;
        bool is_visible = true;

        switch (state) {
        case Recording:
            is_detail_enable = true;
            is_stop_enable = true;
            break;

        case Viewing:
            is_record_enable = true;
            break;

        case Playing:
            is_visible = false;
            break;

        case Not_connected:
            is_visible = false;
            break;
        }

        set_detail_visible(is_detail_enable);
        widget_->record_button_->setEnabled(is_record_enable);
        widget_->stop_button_->setEnabled(is_stop_enable);
        widget_->setVisible(is_visible);
    }


    void update_counter(void)
    {
        QString text = counter_text(record_timer_.elapsed() / 1000);
        widget_->progress_label_->setText(text);
    }
};


Recorder_widget::Recorder_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
}


Recorder_widget::~Recorder_widget(void)
{
}


void Recorder_widget::set_state(state_t state)
{
    pimpl->set_state(state);
}


void Recorder_widget::start_recording(void)
{
    record_button_clicked();
}


void Recorder_widget::stop_recording(void)
{
    pimpl->update_timer_.stop();
}


void Recorder_widget::setFocus(void)
{
    stop_button_->setFocus();
}


void Recorder_widget::record_button_clicked(void)
{
    // カウントを開始する
    pimpl->record_timer_.start();
    pimpl->update_timer_.start();

    emit record_clicked();
}


void Recorder_widget::stop_button_clicked(void)
{
    stop_recording();
    emit stop_clicked();
}


void Recorder_widget::update_timer_timeout(void)
{
    pimpl->update_counter();
}
