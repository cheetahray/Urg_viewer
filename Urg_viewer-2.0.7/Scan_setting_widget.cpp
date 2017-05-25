/*!
  \file
  \brief 計測の設定 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <QShortcut>
#include <QCloseEvent>
#include "Scan_setting_widget.h"
#include "Preview_widget.h"
#include "Scan_setting.h"

using namespace hrk;


struct Scan_setting_widget::pImpl
{
    Scan_setting_widget* widget_;
    Preview_widget preview_widget_;
    Lidar* lidar_;
    bool use_default_;

    Scan_setting setting_;
    int scan_interval_;
    int front_index_;
    int total_steps_;


    pImpl(Scan_setting_widget* widget)
        : widget_(widget), lidar_(NULL), use_default_(true), scan_interval_(0)
    {
    }


    void initialize_form(void)
    {
        QVBoxLayout* box = new QVBoxLayout();
        box->addWidget(&preview_widget_);
        widget_->preview_groupbox_->setLayout(box);

        // signals
        connect(widget_->close_button_, SIGNAL(clicked()),
                widget_, SLOT(close()));
        connect(widget_->default_button_, SIGNAL(clicked()),
                widget_, SLOT(default_clicked()));
        connect(widget_->range_first_spinbox_, SIGNAL(valueChanged(int)),
                widget_, SLOT(setting_changed()));
        connect(widget_->range_last_spinbox_, SIGNAL(valueChanged(int)),
                widget_, SLOT(setting_changed()));
        connect(widget_->grouping_spinbox_, SIGNAL(valueChanged(int)),
                widget_, SLOT(setting_changed()));
        connect(widget_->interval_spinbox_, SIGNAL(valueChanged(int)),
                widget_, SLOT(setting_changed()));
        connect(widget_->apply_button_, SIGNAL(clicked()),
                widget_, SLOT(apply_clicked()));

        // short cut
        new QShortcut(Qt::CTRL + Qt::Key_W, widget_, SLOT(close()));
        new QShortcut(Qt::CTRL + Qt::Key_Q, widget_, SLOT(quit()));
    }


    bool load_default_setting(void)
    {
        int min_step = lidar_->min_step();
        int max_step = lidar_->max_step();

        widget_->range_first_spinbox_->setValue(min_step);
        widget_->range_last_spinbox_->setValue(max_step);
        widget_->range_first_spinbox_->setRange(min_step, max_step);
        widget_->range_last_spinbox_->setRange(min_step, max_step);

        enum {
            Default_grouping_steps = 1,
            Default_scan_interval = 0,
        };
        widget_->grouping_spinbox_->setValue(Default_grouping_steps);
        widget_->interval_spinbox_->setValue(Default_scan_interval);

        front_index_ = lidar_->front_step();

        // １周分のステップ数の計算
        total_steps_ = (lidar_->deg2step(+45) - lidar_->deg2step(-45)) * 4;

        return true;
    }


    void apply_sensor_setting(void)
    {
        // 新しい設定での計測を指示する
        widget_->emit scan_setting_updated(setting_, scan_interval_);
    }


    void setting_changed(void)
    {
        use_default_ = false;

        setting_.first_step = widget_->range_first_spinbox_->value();
        setting_.last_step = widget_->range_last_spinbox_->value();
        setting_.group_steps = widget_->grouping_spinbox_->value();
        scan_interval_ = widget_->interval_spinbox_->value();

        update_preview_widget();
    }


    void update_preview_widget(void)
    {
        preview_widget_.set_setting(setting_, scan_interval_, front_index_,
                                    total_steps_);
    }
};


Scan_setting_widget::Scan_setting_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
}


Scan_setting_widget::~Scan_setting_widget(void)
{
}


void Scan_setting_widget::set_control_enabled(bool enable)
{
    default_button_->setEnabled(enable);
    range_first_spinbox_->setEnabled(enable);
    range_last_spinbox_->setEnabled(enable);
    grouping_spinbox_->setEnabled(enable);
    interval_spinbox_->setEnabled(enable);
    apply_button_->setEnabled(enable);

    if (!enable) {
        pimpl->preview_widget_.clear_setting();
    }
}


bool Scan_setting_widget::load_sensor_setting(hrk::Lidar& lidar)
{
    // デフォルト設定を使わない場合は、読み込みを行わない
    if (!pimpl->use_default_) {
        pimpl->update_preview_widget();
        return false;
    }

    pimpl->lidar_ = &lidar;
    if (!pimpl->load_default_setting()) {
        return false;
    }

    pimpl->update_preview_widget();
    pimpl->use_default_ = true;
    return true;
}


void Scan_setting_widget::closeEvent(QCloseEvent* event)
{
    emit close_nortify();
    event->accept();
}


void Scan_setting_widget::quit(void)
{
    emit quit_nortify();
}


void Scan_setting_widget::default_clicked(void)
{
    pimpl->load_default_setting();
    pimpl->use_default_ = true;
}


void Scan_setting_widget::setting_changed(void)
{
    pimpl->setting_changed();
}


void Scan_setting_widget::apply_clicked(void)
{
    pimpl->apply_sensor_setting();
}
