/*!
  \file
  \brief プレビュー表示

  \author Satofumi Kamimura

  $Id$
*/

#include <cmath>
#include <QPainter>
#include "Preview_widget.h"
#include "Scan_setting.h"

using namespace std;


namespace
{
    enum {
        Minimum_length = 250,
    };
}


struct Preview_widget::pImpl
{
    Preview_widget* widget_;
    bool is_parameter_loaded_;

    Scan_setting setting_;
    int scan_interval_;
    int front_index_;
    int total_steps_;


    pImpl(Preview_widget* widget)
        : widget_(widget), is_parameter_loaded_(false)
    {
    }


    void initialize_form(void)
    {
        widget_->setMinimumSize(Minimum_length, Minimum_length);
    }


    void draw_preview_image(QPainter& painter)
    {
        // X, Y 軸の描画
        const QRect this_rect = widget_->rect();
        const QPoint center(this_rect.width() / 2, this_rect.height() / 2);
        const int offset = 4;
        painter.setPen(QColor(Qt::black));
        painter.drawLine(offset, center.y(),
                         this_rect.width() - offset, center.y());
        painter.drawLine(center.x(), offset,
                         center.x(), this_rect.height() - offset);

        // パラメータ表示用のフォント作成
        enum { FontSize = 12 };
        QFont font("times", FontSize);
        QFontMetrics fm(font);
        size_t text_height = fm.height();
        size_t text_width = fm.width("-XXX[deg]");

        // 円の描画
        painter.setPen(QColor(Qt::gray));
        int width_radius = (this_rect.width() / 2) - text_width - offset;
        int height_radius =
            (this_rect.height() / 2) - (text_height * 2) - offset;
        size_t radius = min(width_radius, height_radius);
        int cx = center.x() - radius;
        int cy = center.y() - radius;
        const double raidus_offset = 2.02;
        painter.drawEllipse(cx, cy,
                            static_cast<int>(radius * raidus_offset),
                            static_cast<int>(radius * raidus_offset));

        painter.setPen(QColor(Qt::black));
        if (!is_parameter_loaded_) {
            // パラメータが読み込まれてないときのメッセージ表示
            painter.setBackground(QColor(Qt::white));
            painter.setBackgroundMode(Qt::OpaqueMode);
            painter.drawText(0, 0, this_rect.width(), this_rect.height() / 2,
                             Qt::AlignVCenter | Qt::AlignHCenter,
                             tr("no range parameter."));
            return;
        }

        // 計測範囲の描画
        // !!! first, last の共通部分をまとめる
        double first_radian = index2rad(setting_.first_step) + (M_PI / 2.0);
        double last_radian = index2rad(setting_.last_step) + (M_PI / 2.0);
        QPoint first_point =
            calculatePoint(radius + (offset * 2), first_radian);
        QPoint last_point =
            calculatePoint(radius + (text_height / 2) + offset, last_radian);
        first_point.setY(-first_point.y());
        first_point += center;
        last_point.setY(-last_point.y());
        last_point += center;

        int first_deg = lrint(first_radian * 180.0 / M_PI) - 90;
        int last_deg = lrint(last_radian * 180.0 / M_PI) - 90;

        painter.setBrush(QColor(0x33, 0x77, 0xff));
        painter.drawPie(cx, cy, radius * 2, radius * 2,
                        (first_deg + 90) * 16, (last_deg - first_deg) * 16);

        // 計測範囲の角度を描画
        painter.drawText(first_point.x() + (offset * 3),
                         first_point.y() - (text_height / 2),
                         text_width, text_height,
                         Qt::AlignLeft | Qt::AlignBottom,
                         tr("%1[deg]").arg(first_deg));

        painter.drawText(last_point.x() - text_width,
                         last_point.y() - (text_height / 2),
                         text_width, text_height,
                         Qt::AlignRight | Qt::AlignBottom,
                         tr("%1[deg]").arg(last_deg));
    }


    double index2rad(int index)
    {
        int index_from_front = index - front_index_;
        return index_from_front * (2.0 * M_PI) / total_steps_;
    }


    QPoint calculatePoint(int radius, double radian)
    {
        int x = static_cast<int>(radius * cos(radian));
        int y = static_cast<int>(radius * sin(radian));

        return QPoint(x, y);
    }
};


Preview_widget::Preview_widget(QWidget* widget)
    : QWidget(widget), pimpl(new pImpl(this))
{
    pimpl->initialize_form();
}


Preview_widget::~Preview_widget(void)
{
}


void Preview_widget::clear_setting(void)
{
    pimpl->is_parameter_loaded_ = false;

    repaint();
}


void Preview_widget::set_setting(const Scan_setting& setting, int scan_interval,
                                 int front_index, int total_steps)
{
    pimpl->setting_ = setting;
    pimpl->scan_interval_ = scan_interval;
    pimpl->front_index_ = front_index;
    pimpl->total_steps_ = total_steps;
    pimpl->is_parameter_loaded_ = true;

    repaint();
}


void Preview_widget::paintEvent(QPaintEvent* event)
{
    static_cast<void>(event);

    // 背景の塗りつぶしとプレビュー表示
    QPainter painter(this);
    painter.fillRect(0, 0, width(), height(), QBrush(Qt::white));
    pimpl->draw_preview_image(painter);
    painter.end();
}
