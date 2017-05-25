/*!
  \file
  \brief 距離・強度データの表示

  \author Satofumi Kamimura

  $Id$
*/

#include <QMutexLocker>
#include "Step_value_widget.h"

using namespace hrk;
using namespace std;


namespace
{
    enum {
        Default_step_size = 726,
        Default_max_echo_size = 3,
        Multiecho_row_height = 38,
    };


    typedef struct
    {
        Lidar::measurement_t type;
        vector<long> distance;
        vector<unsigned short> intensity;
    } step_data_t;
}


struct Step_value_widget::pImpl
{
    Step_value_widget* widget_;
    QMutex mutex_;
    size_t steps_;
    size_t step_offset_;
    size_t max_echo_size_;
    step_data_t step_data_;
    Lidar::measurement_t current_type_;
    int selected_step_;


    pImpl(Step_value_widget* widget)
        : widget_(widget),
          steps_(Default_step_size), step_offset_(0),
          max_echo_size_(Default_max_echo_size),
          current_type_(Lidar::Distance), selected_step_(0)
    {
    }


    void initialize_form(void)
    {
        // signals
        connect(widget_->intensity_button_, SIGNAL(clicked()),
                widget_, SLOT(config_button_changed()));
        connect(widget_->multiecho_button_, SIGNAL(clicked()),
                widget_, SLOT(config_button_changed()));
        connect(widget_->update_button_, SIGNAL(clicked()),
                widget_, SIGNAL(update_requested()));
        connect(widget_->table_, SIGNAL(currentCellChanged(int, int, int, int)),
                widget_, SLOT(currentCellChanged(int, int, int, int)));
        connect(widget_->auto_checkbox_, SIGNAL(toggled(bool)),
                widget_, SLOT(auto_checkbox_changed(bool)));
    }


    void set_state(state_t state)
    {
        bool is_update_enabled = false;
        bool is_update_button_enabled = false;
        bool is_config_enabled = false;

        switch (state) {
        case Playing:
            is_update_enabled = true;
            clear_value_data();
            break;

        case Viewing:
            is_update_enabled = true;
            is_update_button_enabled = true;
            is_config_enabled = true;
            break;

        case Not_connected:
            is_config_enabled = true;
            clear_value_data();
            break;

        case Recording:
            is_update_enabled = true;
            break;
        }

        set_update_enabled(is_update_enabled, is_update_button_enabled);
        set_config_enabled(is_config_enabled);
    }


    void set_update_enabled(bool enable, bool button_enable)
    {
        widget_->table_->setEnabled(enable);
        widget_->auto_checkbox_->setEnabled(button_enable);
        bool update_button_enable =
            widget_->auto_checkbox_->isChecked() ? false : button_enable;
        widget_->update_button_->setEnabled(update_button_enable);
    }


    void set_config_enabled(bool enable)
    {
        widget_->intensity_button_->setEnabled(enable);
        widget_->multiecho_button_->setEnabled(enable);
    }


    void update_form(bool with_intensity, bool is_multiecho)
    {
        widget_->table_->clear();

        if (is_multiecho) {
            if (with_intensity) {
                set_multiecho_intnsity_form();
            } else {
                set_multiecho_form();
            }
        } else {
            if (with_intensity) {
                set_distance_intensity_form();
            } else {
                set_distance_form();
            }
        }
    }


    void set_distance_form(void)
    {
        QStringList header_labels;
        header_labels << tr("Distance");
        register_table_items(header_labels);
    }


    void set_distance_intensity_form(void)
    {
        QStringList header_labels;
        header_labels << tr("Distance\nIntensity");
        register_table_items(header_labels);
    }


    void set_multiecho_form(void)
    {
        QStringList header_labels;
        header_labels << tr("Distance")
                      << tr("Distance")
                      << tr("Distance");
        register_table_items(header_labels);
    }


    void set_multiecho_intnsity_form(void)
    {
        QStringList header_labels;
        header_labels << tr("Distance\nIntensity")
                      << tr("Distance\nIntensity")
                      << tr("Distance\nIntensity");
        register_table_items(header_labels);
    }


    void register_table_items(const QStringList& header_labels)
    {
        QTableWidget* table = widget_->table_;

        table->setColumnCount(header_labels.size());
        table->setHorizontalHeaderLabels(header_labels);

        size_t n = table->columnCount();
        for (size_t i = 0; i < n; ++i) {
            table->horizontalHeader()->setResizeMode(i, QHeaderView::Stretch);
        }

        table->setRowCount(steps_);
        table->verticalHeader()->setDefaultAlignment(Qt::AlignCenter);

        for (size_t i = 0; i < n; ++i) {
            set_row_items(steps_, i);
        }

        update_row_labels();
    }


    void set_row_items(size_t steps, size_t column_index)
    {
        QString first_item;
        if (column_index == 0) {
            if (widget_->intensity_button_->isChecked()) {
                first_item = tr("%1 \t%2 ").arg(0).arg(0);
            } else {
                first_item = tr("%1 ").arg(0);
            }
        } else {
            first_item = "";
        }

        QTableWidget* table = widget_->table_;
        for (size_t i = 0; i < steps; ++i) {
            QTableWidgetItem *item = new QTableWidgetItem(first_item);
            item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
            item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
            table->setItem(i, column_index, item);
        }
    }


    bool update_values(void)
    {
        QTableWidget* table = widget_->table_;

        const vector<long>& distance = step_data_.distance;
        const vector<unsigned short>& intensity = step_data_.intensity;

        bool with_intensity = !intensity.empty();
        bool is_multiecho = widget_->multiecho_button_->isChecked();
        size_t echo_size = is_multiecho ? max_echo_size_ : 1;

        size_t n = min(table->rowCount(), static_cast<int>(distance.size()));
        for (size_t i = 0; i < n; ++i) {
            size_t echo_index;
            // 距離データの格納
            for (echo_index = 0; echo_index < echo_size; ++echo_index) {
                QString value;
                QTableWidgetItem* item = table->item(i, echo_index);

                int data_index = (echo_size * i) + echo_index;
                long distance_value = distance[data_index];
                if ((echo_index > 0) && (distance_value == 0)) {
                    break;
                }
                if (with_intensity) {
                    value = tr("%1 \t%2 ")
                        .arg(distance_value).arg(intensity[data_index]);
                } else {
                    value = tr("%1 ").arg(distance_value);
                }
                item->setText(value);
            }

            // 空白の格納
            for (; echo_index < echo_size; ++echo_index) {
                QTableWidgetItem* item = table->item(i, echo_index);
                item->setText("");
            }
        }
        return true;
    }


    void update_row_size(void)
    {
        QTableWidget* table = widget_->table_;

        size_t current_steps = table->rowCount();
        if (current_steps == steps_) {
            update_row_labels();
            return;
        }

        table->setRowCount(steps_);
        if (current_steps > steps_) {
            update_row_labels();
            return;
        }

        size_t column_size = table->columnCount();
        for (size_t i = current_steps; i < steps_; ++i) {
            for (size_t k = 0; k < column_size; ++k) {
                QTableWidgetItem *item = new QTableWidgetItem("");
                item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
                item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
                table->setItem(i, k, item);
            }
        }

        update_row_labels();
    }


    void update_row_labels(void)
    {
        QTableWidget* table = widget_->table_;

        QStringList row_labels;
        for (size_t i = 0; i < steps_; ++i) {
            row_labels << QString("%1").arg(i + step_offset_);
        }
        table->setVerticalHeaderLabels(row_labels);

        for (size_t i = 0; i < steps_; ++i) {
            table->setRowHeight(i, Multiecho_row_height);
        }
    }


    void clear_value_data(void)
    {
        update_form(widget_->intensity_button_->isChecked(),
                    widget_->multiecho_button_->isChecked());
    }
};


Step_value_widget::Step_value_widget(QWidget* parent)
    : QWidget(parent), pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
}


Step_value_widget::~Step_value_widget(void)
{
}


void Step_value_widget::set_state(state_t state)
{
    pimpl->set_state(state);
}


void Step_value_widget::set_max_echo_size(int echoes)
{
    if (echoes < 0) {
        return;
    }
    pimpl->max_echo_size_ = echoes;
}


void Step_value_widget::set_steps(int steps, int step_offset)
{
    QMutexLocker locker(&pimpl->mutex_);

    if (steps < 0) {
        return;
    }

    pimpl->steps_ = steps;
    pimpl->step_offset_ = step_offset;
    pimpl->update_row_size();
}


void Step_value_widget::set_config(bool with_intensity, bool is_multiecho)
{
    intensity_button_->setChecked(with_intensity);
    multiecho_button_->setChecked(is_multiecho);
    config_button_changed();
}


void Step_value_widget::restrict_config_buttons(bool with_intensity,
                                                bool is_multiecho)
{
    intensity_button_->setVisible(with_intensity);
    multiecho_button_->setVisible(is_multiecho);
}


void Step_value_widget::set_auto_update(bool auto_update)
{
    auto_checkbox_->setChecked(auto_update);
}


bool Step_value_widget::auto_update(void) const
{
    return auto_checkbox_->isChecked();
}


bool Step_value_widget::set_value_data(hrk::Lidar::measurement_t type,
                                       const std::vector<long>& distance,
                                       const std::vector<unsigned short>&
                                       intensity)
{
    QMutexLocker locker(&pimpl->mutex_);

    if (pimpl->current_type_ != type) {
        // 要求している種類のデータでなければ、戻る
        return false;
    }

    pimpl->step_data_.type = type;
    pimpl->step_data_.distance = distance;
    pimpl->step_data_.intensity = intensity;

    return pimpl->update_values();
}


void Step_value_widget::config_button_changed(void)
{
    pimpl->mutex_.lock();

    bool with_intensity = intensity_button_->isChecked();
    bool is_multiecho = multiecho_button_->isChecked();

    if (with_intensity) {
        if (is_multiecho) {
            pimpl->current_type_ = Lidar::Multiecho_intensity;
        } else {
            pimpl->current_type_ = Lidar::Distance_intensity;
        }
    } else {
        if (is_multiecho) {
            pimpl->current_type_ = Lidar::Multiecho;
        } else {
            pimpl->current_type_ = Lidar::Distance;
        }
    }

    pimpl->update_form(with_intensity, is_multiecho);
    pimpl->mutex_.unlock();

    emit config_changed(with_intensity, is_multiecho);
}


void Step_value_widget::show_step_data(int step)
{
    // 全範囲のセルを非選択にする
    int column_count = table_->columnCount();
    QTableWidgetSelectionRange all_range(0, 0,
                                     table_->rowCount() - 1,
                                     column_count - 1);
    table_->setRangeSelected(all_range, false);

    // 指定された範囲を選択にして、スクロールさせる
    QTableWidgetSelectionRange selected_range(step, 0, step, column_count - 1);
    table_->setRangeSelected(selected_range, true);
    QTableWidgetItem* item = table_->item(step, 0);
    if (item) {
        table_->scrollToItem(item);
    }

    pimpl->selected_step_ = step;
}


void Step_value_widget::currentCellChanged(int current_row, int current_column,
                                           int previous_row,
                                           int previous_column)
{
    static_cast<void>(current_column);
    static_cast<void>(previous_row);
    static_cast<void>(previous_column);

    // 全範囲の選択を解除する
    QTableWidgetSelectionRange
        all_range(0, 0, table_->rowCount(), table_->columnCount());
    table_->setRangeSelected(all_range, false);

    int step = current_row;
    if ((previous_row >= 0) || (current_row != 0)) {
        // table を enable にすると (current: 0, previous: -1) になるのを除外
        emit selection_changed(step);
    }
}


void Step_value_widget::auto_checkbox_changed(bool auto_update)
{
    update_button_->setEnabled(!auto_update);
    emit auto_update_changed(auto_update);
}
