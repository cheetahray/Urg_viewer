#ifndef STEP_VALUE_WIDGET_H
#define STEP_VALUE_WIDGET_H

/*!
  \file
  \brief 距離・強度データの表示

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "ui_Step_value_widget_form.h"
#include "State.h"
#include "Lidar.h"


class Step_value_widget
    : public QWidget, public State, private Ui::Step_value_widget_form
{
    Q_OBJECT;

 public:
    Step_value_widget(QWidget* parent = NULL);
    ~Step_value_widget(void);

    void set_state(state_t state);
    void set_max_echo_size(int echoes);
    void set_steps(int steps, int step_offset = 0);
    void set_config(bool with_intensity, bool is_multiecho);
    void restrict_config_buttons(bool with_intensity, bool is_multiecho);
    void set_auto_update(bool auto_update);
    bool auto_update(void) const;

    bool set_value_data(hrk::Lidar::measurement_t type,
                        const std::vector<long>& distance,
                        const std::vector<unsigned short>& intensity);

 signals:
    void config_changed(bool with_intensity, bool is_multiecho);
    void update_requested(void);
    void selection_changed(int step);
    void auto_update_changed(bool auto_update);

 private slots:
    void config_button_changed(void);
    void show_step_data(int step);
    void currentCellChanged(int current_row, int current_column,
                            int previous_row, int previous_column);
    void auto_checkbox_changed(bool auto_update);

 private:
    Step_value_widget(const Step_value_widget& rhs);
    Step_value_widget& operator = (const Step_value_widget& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
