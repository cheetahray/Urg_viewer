#ifndef PLOT_2D_WIDGET_H
#define PLOT_2D_WIDGET_H

/*!
  \file
  \brief URG データの 2D 描画 Widget

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <QGLWidget>
#include "State.h"
#include "Lidar.h"

class Scan_setting;
class Step_value_widget;


class Plotter_2d_widget : public QGLWidget, public State
{
    Q_OBJECT;

 public:
    typedef enum {
        None,
        Playing,
        Recording,
        Pausing,
    } icon_t;

    Plotter_2d_widget(hrk::Lidar& lidar, Step_value_widget& step_value_widget,
                   QWidget* parent = NULL);
    ~Plotter_2d_widget(void);

    void set_state(state_t state);
    void set_step_value_auto_update(bool on);

    void set_scan_setting(const Scan_setting& setting);
    void set_plot_data(hrk::Lidar::measurement_t type,
                       std::vector<long>& distance,
                       std::vector<unsigned short>& intensity,
                       long timestamp);
    void clear_message(void);
    void set_message(const QString& message);
    void set_icon(icon_t icon);
    void redraw(void);
    void wait(void);

 signals:
    void clicked_mm_point(long mm_x, long mm_y);

 public slots:
    void step_value_requested(void);
    void show_step_data(int step);
    void zoom_out(void);
    void zoom_in(void);
    void zoom_fit(void);
    void recorded(long recorded_times, long loss_times);

 protected:
    void initializeGL(void);
    void resizeGL(int width, int height);
    void paintGL(void);

    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);

 private:
    Plotter_2d_widget(const Plotter_2d_widget& rhs);
    Plotter_2d_widget& operator = (const Plotter_2d_widget& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
