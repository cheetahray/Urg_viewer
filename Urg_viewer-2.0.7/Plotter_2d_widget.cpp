/*!
  \file
  \brief URG データの 2D 描画 Widget

  \author Satofumi Kamimura

  $Id$
*/

#define USE_GL_2 1

#if defined(USE_GL_2)
#define GL_GLEXT_PROTOTYPES 1
#define GL3_PROTOTYPES 1
#endif

#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include <QtCore/qmath.h>

#include "detect_os.h"
#include <cmath>
#if defined(MAC_OS)
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#elif defined(WINDOWS_OS)
#include <GL/glew.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif
#include <QMutexLocker>
#include <QWheelEvent>
#include "Plotter_2d_widget.h"
#include "Step_value_widget.h"
#include "Scan_setting.h"
#include "Color.h"

#include <cstdio>

using namespace hrk;
using namespace std;


namespace
{
    enum {
        Minimum_width = 300,
        Minimum_height = 300,
        Number_of_arc = 30,     // 1 [m] から 30 [m] までの 30 本
    };

    const double Required_minimum_GL_version = 1.6;

    typedef struct
    {
        Lidar::measurement_t type;
        long timestamp;
        vector<long> distance;
        vector<unsigned short> intensity;
    } plot_data_t;

    typedef struct
    {
        GLfloat x;
        GLfloat y;
    } vector_t;

    typedef vector<vector_t> scan_data_t;
    typedef vector<scan_data_t> scans_t;

    const double Default_mm_per_pixel = 10.0;

    typedef vector<vector_t> Points;
    typedef vector<Points> Points_group;
}

UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );

struct Plotter_2d_widget::pImpl
{
    Plotter_2d_widget* widget_;
    Step_value_widget& step_value_widget_;
    QMutex mutex_;
    Lidar& lidar_;
    QColor clear_color_;
    plot_data_t plot_data_;
    bool is_step_value_requested_;
    state_t current_state_;
    Scan_setting setting_;

    bool is_old_gl_;
    GLuint grid_buffer_id_;
    int grid_points_size_;
    vector<GLuint> arc_buffer_ids_;
    vector<int> arc_points_size_;
    GLuint arc_buffer_id_;
    GLuint step_buffer_id_;
    bool is_arc_discarded_;
    bool exist_step_line_;
    bool is_plot_data_updated_;
    vector<GLuint> scans_buffer_ids_;
    vector<int> scans_points_size_;
    int echo_size_;
    long min_distance_;
    vector<Color> plot_distance_colors_;
    vector<Color> plot_intensity_colors_;
    int pixel_width_;
    int pixel_height_;
    double mm_per_pixel_;
    bool mouse_pressing_;
    QPoint clicked_point_;
    QPoint last_point_;
    vector_t moved_mm_;
    QString draw_message_;
    icon_t draw_icon_;
    GLuint play_icon_id_;
    GLuint recording_icon_id_;
    GLuint pausing_icon_id_;
    bool is_updated_;
    QPoint mm_point_;
    bool is_mm_point_valid_;
    bool is_auto_update_;

    // for old OpenGL
    Points lines_points_;
    Points arc_lines_points_;
    Points_group arc_points_group_;
    Points step_line_;
    Points_group data_points_;

    pImpl(Plotter_2d_widget* widget, Step_value_widget& step_value_widget,
          Lidar& lidar)
        : widget_(widget), step_value_widget_(step_value_widget),
          lidar_(lidar), clear_color_(Qt::white),
          is_step_value_requested_(false), is_old_gl_(false),
          is_arc_discarded_(false), exist_step_line_(false),
          is_plot_data_updated_(false), echo_size_(1), min_distance_(0),
          pixel_width_(Minimum_width), pixel_height_(Minimum_height),
          mm_per_pixel_(Default_mm_per_pixel), mouse_pressing_(false),
          draw_icon_(None), is_updated_(false),
          is_mm_point_valid_(false), is_auto_update_(false)
    {
        // 初期位置を下の方にずらす
        set_default_moved();

        plot_distance_colors_.push_back(Color(0, 0, 128/255.0));
        plot_distance_colors_.push_back(Color(0, 1.0, 1.0));
        plot_distance_colors_.push_back(Color(30/255.0, 144/255.0, 255/255.0));
        plot_intensity_colors_.push_back(Color(255/255.0, 69/255.0, 0.0));
        plot_intensity_colors_.push_back(Color(1.0, 0.0, 1.0));
        plot_intensity_colors_.push_back(Color(255/255.0, 215/255.0, 0/255.0));

    }


    void set_default_moved(void)
    {
        moved_mm_.x = 0.0;
        moved_mm_.y = -1000.0;
    }


    void initialize_form(void)
    {
        widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        widget_->setMinimumSize(Minimum_width, Minimum_height);
        widget_->setMouseTracking(true);
    }


    void clear(void)
    {
        widget_->qglClearColor(clear_color_);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }


    void create_grid_data(void)
    {
        // -110 [m] から +110 [m] までの範囲に直線を描画する
        const int first = -110 * 1000;
        const int last = +110 * 1000;

        vector<vector_t> lines;
        for (int i = first; i <= last; i += 1000) {
            vector_t v;

            // X 軸
            v.x = i;
            v.y = first;
            lines.push_back(v);

            v.x = i;
            v.y = last;
            lines.push_back(v);

            // Y 軸
            v.x = first;
            v.y = i;
            lines.push_back(v);

            v.x = last;
            v.y = i;
            lines.push_back(v);
        }
        grid_points_size_ = lines.size();

        if (!is_old_gl_) {
            int memory_size = grid_points_size_ * sizeof(vector_t);
            glGenBuffers(1, &grid_buffer_id_);
            glBindBuffer(GL_ARRAY_BUFFER, grid_buffer_id_);
            glBufferData(GL_ARRAY_BUFFER,
                         memory_size, &lines[0], GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            // 円弧を描くためのバッファを取得する
            arc_buffer_ids_.resize(Number_of_arc);
            glGenBuffers(Number_of_arc, &arc_buffer_ids_[0]);

            // 計測範囲を示す直線用のバッファを取得する
            glGenBuffers(1, &arc_buffer_id_);
        } else {
            lines_points_ = lines;
        }

        // 270 [deg] の範囲で円弧を描く
        create_default_arc_lines();
    }


    void create_default_arc_lines(void)
    {
        double min_radian = -M_PI / 4;
        double max_radian = +M_PI * 5 / 4;
        update_arc_lines(min_radian, max_radian);
    }


    void update_arc_lines_used_by_lider(void)
    {
        // センサパラメータに基づいた円弧の作成
        const double resolution = 2.0 * M_PI / lidar_.total_steps();
        double min_radian =
            (lidar_.min_step() - lidar_.front_step()) * resolution + M_PI/2.0;
        double max_radian =
            (lidar_.max_step() - lidar_.front_step()) * resolution + M_PI/2.0;
        update_arc_lines(min_radian, max_radian);
    }


    void update_arc_lines(double min_radian, double max_radian)
    {
        vector<vector_t> points;
        vector_t v;

        const long distance = 200 * 1000;

        v.x = 0;
        v.y = 0;
        points.push_back(v);

        v.x = distance * cos(min_radian);
        v.y = distance * sin(min_radian);
        points.push_back(v);

        v.x = 0;
        v.y = 0;
        points.push_back(v);

        v.x = distance * cos(max_radian);
        v.y = distance * sin(max_radian);
        points.push_back(v);

        if (!is_old_gl_) {
            int memory_size = 4 * sizeof(vector_t);
            glGenBuffers(1, &arc_buffer_id_);
            glBindBuffer(GL_ARRAY_BUFFER, arc_buffer_id_);
            glBufferData(GL_ARRAY_BUFFER,
                         memory_size, &points[0], GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        } else {
            arc_lines_points_ = points;
        }

        update_arcs(min_radian, max_radian);
    }


    void update_arcs(double min_radian, double max_radian)
    {
        arc_points_size_.clear();

        // 30 [m] の範囲まで 1 [m] 毎に円弧を作成する
        for (int i = 0; i < Number_of_arc; ++i) {
            if (!is_old_gl_) {
                create_arc(i + 1, min_radian, max_radian, arc_buffer_ids_[i]);
            } else {
                create_arc(i + 1, min_radian, max_radian, 0);
            }
        }
    }


    void create_arc(int radius, double min_radian, double max_radian,
                    GLuint buffer_id)
    {
        const double draw_radius = radius * 1000;
        int min_degree = 180 * min_radian / M_PI;
        int max_degree = 180 * max_radian / M_PI;

        vector<vector_t> points;
        for (int degree = min_degree; degree <= max_degree; degree += 4) {
            const double radian = M_PI * degree / 180.0;
            vector_t v;
            v.x = draw_radius * cos(radian);
            v.y = draw_radius * sin(radian);
            points.push_back(v);
        }

        if (!is_old_gl_) {
            int memory_size = points.size() * sizeof(vector_t);
            glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
            glBufferData(GL_ARRAY_BUFFER,
                         memory_size, &points[0], GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            arc_points_size_.push_back(points.size());
        } else {
            arc_points_group_.push_back(points);
        }
    }


    void create_step_line(void)
    {
        if (!is_old_gl_) {
            glGenBuffers(1, &step_buffer_id_);
        }
    }


    void draw_grid(void)
    {
        glColor3f(0.8, 0.8, 0.8);
        glPointSize(1.0);

        if (!is_old_gl_) {
            glBindBuffer(GL_ARRAY_BUFFER, grid_buffer_id_);
            glInterleavedArrays(GL_V2F, 0, NULL);
            glDrawArrays(GL_LINES, 0, grid_points_size_);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        } else {
            draw_lines(lines_points_);
        }
    }


    void draw_arc_lines(void)
    {
        glPointSize(1.0);

        if (!is_old_gl_) {
            // 計測範囲の直線を描画
            glColor3f(0.9, 0.9, 0.9);
            glBindBuffer(GL_ARRAY_BUFFER, arc_buffer_id_);
            glInterleavedArrays(GL_V2F, 0, NULL);
            glDrawArrays(GL_LINES, 0, 4);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            // 円弧を描画
            glColor3f(0.8, 0.8, 0.8);
            int index = 0;
            for (vector<GLuint>::const_iterator it = arc_buffer_ids_.begin();
                 it != arc_buffer_ids_.end(); ++it, ++index) {
                glBindBuffer(GL_ARRAY_BUFFER, *it);
                glInterleavedArrays(GL_V2F, 0, NULL);
                glDrawArrays(GL_LINE_STRIP, 0, arc_points_size_[index]);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        } else {
            draw_lines(arc_lines_points_);
            draw_line_strip(arc_points_group_);
        }
    }


    void draw_step_line(void)
    {
        if (!exist_step_line_) {
            return;
        }

        glColor3f(0.0, 1.0, 0.0);

        if (!is_old_gl_) {
            glBindBuffer(GL_ARRAY_BUFFER, step_buffer_id_);
            glInterleavedArrays(GL_V2F, 0, NULL);
            glDrawArrays(GL_LINES, 0, 2);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        } else {
            if (!step_line_.empty()) {
                draw_lines(step_line_);
            }
        }
    }


    void draw_points(double magnify)
    {
        const double point_size = (magnify > 0.003) ? 4.0 : 3.0;
        glPointSize(point_size);

        if (!is_old_gl_) {
            size_t index = 0;
            for (vector<GLuint>::const_iterator it = scans_buffer_ids_.begin();
                 it != scans_buffer_ids_.end(); ++it, ++index) {
                int points_size = scans_points_size_[index];
                if (points_size <= 0) {
                    continue;
                }

                set_plot_color(index);

                glBindBuffer(GL_ARRAY_BUFFER, *it);
                glInterleavedArrays(GL_V2F, 0, NULL);
                glDrawArrays(GL_POINTS, 0, points_size);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        } else {
            size_t n = data_points_.size();
            for (size_t i = 0; i < n; ++i) {
                if (data_points_[i].empty()) {
                    continue;
                }
                set_plot_color(i);
                draw_points(data_points_[i]);
            }
        }
    }


    void set_plot_color(size_t index)
    {
        Color* color = NULL;
        if (static_cast<int>(index) >= echo_size_) {
            // 強度の色
            size_t offset = index - echo_size_;
            offset = min(offset, plot_intensity_colors_.size() - 1);
            color = &plot_intensity_colors_[offset];
        } else {
            // 距離の色
            size_t color_index = min(index, plot_distance_colors_.size() - 1);
            color = &plot_distance_colors_[color_index];
        }

        glColor3f(color->red(), color->green(), color->blue());
    }


    void set_value_data(void)
    {
        if (plot_data_.distance.empty()) {
            // distance が empty ならばデータが格納されていないと判断して戻る
            return;
        }

        if (step_value_widget_.
            set_value_data(plot_data_.type,
                           plot_data_.distance, plot_data_.intensity)) {
            is_step_value_requested_ = false;
        }
    }


    void update_plot_data(void)
    {
        if (!lidar_.is_open()) {
            return;
        }

        int scan_data_size = echo_size_ * (setting_.with_intensity ? 2 : 1);
        scans_t scans(scan_data_size);
        scans_points_size_.clear();

        // 距離データを描画用のデータに変換する
        vector<unsigned short>::const_iterator intensity_it =
            plot_data_.intensity.begin();
        int grouping_add_size = max(1, setting_.group_steps);
        int index = 0;
        for (vector<long>::const_iterator it = plot_data_.distance.begin();
             it != plot_data_.distance.end(); ++it, ++intensity_it, ++index) {
            long distance = *it;
            if (distance <= min_distance_) {
                continue;
            }

            for (int i = 0; i < grouping_add_size; ++i) {
                int step = (grouping_add_size * (index / echo_size_)) + i;
                const double radian =
                    lidar_.step2rad(step) + (M_PI / 2.0);
                vector_t v;
                v.x = distance * cos(radian);
                v.y = distance * sin(radian);
                const int scans_index = index % echo_size_;
                scans[scans_index].push_back(v);

                if(abs((int)v.x) > 75 && abs((int)v.y) > 75 && abs((int)v.x) < 1300 && abs((int)v.y) < 1300 )
                {
                    char buffer[OUTPUT_BUFFER_SIZE];
                    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );

                    p << osc::BeginBundleImmediate
                        /*
                        << osc::BeginMessage( "/test1" )
                            << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
                        << osc::BeginMessage( "/test2" )
                            << true << 24 << (float)10.8 << "world" << osc::EndMessage
                        */
                        << osc::BeginMessage( "/xy" )
                            << (float)v.x << (float)v.y << osc::EndMessage
                        << osc::EndBundle;

                    transmitSocket.Send( p.Data(), p.Size() );
                }
                if (setting_.with_intensity) {
                    // 強度データを描画用のデータに変換する
                    unsigned short intensity = *intensity_it;
                    v.x = intensity * cos(radian);
                    v.y = intensity * sin(radian);
                    scans[echo_size_ + scans_index].push_back(v);
                }
            }
        }
        set_data_to_buffer(scans);
    }


    void set_data_to_buffer(const scans_t& scans)
    {
        size_t n = scans.size();
        for (size_t i = 0; i < n; ++i) {
            const scan_data_t& scan_data = scans[i];

            if (!is_old_gl_) {
                int points_size = scan_data.size();
                int memory_size = points_size * sizeof(vector_t);
                glBindBuffer(GL_ARRAY_BUFFER, scans_buffer_ids_[i]);
                glBufferData(GL_ARRAY_BUFFER,
                             memory_size, &scan_data[0], GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                scans_points_size_.push_back(points_size);
            } else {
                data_points_[i] = scan_data;
            }
        }
    }


    void set_scan_setting(const Scan_setting& setting)
    {
        clear_plot_data();

        setting_ = setting;
        echo_size_ = setting.is_multiecho ? lidar_.max_echo_size() : 1;
        int plot_times = echo_size_ * (setting.with_intensity ? 2 : 1);
        min_distance_ = lidar_.min_distance();

        if (!is_old_gl_) {
            // 距離データ計測用バッファの初期化
            if (!scans_buffer_ids_.empty()) {
                glDeleteBuffers(scans_buffer_ids_.size(),
                                &scans_buffer_ids_[0]);
            }
            scans_buffer_ids_.resize(plot_times);
            glGenBuffers(plot_times, &scans_buffer_ids_[0]);
        } else {
            data_points_.resize(plot_times);
        }

        is_arc_discarded_ = true;
    }


    void clear_plot_data(void)
    {
        plot_data_.distance.clear();
        echo_size_ = 1;
        exist_step_line_ = false;
        is_updated_ = true;
    }


    void update_zoom_magnify(int steps)
    {
        double next_mm_per_pixel = mm_per_pixel_ * pow(1.2, steps);

        const double max_mm_per_pixel = 100.0;
        const double min_mm_per_pixel = 0.1;
        next_mm_per_pixel = max(min(next_mm_per_pixel, max_mm_per_pixel),
                                min_mm_per_pixel);

        if (mm_per_pixel_ != next_mm_per_pixel) {
            mm_per_pixel_ = next_mm_per_pixel;
            is_updated_ = true;
        }
    }


    double scale_magnify(void)
    {
        return 1.0 / mm_per_pixel_ / pixel_height_;
    }


    QPoint calculate_mm_point(const QPoint& current_point)
    {
        QPoint point;
        point.setX(((current_point.x() - pixel_width_/2.0)
                    * mm_per_pixel_ * 2.0) - moved_mm_.x);
        point.setY(-((current_point.y() - pixel_height_/2.0)
                     * mm_per_pixel_ * 2.0) - moved_mm_.y);
        return point;
    }


    void draw_message(void)
    {
        if (draw_message_.isEmpty()) {
            return;
        }

        glPopMatrix();
        glTranslatef(0, 0, +1.0);

        const double aspect = 1.0 * pixel_width_ / pixel_height_;
        QRectF icon_rect(0, 0,
                         48.0 / pixel_width_ * aspect, 48.0 / pixel_height_);
        if (draw_icon_ == Playing) {
            glTranslatef(-aspect + (5.0 / pixel_width_),
                         +1.0 - (53.0 / pixel_width_ * aspect), 0.0);
            glColor3f(0.0, 0.0, 0.0);
            widget_->drawTexture(icon_rect, play_icon_id_);

        } else if (draw_icon_ == Pausing) {
            glTranslatef(-aspect + (5.0 / pixel_width_),
                         +1.0 - (53.0 / pixel_width_ * aspect), 0.0);
            glColor3f(0.0, 0.0, 0.0);
            widget_->drawTexture(icon_rect, pausing_icon_id_);

        } else if (draw_icon_ == Recording) {
            glTranslatef(-aspect + (5.0 / pixel_width_),
                         +1.0 - (53.0 / pixel_width_ * aspect), 0.0);
            glColor3f(1.0, 0.0, 0.0);
            widget_->drawTexture(icon_rect, recording_icon_id_);
        }

        glColor3f(0.0, 0.0, 0.0);
        QFont font;
        font.setPointSize(12);
        widget_->renderText(26, 20, draw_message_, font);
    }


    void draw_mm_point(void)
    {
        if (!is_mm_point_valid_) {
            return;
        }

        QString x_number = format_number(mm_point_.x());
        QString y_number = format_number(mm_point_.y());
        QString text = tr("X: %1 [mm], Y: %2 [mm]").arg(x_number, y_number);

        QFont font;
        font.setPointSize(10);
        glColor3f(0.0, 0.0, 0.0);
        widget_->renderText(5, pixel_height_ - 6, text, font);
    }


    QString format_number(long mm)
    {
        long m = mm / 1000;
        mm %= 1000;

        if (m == 0) {
            return QString("%1").arg(mm);
        } else {
            QChar zero('0');
            return QString("%1,%2").arg(m).arg(labs(mm), 3, 10, zero);
        }
    }


    void draw(void)
    {
        clear();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glPushMatrix();

        const double magnify = scale_magnify();
        glScalef(magnify, magnify, magnify);

        glTranslatef(moved_mm_.x, moved_mm_.y, 0.0);

        // distance が empty ならばデータが格納されていないと判断する
        bool is_invalid_data = plot_data_.distance.empty();
        if (!is_invalid_data) {
            if (is_step_value_requested_ || is_auto_update_ ||
                (current_state_ == State::Playing)) {
                set_value_data();
            }
        }

        bool is_draw_data =
            !is_invalid_data && (current_state_ != State::Not_connected);

        draw_grid();
        if (is_arc_discarded_ && is_draw_data) {
            is_arc_discarded_ = false;
            update_arc_lines_used_by_lider();
        }
        draw_arc_lines();

        glTranslatef(0.0, 0.0, 1.0);
        draw_step_line();
        glTranslatef(0.0, 0.0, 1.0);

        if (is_draw_data) {
            // 新規データが登録されていれば、描画するデータを更新する
            if (is_plot_data_updated_) {
                is_plot_data_updated_ = false;
                update_plot_data();
            }
            draw_points(magnify);
        }

        draw_message();
        draw_mm_point();
    }


    void draw_lines(const Points& points)
    {
        glBegin(GL_LINES);
        size_t n = points.size();
        for (size_t i = 0; i < n; i += 2) {
            const vector_t& from = points[i];
            const vector_t& to = points[i + 1];
            glVertex2d(from.x, from.y);
            glVertex2d(to.x, to.y);
        }
        glEnd();
    }


    void draw_line_strip(const Points_group& points_group)
    {
        for (Points_group::const_iterator it = points_group.begin();
             it != points_group.end(); ++it) {

            glBegin(GL_LINE_STRIP);
            const Points& points = *it;
            size_t n = points.size();
            for (size_t i = 0; i < n; ++i) {
                const vector_t& p = points[i];
                glVertex2d(p.x, p.y);
            }
            glEnd();
        }
    }


    void draw_points(const Points& points)
    {
        glBegin(GL_POINTS);
        for (Points::const_iterator it = points.begin();
             it != points.end(); ++it) {
            const vector_t& p = *it;
            glVertex2d(p.x, p.y);
        }
        glEnd();
    }
};


Plotter_2d_widget::Plotter_2d_widget(hrk::Lidar& lidar,
                                     Step_value_widget& step_value_widget,
                                     QWidget* parent)
    : QGLWidget(parent), pimpl(new pImpl(this, step_value_widget, lidar))
{
    pimpl->initialize_form();
}


Plotter_2d_widget::~Plotter_2d_widget(void)
{
}


void Plotter_2d_widget::set_state(state_t state)
{
    pimpl->current_state_ = state;

    if ((state == State::Not_connected) || (state == State::Playing)) {
        pimpl->clear_plot_data();
    }
}


void Plotter_2d_widget::set_step_value_auto_update(bool on)
{
    pimpl->is_auto_update_ = on;
}


void Plotter_2d_widget::set_scan_setting(const Scan_setting& setting)
{
    QMutexLocker locker(&pimpl->mutex_);
    pimpl->set_scan_setting(setting);
}


void Plotter_2d_widget::set_plot_data(hrk::Lidar::measurement_t type,
                                      std::vector<long>& distance,
                                      std::vector<unsigned short>& intensity,
                                      long timestamp)
{
    QMutexLocker locker(&pimpl->mutex_);

    pimpl->plot_data_.type = type;
    pimpl->plot_data_.distance.swap(distance);
    pimpl->plot_data_.intensity.swap(intensity);
    pimpl->plot_data_.timestamp = timestamp;

    pimpl->is_plot_data_updated_ = true;
    pimpl->is_updated_ = true;
}


void Plotter_2d_widget::clear_message(void)
{
    pimpl->draw_message_.clear();
    pimpl->draw_icon_ = None;
    pimpl->is_updated_ = true;
}


void Plotter_2d_widget::set_message(const QString& message)
{
    pimpl->draw_message_ = message;
    pimpl->is_updated_ = true;
}


void Plotter_2d_widget::set_icon(icon_t icon)
{
    pimpl->draw_icon_ = icon;
    pimpl->is_updated_ = true;
}


void Plotter_2d_widget::redraw(void)
{
    if (pimpl->is_updated_) {
        updateGL();
        pimpl->is_updated_ = false;
    }
}


void Plotter_2d_widget::wait(void)
{
    QMutexLocker locker(&pimpl->mutex_);
}


void Plotter_2d_widget::step_value_requested(void)
{
    QMutexLocker locker(&pimpl->mutex_);
    pimpl->is_step_value_requested_ = true;
}


void Plotter_2d_widget::show_step_data(int step)
{
    if (step < 0) {
        return;
    }

    int echo_size =
        pimpl->setting_.is_multiecho ? pimpl->lidar_.max_echo_size() : 1;
    const double radian =
        pimpl->lidar_.index2rad(step * echo_size) + (M_PI / 2.0);
    const long distance = 120 * 1000;

    vector<vector_t> points;
    vector_t v;
    v.x = 0;
    v.y = 0;
    points.push_back(v);

    v.x = distance * cos(radian);
    v.y = distance * sin(radian);
    points.push_back(v);

    if (!pimpl->is_old_gl_) {
        int memory_size = points.size() * sizeof(vector_t);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->step_buffer_id_);
        glBufferData(GL_ARRAY_BUFFER, memory_size, &points[0], GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    } else {
        pimpl->step_line_ = points;
    }

    pimpl->exist_step_line_ = true;
    pimpl->is_updated_ = true;
}


void Plotter_2d_widget::zoom_out(void)
{
    pimpl->update_zoom_magnify(+1);
}


void Plotter_2d_widget::zoom_in(void)
{
    pimpl->update_zoom_magnify(-1);
}


void Plotter_2d_widget::zoom_fit(void)
{
    pimpl->mm_per_pixel_ = Default_mm_per_pixel;
    pimpl->update_zoom_magnify(0);
    pimpl->set_default_moved();
}


void Plotter_2d_widget::recorded(long recorded_times, long loss_times)
{
    if (recorded_times <= 0) {
        clear_message();
        return;
    }
    set_message(tr("Recorded %1 scans, loss %2 scans").
                arg(recorded_times).arg(loss_times));
    set_icon(Plotter_2d_widget::Recording);
}


void Plotter_2d_widget::initializeGL(void)
{
    const string gl_version_string =
        reinterpret_cast<const char*>(glGetString(GL_VERSION));
    double gl_version = atof(gl_version_string.c_str());
    if (gl_version <= Required_minimum_GL_version) {
        pimpl->is_old_gl_ = true;
    }

#if defined(WINDOWS_OS)
    glewInit();
#endif
    qglClearColor(Qt::black);

    pimpl->clear();
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glViewport(-1, -1, +1, +1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-1.0, +1.0, -1.0, +1.0, -10.0, +10.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);

    if (!pimpl->is_old_gl_) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_INDEX_ARRAY);
    }

    // データの初期化
    pimpl->create_grid_data();
    pimpl->create_step_line();

    pimpl->play_icon_id_ = bindTexture(QImage(":/icons/play_icon"));
    pimpl->recording_icon_id_ = bindTexture(QImage(":/icons/record_icon"));
    pimpl->pausing_icon_id_ = bindTexture(QImage(":/icons/pause_icon"));
}


void Plotter_2d_widget::resizeGL(int width, int height)
{
    pimpl->pixel_height_ = height;
    pimpl->pixel_width_ = width;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double aspect = 1.0 * width / height;
    glOrtho(-1.0 * aspect, +1.0 * aspect, -1.0, +1.0, -10.0, +10.0);

    glMatrixMode(GL_MODELVIEW);
    pimpl->is_updated_ = true;
}


void Plotter_2d_widget::paintGL(void)
{
    QMutexLocker locker(&pimpl->mutex_);
    pimpl->draw();
}


void Plotter_2d_widget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        pimpl->mouse_pressing_ = true;
        pimpl->clicked_point_ = event->pos();
        pimpl->last_point_ = pimpl->clicked_point_;
    }
}


void Plotter_2d_widget::mouseMoveEvent(QMouseEvent* event)
{
    // カーソル位置が [mm] 位置のどこに相当するかを表示する
    const QPoint current_point = event->pos();
    int x = current_point.x();
    int y = current_point.y();

    // Widget 外かの判定
    if ((x < 0) || (x >= static_cast<int>(pimpl->pixel_width_)) ||
        (y < 0) || (y >= static_cast<int>(pimpl->pixel_height_))) {
        pimpl->is_mm_point_valid_ = false;

    } else {
        pimpl->mm_point_ = pimpl->calculate_mm_point(current_point);
        pimpl->is_mm_point_valid_ = true;
        pimpl->is_updated_ = true;
    }

    if (pimpl->mouse_pressing_) {
        int moved_x = x - pimpl->last_point_.x();
        int moved_y = y - pimpl->last_point_.y();

        pimpl->moved_mm_.x += moved_x * pimpl->mm_per_pixel_ * 2.0;
        pimpl->moved_mm_.y -= moved_y * pimpl->mm_per_pixel_ * 2.0;

        pimpl->last_point_ = current_point;

        setCursor(Qt::ClosedHandCursor);

        pimpl->is_updated_ = true;
    }
}


void Plotter_2d_widget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        pimpl->mouse_pressing_ = false;

        const int Tiny_pixel = 4;
        if ((abs(pimpl->clicked_point_.x() - event->pos().x()) < Tiny_pixel) &&
            (abs(pimpl->clicked_point_.y() - event->pos().y()) < Tiny_pixel)) {
            // クリック位置 (x [mm], y [mm]) を通知する
            const QPoint current_point = event->pos();
            pimpl->mm_point_ = pimpl->calculate_mm_point(current_point);
            emit clicked_mm_point(pimpl->mm_point_.x(), pimpl->mm_point_.y());
        }
    }
    setCursor(Qt::ArrowCursor);
}


void Plotter_2d_widget::wheelEvent(QWheelEvent* event)
{
    int degrees = event->delta() / 8;
    int steps = degrees / 15;

    QPoint first_mm_point = pimpl->calculate_mm_point(event->pos());
    event->accept();
    pimpl->update_zoom_magnify(steps);
    QPoint current_mm_point = pimpl->calculate_mm_point(event->pos());

    pimpl->moved_mm_.x += current_mm_point.x() - first_mm_point.x();
    pimpl->moved_mm_.y += current_mm_point.y() - first_mm_point.y();
}
