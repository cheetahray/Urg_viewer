/*!
  \file
  \brief データの受信用スレッド

  \author Satofumi Kamimura

  $Id$
*/

#include <cmath>
#include <QMutex>
#include <QTime>
#include "Receive_thread.h"
#include "Plotter_2d_widget.h"
#include "Scan_setting.h"
#include "Urg_driver.h"
#include "Urg_log_reader.h"
#include "Csv_recorder.h"
#include "product_utils.h"
#include "plugin.h"

using namespace hrk;
using namespace std;


namespace
{
    enum {
        Invalid_scan_index = -1,
    };
}


struct Receive_thread::pImpl
{
    Receive_thread* thread_;
    Urg_driver& urg_;
    Urg_log_reader& urg_log_reader_;
    Plotter_2d_widget& plotter_2d_widget_;
    mode_t mode_;
    QMutex mutex_;
    bool quit_;
    bool pause_;
    bool receive_one_scan_;

    Scan_setting setting_;
    int scan_interval_;

    int next_scan_index_;
    int add_scan_index_;
    double play_speed_magnification_;

    Csv_recorder csv_recorder_;
    size_t csv_recording_scans_;


    pImpl(Receive_thread* thread,
          Urg_driver& urg, Urg_log_reader& urg_log_reader,
          Plotter_2d_widget& plotter_2d_widget)
        : thread_(thread), urg_(urg), urg_log_reader_(urg_log_reader),
          plotter_2d_widget_(plotter_2d_widget),
          mode_(Normal), quit_(false), pause_(false), receive_one_scan_(false),
          scan_interval_(0),
          next_scan_index_(Invalid_scan_index), add_scan_index_(0),
          play_speed_magnification_(1.0), csv_recording_scans_(0)
    {
    }


    void receive_thread(void)
    {
        quit_ = false;
        next_scan_index_ = 0;
        const long scan_msec = urg_.scan_usec() / 1000.0;
        double timestamp_unit = product_timestamp_unit(urg_);

        // 計測の開始
        if (!start_scanning(true)) {
            emit thread_->receive_failed(urg_.what());
            return;
        }
        Lidar::measurement_t type = measurement_type();

        enum {
            Retry_timeout_msec = 1000,
            Retry_wait_msec = 100,
        };

        int retry_count = 0;
        vector<long> distance;
        vector<unsigned short> intensity;
        long timestamp;
        bool is_pause = false;
        size_t scan_count = 0;
        size_t loss_count = 0;
        size_t max_scan_count = urg_log_reader_.total_scans();
        size_t total_recording_scans = 0;
        size_t left_recording_scans = 0;
        double play_speed_magnification = 1.0;
        long previous_timestamp = 0;
        QTime cycle_timer;
        int consecutive_loss_times = 0;

        while (true) {
            msleep(1);

            if (!is_pause) {
                // データの受信
                if (!receive_data(distance, intensity, timestamp)) {
                    if (mode_ == Seekable) {
                        emit thread_->play_completed();
                        if (left_recording_scans > 0) {
                            emit thread_->csv_recording_completed();
                        }
                        return;
                    }

                    // 数秒のリトライ後に計測をあきらめる
                    ++retry_count;
                    if ((retry_count * Retry_wait_msec) > Retry_timeout_msec) {
                        emit thread_->receive_failed(urg_.what());
                        return;
                    }
                    msleep(Retry_wait_msec);

                    if (!start_scanning(false)) {
                        emit thread_->receive_failed(urg_.what());
                        return;
                    }
                    continue;
                }

                if (mode_ == Seekable) {
                    long total_play_second;
                    long msec_to_next_scan;
                    urg_log_reader_.scan_data(scan_count,
                                              total_play_second,
                                              msec_to_next_scan);
                    msec_to_next_scan /= timestamp_unit;
                    msleep(msec_to_next_scan / play_speed_magnification);
                    emit thread_->played(total_play_second / timestamp_unit,
                                         scan_count);
                }
                ++scan_count;

                // CSV 保存のためのデータ登録
                if (left_recording_scans > 0) {
                    csv_recorder_.set_receive_data(distance, intensity);
                    if (--left_recording_scans == 0) {
                        emit thread_->csv_recording_percent(100);
                        emit thread_->csv_recording_completed();
                    } else {
                        int percent = 100 - (100.0 * left_recording_scans
                                             / total_recording_scans);
                        emit thread_->csv_recording_percent(percent);
                    }
                }

                // 描画のためのデータ登録
                long msec_timestamp = timestamp / timestamp_unit;
                plugin_get_measurement_data(type, distance.size(),
                                            &distance[0], &intensity[0],
                                            msec_timestamp);

                // データを登録すると distance, intensity は空になる
                plotter_2d_widget_.
                    set_plot_data(type, distance, intensity, msec_timestamp);
                if (mode_ == Recording) {
                    emit thread_->recorded(scan_count, loss_count);
                }

                // データの受信が遅れているときは、再描画を促さないようにする
                bool not_redraw = false;
                if ((mode_ == Recording) || (mode_ == Normal)) {
                    unsigned short diff = timestamp - previous_timestamp;
                    previous_timestamp = timestamp;
                    diff /= timestamp_unit;

                    if (scan_count > 1) {
                        loss_count += round(abs(diff - scan_msec) / scan_msec)
                            / (scan_interval_ + 1);
                    }

                    if (cycle_timer.elapsed() < (scan_msec / 2)) {
                        not_redraw = true;
                    }
                    cycle_timer.restart();
                }

                enum { Force_draw_interval = 10 };
                if (!not_redraw ||
                    (consecutive_loss_times >= Force_draw_interval)) {
                    consecutive_loss_times = 0;
                    emit thread_->received();
                }
                ++consecutive_loss_times;
            }

            QMutexLocker locker(&mutex_);
            if (quit_) {
                break;
            }

            is_pause = (receive_one_scan_) ? false : pause_;
            receive_one_scan_ = false;
            if (mode_ == Seekable) {
                play_speed_magnification = play_speed_magnification_;
            }

            if (csv_recording_scans_ > 0) {
                total_recording_scans = csv_recording_scans_;
                left_recording_scans = csv_recording_scans_;
                csv_recording_scans_ = 0;
            }

            if (add_scan_index_ != 0) {
                int next_index =
                    static_cast<int>(scan_count) + add_scan_index_ - 1;
                if ((scan_count == 0) && (add_scan_index_ > 0)) {
                    ++next_index;
                }
                next_scan_index_ = max(0, next_index);
                add_scan_index_ = 0;
            }
            if (next_scan_index_ != Invalid_scan_index) {
                if (next_scan_index_ >= static_cast<int>(max_scan_count)) {
                    next_scan_index_ = max_scan_count - 1;
                }
                if (urg_log_reader_.set_next_scans(next_scan_index_)) {
                    scan_count = next_scan_index_;
                }
                next_scan_index_ = Invalid_scan_index;
            }
        }

        // 計測停止コマンドの発行
        urg_.stop_measurement();

        if (mode_ == Recording) {
            emit thread_->recorded(0, 0);
        }
    }


    bool start_scanning(bool range_updated)
    {
        if (range_updated) {
            urg_.set_scanning_parameter(setting_.first_step, setting_.last_step,
                                        setting_.group_steps);
        }
        Lidar::measurement_t type = measurement_type();
        if (!urg_.start_measurement(type, Urg_driver::Infinity_scan_times,
                                    scan_interval_)) {
            return false;
        }
        return true;
    }


    Lidar::measurement_t measurement_type(void)
    {
        Lidar::measurement_t type;
        if (setting_.is_multiecho) {
            if (setting_.with_intensity) {
                type = Lidar::Multiecho_intensity;
            } else {
                type = Lidar::Multiecho;
            }
        } else {
            if (setting_.with_intensity) {
                type = Lidar::Distance_intensity;
            } else {
                type = Lidar::Distance;
            }
        }
        return type;
    }


    bool receive_data(vector<long>& distance,
                      vector<unsigned short>& intensity, long& timestamp)
    {
        bool ret;

        if (setting_.with_intensity) {
            if (setting_.is_multiecho) {
                ret = urg_.get_multiecho_intensity(distance, intensity,
                                                   &timestamp);
            } else {
                ret = urg_.get_distance_intensity(distance, intensity,
                                                  &timestamp);
            }
        } else {
            intensity.clear();

            if (setting_.is_multiecho) {
                ret = urg_.get_multiecho(distance, &timestamp);
            } else {
                ret = urg_.get_distance(distance, &timestamp);
            }
        }

        return ret;
    }


    void start_csv_recording(void)
    {
        csv_recorder_.set_scan_setting(urg_.sensor_product_type(),
                                       setting_, urg_.max_echo_size());
        csv_recording_scans_ = csv_recorder_.recordable_scan_times();
    }
};


Receive_thread::Receive_thread(hrk::Urg_driver& urg,
                               hrk::Urg_log_reader& urg_log_reader,
                               Plotter_2d_widget& plotter_2d_widget)
    : pimpl(new pImpl(this, urg, urg_log_reader, plotter_2d_widget))
{
}


Receive_thread::~Receive_thread(void)
{
}


void Receive_thread::set_mode(mode_t mode)
{
    pimpl->mode_ = mode;

    pimpl->next_scan_index_ = Invalid_scan_index;
    pimpl->add_scan_index_ = 0;
}


void Receive_thread::set_scan_setting(const Scan_setting& setting,
                                      int scan_interval)
{
    pimpl->setting_ = setting;
    pimpl->scan_interval_ = scan_interval;
}


void Receive_thread::set_play_speed(double magnification)
{
    pimpl->mutex_.lock();
    pimpl->play_speed_magnification_ = magnification;
    pimpl->mutex_.unlock();
}


void Receive_thread::run(void)
{
    pimpl->receive_thread();
}


void Receive_thread::stop(void)
{
    pimpl->mutex_.lock();
    pimpl->quit_ = true;
    pimpl->pause_ = false;
    pimpl->mutex_.unlock();
}


void Receive_thread::pause(void)
{
    pimpl->mutex_.lock();
    pimpl->pause_ = true;
    pimpl->mutex_.unlock();
}


void Receive_thread::resume(void)
{
    pimpl->mutex_.lock();
    pimpl->pause_ = false;
    pimpl->mutex_.unlock();
}


void Receive_thread::set_play_scan_index(int index)
{
    pimpl->mutex_.lock();
    pimpl->next_scan_index_ = index;
    pimpl->mutex_.unlock();
}


void Receive_thread::change_play_scan_index(int add_scan_index)
{
    pimpl->mutex_.lock();
    pimpl->add_scan_index_ = add_scan_index;
    pimpl->mutex_.unlock();
}


void Receive_thread::receive_one_scan(void)
{
    pimpl->mutex_.lock();
    pimpl->receive_one_scan_ = true;
    pimpl->mutex_.unlock();
}


void Receive_thread::start_csv_recording(void)
{
    pimpl->mutex_.lock();
    pimpl->start_csv_recording();
    pimpl->mutex_.unlock();
}


void Receive_thread::save_csv_file(const char* file_path)
{
    pimpl->csv_recorder_.save_file(file_path);
}
