#ifndef RECEIVE_THREAD_H
#define RECEIVE_THREAD_H

/*!
  \file
  \brief データの受信用スレッド

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <QThread>

namespace hrk
{
    class Urg_driver;
    class Urg_log_reader;
}

class Scan_setting;
class Plotter_2d_widget;


class Receive_thread : public QThread
{
    Q_OBJECT;

 public:
    typedef enum {
        Normal,
        Seekable,
        Recording,
    } mode_t;

    Receive_thread(hrk::Urg_driver& urg, hrk::Urg_log_reader& urg_log_reader,
                   Plotter_2d_widget& plotter_2d_widget);
    ~Receive_thread(void);

    void set_mode(mode_t mode);
    void set_scan_setting(const Scan_setting& setting, int scan_interval);
    void set_play_speed(double magnification);
    void run(void);
    void stop(void);
    void pause(void);
    void resume(void);
    void set_play_scan_index(int index);
    void change_play_scan_index(int add_scan_index);
    void receive_one_scan(void);
    void start_csv_recording(void);
    void save_csv_file(const char* file_path);

 signals:
    void receive_failed(const char* error_message);
    void received(void);
    void played(long total_second, long scan_index);
    void recorded(long recorded_times, long loss_times);
    void play_completed(void);
    void csv_recording_percent(int percent);
    void csv_recording_completed(void);

 private:
    Receive_thread(void);
    Receive_thread(const Receive_thread& rhs);
    Receive_thread& operator = (const Receive_thread& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
