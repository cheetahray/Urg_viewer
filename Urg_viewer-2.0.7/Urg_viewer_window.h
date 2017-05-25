#ifndef URG_VIEWER_WINDOW_H
#define URG_VIEWER_WINDOW_H

/*!
  \file
  \brief URG のデータ表示アプリケーション

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "ui_Urg_viewer_window_form.h"

class Scan_setting;


class Urg_viewer_window
    : public QMainWindow, private Ui::Urg_viewer_window_form
{
    Q_OBJECT;

 public:
    Urg_viewer_window(void);
    ~Urg_viewer_window(void);

    bool load_play_file(const char* file_path);

 protected:
    void closeEvent(QCloseEvent* event);
    void showEvent(QShowEvent* event);
    void dragEnterEvent(QDragEnterEvent* event);
    void dragMoveEvent(QDragEnterEvent* event);
    void dropEvent(QDropEvent* event);

 signals:
    void show_step_data(int step);

 public slots:
    void update_scan_setting(const Scan_setting& setting, int scan_interval);

 private slots:
    void show_about(void);
    void change_clicked(void);
    void serial_connect_clicked(const std::string& device, long baudrate);
    void ethernet_connect_clicked(const std::string& address, long port);
    void disconnect_clicked(void);
    void rescan_clicked(void);
    void folder_clicked(void);
    void record_clicked(void);
    void record_stop_clicked(void);
    void urg_connected(void);
    void urg_connect_failed(const QString& error_message);
    void scan_config_changed(bool with_intensity, bool is_multiecho);
    void length_data_received(void);
    void clicked_mm_point(long mm_x, long mm_y);
    void escape_pressed(void);
    void stop_playing_clicked(void);
    void start_playing_clicked(void);
    void pause_playing_clicked(void);
    void play_completed(void);
    void slider_changed(int index);
    void scan_forward_1(void);
    void scan_back_1(void);
    void scan_forward_10(void);
    void scan_back_10(void);
    void play_faster(void);
    void play_slower(void);
    void save_as_csv(void);
    void input_csv_save_file(void);
    void set_step_value_visible(bool checked);
    void set_scan_setting_visible(bool checked = false);
    void auto_update_chaned(bool auto_update);
    void receive_failed(const char* message);
    void notify_play_time(long second, long index);

 private:
    Urg_viewer_window(const Urg_viewer_window& rhs);
    Urg_viewer_window& operator = (const Urg_viewer_window& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
