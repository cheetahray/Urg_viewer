/*!
  \file
  \brief URG のデータ表示アプリケーション

  \author Satofumi Kamimura

  $Id$
*/

#include <cmath>
#include <QCloseEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QSettings>
#include <QDateTime>
#include <QShortcut>
#include <QTimer>
#include <QUrl>
#include <QFileInfo>
#include "Urg_viewer_window.h"
#include "Connection_widget.h"
#include "Serial_connection_widget.h"
#include "Ethernet_connection_widget.h"
#include "handle_ethernet_setting.h"
#include "Step_value_widget.h"
#include "Plotter_2d_widget.h"
#include "Recorder_widget.h"
#include "Player_widget.h"
#include "Scan_setting_widget.h"
#include "Connect_thread.h"
#include "Receive_thread.h"
#include "Scan_setting.h"
#include "Receive_recorder.h"
#include "Urg_log_reader.h"
#include "product_utils.h"
#include "Serial.h"
#include "Tcpip.h"
#include "convert_path_codec.h"
#include "plugin.h"

using namespace hrk;
using namespace std;


namespace
{
    const char* Organization_ = "Hobby Robot SDK";
    const char* Application_ = "URG Viewer";

    const string Version = "2.0.7";
    const string Application_title = "Urg Viewer";

    const char* Completer_file = "address.txt";

    typedef vector<State*> State_forms;

    enum {
        Urg_port_number = 10940,
        Default_redraw_msec = 100,
        Invalid_step = -1,
    };

    const double Play_max_speed = 8.0;


    class Plugin_handler
    {
    public:
        Plugin_handler(void) : is_log_file_opened_(false)
        {
        }


        ~Plugin_handler(void)
        {
            close_log_file();
        }


        void load_plugin_file(const char* plugin_file)
        {
            plguin_load_plugin_file(plugin_file);
        }


        void open_log_file(const char* log_file)
        {
            is_log_file_opened_ = plugin_open_log_file(log_file);
        }


        void close_log_file(void)
        {
            if (is_log_file_opened_) {
                plugin_close_log_file();
                is_log_file_opened_ = false;
            }
        }


        void log_index_updated(int index)
        {
            plugin_log_index_updated(index);
        }


    public:
        bool is_log_file_opened_;
    };
}


struct Urg_viewer_window::pImpl
{
    Urg_viewer_window* widget_;
    Urg_driver urg_;
    QString application_title_;
    Connection_widget connection_widget_;
    Serial_connection_widget* serial_connection_widget_;
    Ethernet_connection_widget* ethernet_connection_widget_;
    Step_value_widget step_value_widget_;
    Plotter_2d_widget plotter_2d_widget_;
    Recorder_widget recorder_widget_;
    Player_widget player_widget_;
    Scan_setting_widget scan_setting_widget_;

    Urg_log_reader urg_log_reader_;
    QTimer redraw_timer_;
    Connect_thread connect_thread_;
    Receive_thread receive_thread_;
    State_forms state_forms_;
    State::state_t current_state_;
    State::state_t next_state_;
    Scan_setting next_scan_setting_;
    int next_scan_interval_;
    QStringList completer_address_;
    string last_ethernet_address_;

    string error_message_;
    Connection* original_connection_;
    Receive_recorder receive_recorder_;
    QString last_access_folder_;
    QString recording_file_;

    bool is_pausing_;
    double play_speed_magnification_;
    QString first_play_file_;
    int last_clicked_step_;

    string last_device_or_address_;
    long last_baudrate_or_port_;
    Urg_driver::connection_t last_type_;
    int connect_retry_count_;
    bool load_default_when_connected_;

    Plugin_handler plugin_;


    pImpl(Urg_viewer_window* widget)
        : widget_(widget),
          serial_connection_widget_(connection_widget_.serial()),
          ethernet_connection_widget_(connection_widget_.ethernet()),
          plotter_2d_widget_(urg_, step_value_widget_),
          connect_thread_(urg_),
          receive_thread_(urg_, urg_log_reader_, plotter_2d_widget_),
          next_scan_interval_(0),
          original_connection_(NULL), is_pausing_(false),
          play_speed_magnification_(1.0), last_clicked_step_(Invalid_step),
          connect_retry_count_(0), load_default_when_connected_(true)
    {
        redraw_timer_.setInterval(Default_redraw_msec);
        state_forms_.push_back(&plotter_2d_widget_);
        state_forms_.push_back(&step_value_widget_);
        state_forms_.push_back(&recorder_widget_);
        state_forms_.push_back(&player_widget_);
    }


    void initialize_form(void)
    {
        // タイトルにバージョン情報を付加
        application_title_ = widget_->windowTitle();
        set_version_title();

        widget_->connection_layout_->addWidget(&connection_widget_);
        widget_->main_layout_->addWidget(&step_value_widget_);
        widget_->main_layout_->addWidget(&plotter_2d_widget_);
        widget_->under_layout_->addWidget(&player_widget_);
        widget_->under_layout_->addWidget(&recorder_widget_);

        // フォームの初期化
        serial_connection_widget_->set_baudrate_visible(false);
        ethernet_connection_widget_->set_port_visible(false);
        set_not_connected();

        ethernet_connection_widget_->set_port_number(Urg_port_number);
        widget_->csv_progress_bar_->hide();

        // signals about menu
        connect(widget_->action_quit_, SIGNAL(triggered()),
                widget_, SLOT(close()));
        connect(widget_->action_about_, SIGNAL(triggered()),
                widget_, SLOT(show_about()));

        // signals about connection
        connect(widget_->change_button_, SIGNAL(clicked()),
                widget_, SLOT(change_clicked()));
        connect(serial_connection_widget_, SIGNAL(rescan_clicked()),
                widget_, SLOT(rescan_clicked()));
        connect(serial_connection_widget_,
                SIGNAL(connect_clicked(const std::string&, long)),
                widget_,
                SLOT(serial_connect_clicked(const std::string&, long)));
        connect(ethernet_connection_widget_,
                SIGNAL(connect_clicked(const std::string&, long)),
                widget_,
                SLOT(ethernet_connect_clicked(const std::string&, long)));
        connect(serial_connection_widget_, SIGNAL(disconnect_clicked()),
                widget_, SLOT(disconnect_clicked()));
        connect(ethernet_connection_widget_, SIGNAL(disconnect_clicked()),
                widget_, SLOT(disconnect_clicked()));
        connect(&connect_thread_, SIGNAL(connected()),
                widget_, SLOT(urg_connected()));
        connect(&connect_thread_, SIGNAL(connect_failed(const QString&)),
                widget_, SLOT(urg_connect_failed(const QString&)));

        // signals about playing & recording
        connect(&player_widget_, SIGNAL(folder_clicked()),
                widget_, SLOT(folder_clicked()));
        connect(&recorder_widget_, SIGNAL(record_clicked()),
                widget_, SLOT(record_clicked()));
        connect(widget_->action_record_, SIGNAL(triggered()),
                widget_, SLOT(record_clicked()));
        connect(&recorder_widget_, SIGNAL(stop_clicked()),
                widget_, SLOT(record_stop_clicked()));
        connect(widget_->action_open_, SIGNAL(triggered()),
                widget_, SLOT(folder_clicked()));
        connect(widget_->action_stop_, SIGNAL(triggered()),
                widget_, SLOT(escape_pressed()));

        connect(&receive_thread_, SIGNAL(played(long, long)),
                &player_widget_, SLOT(set_play_time(long, long)));
        connect(&receive_thread_, SIGNAL(played(long, long)),
                widget_, SLOT(notify_play_time(long, long)));
        connect(&receive_thread_, SIGNAL(recorded(long, long)),
                &plotter_2d_widget_, SLOT(recorded(long, long)));
        connect(&player_widget_, SIGNAL(play_clicked()),
                widget_, SLOT(start_playing_clicked()));
        connect(&player_widget_, SIGNAL(pause_clicked()),
                widget_, SLOT(pause_playing_clicked()));
        connect(&receive_thread_, SIGNAL(play_completed()),
                widget_, SLOT(play_completed()));
        connect(&player_widget_, SIGNAL(slider_changed(int)),
                widget_, SLOT(slider_changed(int)));
        connect(widget_->action_scan_forward_1_, SIGNAL(triggered()),
                widget_, SLOT(scan_forward_1()));
        connect(widget_->action_scan_forward_10_, SIGNAL(triggered()),
                widget_, SLOT(scan_forward_10()));
        connect(widget_->action_scan_back_1_, SIGNAL(triggered()),
                widget_, SLOT(scan_back_1()));
        connect(widget_->action_scan_back_10_, SIGNAL(triggered()),
                widget_, SLOT(scan_back_10()));
        connect(widget_->action_play_faster_, SIGNAL(triggered()),
                widget_, SLOT(play_faster()));
        connect(widget_->action_play_slower_, SIGNAL(triggered()),
                widget_, SLOT(play_slower()));

        // data receiving
        connect(&redraw_timer_, SIGNAL(timeout()),
                widget_, SLOT(length_data_received()));
        connect(&player_widget_, SIGNAL(stop_clicked()),
                widget_, SLOT(stop_playing_clicked()));
        connect(&receive_thread_, SIGNAL(receive_failed(const char*)),
                widget_, SLOT(receive_failed(const char*)));

        // signals about data showing
        connect(&receive_thread_, SIGNAL(received()),
                widget_, SLOT(length_data_received()));
        connect(&step_value_widget_, SIGNAL(config_changed(bool, bool)),
                widget_, SLOT(scan_config_changed(bool, bool)));
        connect(&step_value_widget_, SIGNAL(update_requested()),
                &plotter_2d_widget_, SLOT(step_value_requested()));
        connect(&plotter_2d_widget_, SIGNAL(clicked_mm_point(long, long)),
                widget_, SLOT(clicked_mm_point(long, long)));
        connect(widget_, SIGNAL(show_step_data(int)),
                &step_value_widget_, SLOT(show_step_data(int)));
        connect(widget_, SIGNAL(show_step_data(int)),
                &plotter_2d_widget_, SLOT(show_step_data(int)));
        connect(&step_value_widget_, SIGNAL(selection_changed(int)),
                &plotter_2d_widget_, SLOT(show_step_data(int)));
        connect(widget_->action_zoom_out_, SIGNAL(triggered()),
                &plotter_2d_widget_, SLOT(zoom_out()));
        connect(widget_->action_zoom_in_, SIGNAL(triggered()),
                &plotter_2d_widget_, SLOT(zoom_in()));

        // signal about save as csv
        connect(widget_->action_save_as_csv_, SIGNAL(triggered()),
                widget_, SLOT(save_as_csv()));
        connect(&receive_thread_, SIGNAL(csv_recording_completed()),
                widget_, SLOT(input_csv_save_file()));
        connect(&receive_thread_, SIGNAL(csv_recording_percent(int)),
                widget_->csv_progress_bar_, SLOT(setValue(int)));

        // signal about window show/hide
        connect(widget_->action_data_value_window_, SIGNAL(triggered(bool)),
                widget_, SLOT(set_step_value_visible(bool)));
        connect(widget_->action_scan_setting_window_, SIGNAL(triggered(bool)),
                widget_, SLOT(set_scan_setting_visible(bool)));

        // signal about set_step_value_auto_update
        connect(&step_value_widget_, SIGNAL(auto_update_changed(bool)),
                widget_, SLOT(auto_update_chaned(bool)));

        // signal about scan_setting_widget
        connect(&scan_setting_widget_, SIGNAL(close_nortify()),
                widget_, SLOT(set_scan_setting_visible()));
        connect(&scan_setting_widget_, SIGNAL(quit_nortify()),
                widget_, SLOT(close()));
        connect(&scan_setting_widget_,
                SIGNAL(scan_setting_updated(const Scan_setting&, int)),
                widget_, SLOT(update_scan_setting(const Scan_setting&, int)));

        // short cut
        widget_->action_open_->
            setShortcut(QApplication::translate("Urg_viewer_window_form",
                                                "Ctrl+O", 0,
                                                QApplication::UnicodeUTF8));
        new QShortcut(Qt::Key_Escape, widget_, SLOT(escape_pressed()));

        plugin_register_plotter(&plotter_2d_widget_);

        // 描画の開始
        redraw_timer_.start();
    }


    void load_settings(void)
    {
        QSettings settings(Organization_, Application_);

        widget_->restoreGeometry(settings.value("geometry").toByteArray());
        scan_setting_widget_.
            restoreGeometry(settings. value("setting_geometry").toByteArray());

        int connection_value =
            settings.value("connection_type",
                           Connection_widget::Serial).toInt();
        Connection_widget::connection_t connection_type =
            static_cast<Connection_widget::connection_t>(connection_value);
        if (connection_type != connection_widget_.connection_type()) {
            change_clicked();
        }
        rescan_serial_devices();
        load_ethernet_setting(settings, ethernet_connection_widget_);
        completer_address_ = load_complete_address(Completer_file);
        ethernet_connection_widget_->set_completer_address(completer_address_);
        connection_widget_.setFocus();

        bool with_intensity = settings.value("with_intensity", false).toBool();
        bool is_multiecho = settings.value("is_multiecho", false).toBool();
        step_value_widget_.set_config(with_intensity, is_multiecho);

        last_access_folder_ = settings.value("save_folder", ".").toString();

        bool data_value_visible =
            settings.value("data_value_visible", true).toBool();
        widget_->action_data_value_window_->setChecked(data_value_visible);
        widget_->set_step_value_visible(data_value_visible);
        bool scan_setting_visible =
            settings.value("scan_setting_visible", false).toBool();
        widget_->action_scan_setting_window_->setChecked(scan_setting_visible);
        scan_setting_widget_.setVisible(scan_setting_visible);

        bool auto_update = settings.value("auto_update", false).toBool();
        step_value_widget_.set_auto_update(auto_update);
    }


    void save_settings(void)
    {
        QSettings settings(Organization_, Application_);

        settings.setValue("geometry", widget_->saveGeometry());
        settings.setValue("setting_geometry",
                          scan_setting_widget_.saveGeometry());

        settings.setValue("connection_type",
                          connection_widget_.connection_type());
        save_ethernet_setting(settings, ethernet_connection_widget_);

        settings.setValue("with_intensity", next_scan_setting_.with_intensity);
        settings.setValue("is_multiecho", next_scan_setting_.is_multiecho);

        settings.setValue("save_folder", last_access_folder_);

        settings.setValue("data_value_visible",
                          widget_->action_data_value_window_->isChecked());
        settings.setValue("scan_setting_visible",
                          widget_->action_scan_setting_window_->isChecked());

        settings.setValue("auto_update",
                          step_value_widget_.auto_update());
    }


    void save_last_connected_address(void)
    {
        // 今までにない IP address の場合、追加して保存する
        if (!last_ethernet_address_.empty()) {
            QString connected_address = last_ethernet_address_.c_str();
            if (!completer_address_.contains(connected_address)) {
                completer_address_ << connected_address;
                save_complete_address(Completer_file, completer_address_);
            }
        }
    }


    void change_clicked(void)
    {
        connection_widget_.toggle_connection_type();

        Connection_widget::connection_t current_type =
            connection_widget_.connection_type();
        if (current_type == Connection_widget::Serial) {
            widget_->change_button_->setText(tr("Change to Ethernet"));
            rescan_serial_devices();
        } else {
            widget_->change_button_->setText(tr("Change to Serial"));
        }
    }


    void set_state_forms(State::state_t state)
    {
        for (State_forms::iterator it = state_forms_.begin();
             it != state_forms_.end(); ++it) {
            (*it)->set_state(state);
        }

        bool hide_under_spacer = false;
        bool change_button_enable = false;
        bool set_connection_disabled = false;
        bool save_as_csv_enabled = false;
        bool record_enable = false;
        bool stop_enable = false;
        bool setting_changable = false;
        switch (state) {
        case State::Not_connected:
            change_button_enable = true;
            set_connection_disabled = true;
            break;

        case State::Viewing:
            save_as_csv_enabled = true;
            record_enable = true;
            setting_changable = true;
            break;

        case State::Playing:
            hide_under_spacer = true;
            set_connection_disabled = true;
            save_as_csv_enabled = true;
            stop_enable = true;
            player_widget_.set_focus(Player_widget::Play);
            break;

        case State::Recording:
            stop_enable = true;
            recorder_widget_.setFocus();
            break;
        }

        widget_->change_button_->setEnabled(change_button_enable);
        set_enable_spacer(!hide_under_spacer);
        widget_->action_save_as_csv_->setEnabled(save_as_csv_enabled);

        if (set_connection_disabled) {
            connection_widget_.set_connected(false);
        }

        bool is_not_playing = (state != State::Playing) ? true : false;
        connection_widget_.setEnabled(is_not_playing);
        set_play_menu_enabled(!is_not_playing, record_enable, stop_enable);

        bool is_viewing = (state != State::Not_connected) ? true : false;
        set_zoom_enabled(is_viewing);

        scan_setting_widget_.set_control_enabled(setting_changable);

        current_state_ = state;
    }


    void set_zoom_enabled(bool enable)
    {
        widget_->action_zoom_out_->setEnabled(enable);
        widget_->action_zoom_in_->setEnabled(enable);
    }


    void rescan_serial_devices(void)
    {
        vector<string> device_names = Urg_driver::find_ports();
        for (vector<string>::iterator it = device_names.begin();
             it != device_names.end(); ++it) {
            if (Urg_driver::is_urg_port(*it)) {
                *it = *it + " [URG]";
            }
        }
        serial_connection_widget_->set_device_names(device_names);
    }


    void connect_device(const string& device_or_address,
                        long baudrate_or_port, Urg_driver::connection_t type)
    {
        connect_thread_.set_open_setting(device_or_address,
                                         baudrate_or_port, type);

        last_device_or_address_ = device_or_address;
        last_baudrate_or_port_ = baudrate_or_port;
        last_type_ = type;

        connect_thread_.start();
    }


    void reconnect_by_previous_setting(void)
    {
        connect_device(last_device_or_address_, last_baudrate_or_port_,
                       last_type_);
    }


    void start_measurement(void)
    {
        // 計測と受信を開始する
        set_scan_parameter();
        start_receiving();
    }


    void stop_measurement(void)
    {
        stop_receiving();
        stop_csv_recording();
    }


    void restart_measurement(void)
    {
        stop_measurement();
        start_measurement();
    }


    void start_receiving(void)
    {
        redraw_timer_.stop();
        receive_thread_.start();
    }


    void stop_receiving(void)
    {
        receive_thread_.stop();
        receive_thread_.wait();
        plotter_2d_widget_.wait();

        redraw_timer_.start();
    }


    void stop_csv_recording(void)
    {
        if (current_state_ == State::Playing) {
            player_widget_.set_folder_enabled(true);
            player_widget_.set_pause_enabled(true);
            if (is_pausing_) {
                receive_thread_.pause();
            }
        }
        widget_->csv_progress_bar_->setVisible(false);
    }


    void set_scan_parameter(void)
    {
        step_value_widget_.set_max_echo_size(urg_.max_echo_size());

        int scan_steps =
            next_scan_setting_.last_step - next_scan_setting_.first_step + 1;
        step_value_widget_.set_steps(scan_steps, next_scan_setting_.first_step);

        plotter_2d_widget_.set_scan_setting(next_scan_setting_);
        receive_thread_.set_scan_setting(next_scan_setting_,
                                         next_scan_interval_);
    }


    void load_default_scan_setting(void)
    {
        next_scan_setting_.first_step = urg_.min_step();
        next_scan_setting_.last_step = urg_.max_step();
        next_scan_setting_.group_steps = 1;
        next_scan_interval_ = 0;
    }


    bool start_recording(void)
    {
        QString save_timing = QDateTime::currentDateTime().
            toString("yyyyMMdd_hhmmss");
        recording_file_ = "received_" + save_timing + ".txt";

        original_connection_ = urg_.connection();
        if (!receive_recorder_.open(original_connection_,
                                    recording_file_.toStdString().c_str())) {
            error_message_ = receive_recorder_.what();
            return false;
        }

        if (!urg_.open(&receive_recorder_)) {
            return false;
        }

        receive_thread_.set_mode(Receive_thread::Recording);
        start_receiving();

        return true;
    }


    void stop_recording(void)
    {
        stop_measurement();
        urg_.close();
        receive_recorder_.close();

        delete original_connection_;
        original_connection_ = NULL;

        recorder_widget_.stop_recording();
        receive_thread_.set_mode(Receive_thread::Normal);
    }


    void set_enable_spacer(bool enable)
    {
        QSizePolicy::Policy policy =
            enable ? QSizePolicy::Expanding : QSizePolicy::Minimum;
        widget_->under_spacer_->changeSize(0, 0, policy);
    }


    void set_product_name_title(void)
    {
        QString title = application_title_ +
            "  [ " + urg_.sensor_product_type().c_str() +
            " " + urg_.sensor_product_version().c_str() + " ]";
        widget_->setWindowTitle(title);
    }


    void set_version_title(void)
    {
        const QString title = application_title_ + "  " + Version.c_str();
        widget_->setWindowTitle(title);
    }


    void set_playfile_title(const QString& file_name)
    {
        const QString title = application_title_ + "  [ " + file_name + " ]";
        widget_->setWindowTitle(title);
    }


    void restrict_config_by_product_name(void)
    {
        typedef struct
        {
            const char* name;
            bool with_intensity;
            bool is_multiecho;
        } restrict_config_t;

        restrict_config_t list[] = {
            { "URG-04LX", false, false },
            { "URG-04LX-UG01", false, false },
            { "UBG-04LX-F01", false, false },

            { "UTM-30LX", true, false },
        };

        string product_name = urg_.sensor_product_type();
        size_t n = sizeof(list) / sizeof(list[0]);
        for (size_t i = 0; i < n; ++i) {
            restrict_config_t& product = list[i];
            if (product_name == product.name) {
                restrict_config(product.with_intensity, product.is_multiecho);
                return;
            }
        }

        // with_intensity, is_multiecho の操作を許可する
        restrict_config(true, true);
    }


    void restrict_config(bool with_intensity, bool is_multiecho)
    {
        bool next_with_intensity = with_intensity ?
            next_scan_setting_.with_intensity : false;
        bool next_is_multiecho = is_multiecho ?
            next_scan_setting_.is_multiecho : false;

        step_value_widget_.set_config(next_with_intensity, next_is_multiecho);
        step_value_widget_.
            restrict_config_buttons(with_intensity, is_multiecho);
    }


    QString save_recorded_file(void)
    {
        // 記録後に Viewing を再開するときには、
        // センサから設定を読み直さないようにする
        load_default_when_connected_ = false;

        // 保存するファイル名の入力
        QString save_file = QFileDialog::
            getSaveFileName(widget_, tr("save log file"),
                            last_access_folder_ + "/" + recording_file_,
                            tr("txt files (*.txt);;all files (*.*)"),
                            NULL, QFileDialog::DontConfirmOverwrite);

        // ファイル名の入力がキャンセルされたら、ファイルを削除する
        QDir recorded_file(recording_file_);
        if (save_file.isEmpty()) {
            QFile saved_file(recording_file_);
            saved_file.remove();
            return "";
        }

        // 記録したファイル名と異なる名前が入力されたら、ファイルを rename する
        QDir inputed_file(save_file);
        QString save_file_path = inputed_file.absolutePath();
        QString recorded_file_path = recorded_file.absolutePath();
        if (save_file_path != recorded_file_path) {
            recorded_file.rename(recorded_file_path, save_file_path);
        }
        QFileInfo dir(save_file);
        last_access_folder_ = dir.absoluteDir().path();

        return save_file;
    }


    bool open_play_file(const QString& file_name)
    {
        if (file_name.isEmpty()) {
            return false;
        }

        string std_file_path = std_string_path(file_name);
        if (!urg_log_reader_.load(std_file_path.c_str())) {
            return false;
        }

        // plugin の読み込み
        plugin_.load_plugin_file("plugin.lua");
        plugin_.open_log_file(std_file_path.c_str());

        QFileInfo file_info(file_name);
        set_playfile_title(file_info.baseName());
        return true;
    }


    void set_play_speed(double play_speed_magnification)
    {
        receive_thread_.set_play_speed(play_speed_magnification);
        plotter_2d_widget_.
            set_message(tr("Play speed x %1").arg(play_speed_magnification));
        plotter_2d_widget_.set_icon(Plotter_2d_widget::Playing);
    }


    void reset_play_speed(void)
    {
        play_speed_magnification_ = 1.0;
        set_play_speed(play_speed_magnification_);
    }


    void set_play_menu_enabled(bool enable,
                               bool record_enable, bool stop_enable)
    {
        widget_->action_record_->setEnabled(record_enable);
        widget_->action_play_->setEnabled(enable);
        widget_->action_stop_->setEnabled(stop_enable);
        widget_->action_pause_->setEnabled(enable);

        widget_->action_scan_forward_1_->setEnabled(enable);
        widget_->action_scan_back_1_->setEnabled(enable);
        widget_->action_scan_forward_10_->setEnabled(enable);
        widget_->action_scan_back_10_->setEnabled(enable);

        widget_->action_play_faster_->setEnabled(enable);
        widget_->action_play_slower_->setEnabled(enable);
    }


    void stop_viewing(void)
    {

        stop_measurement();
        urg_.close();
    }


    bool start_playing(void)
    {
        // データ取得の設定を各フォームに登録する
        bool with_intensity;
        bool is_multiecho;
        urg_log_reader_.log_measurement_type(with_intensity, is_multiecho);

        int total_play_second = urg_log_reader_.total_sec();
        int total_play_scans = urg_log_reader_.total_scans();
        step_value_widget_.set_config(with_intensity, is_multiecho);
        receive_thread_.set_mode(Receive_thread::Seekable);

        if (!urg_.open(&urg_log_reader_)) {
            return false;
        }

        urg_log_reader_.log_range(next_scan_setting_.first_step,
                                  next_scan_setting_.last_step,
                                  next_scan_setting_.group_steps);
        int scan_times = 0;
        urg_log_reader_.log_scans(scan_times, next_scan_interval_);

        next_scan_interval_ = 0;
        receive_thread_.set_scan_setting(next_scan_setting_,
                                         next_scan_interval_);

        set_scan_parameter();

        const string product_type = urg_.sensor_product_type();
        double timestamp_unit = product_timestamp_unit(urg_);
        total_play_second /= timestamp_unit;
        player_widget_.set_total_play_time(total_play_second, total_play_scans);

        is_pausing_ = false;
        pause_playing();
        start_receiving();
        redraw_timer_.start();

        if (last_clicked_step_ != Invalid_step) {
            emit widget_->show_step_data(last_clicked_step_);
        }

        return true;
    }


    void stop_playing(void)
    {
        stop_measurement();
        receive_thread_.resume();
        urg_.close();
        urg_log_reader_.close();
        receive_thread_.set_mode(Receive_thread::Normal);
        plotter_2d_widget_.clear_message();

        plugin_.close_log_file();
    }


    void pause_playing(void)
    {
        if (!is_pausing_) {
            is_pausing_ = true;
            receive_thread_.pause();
            plotter_2d_widget_.set_icon(Plotter_2d_widget::Pausing);
            player_widget_.set_focus(Player_widget::Play);
        }
    }


    void resume_playing()
    {
        if (is_pausing_) {
            is_pausing_ = false;
            receive_thread_.resume();
            plotter_2d_widget_.set_icon(Plotter_2d_widget::Playing);
        }
    }


    void play_completed(void)
    {
        receive_thread_.wait();
        plotter_2d_widget_.wait();

        urg_log_reader_.reload();
        start_playing();
    }


    void update_drawing_data(void)
    {
        if (is_pausing_) {
            receive_thread_.receive_one_scan();
        }
    }


    bool set_play_file(const QString& file_name)
    {
        if (!open_play_file(file_name)) {
            return false;
        }

        last_clicked_step_ = Invalid_step;
        reset_play_speed();
        set_state_forms(State::Playing);
        start_playing();

        return true;
    }


    void set_not_connected(void)
    {
        set_version_title();
        set_state_forms(State::Not_connected);
        connection_widget_.setFocus();
        plotter_2d_widget_.clear_message();

        if (connection_widget_.connection_type() == Connection_widget::Serial) {
            rescan_serial_devices();
        }
    }


    void show_log_file_error_message(const QString& file_path)
    {
        QFileInfo file_info(file_path);
        QString file_name = file_info.fileName();
        QMessageBox::warning(widget_, tr("Urg Viewer"),
                             tr("invalid log file. %1:").arg(file_name) +
                             urg_log_reader_.what(),
                             QMessageBox::Ok);
    }
};


Urg_viewer_window::Urg_viewer_window(void) : pimpl(new pImpl(this))
{
    setupUi(this);
    pimpl->initialize_form();
    pimpl->load_settings();
}


Urg_viewer_window::~Urg_viewer_window(void)
{
    pimpl->save_settings();
}


bool Urg_viewer_window::load_play_file(const char* file_path)
{
    pimpl->first_play_file_ = file_path;
    QFileInfo file_info(pimpl->first_play_file_);
    return file_info.exists();
}


void Urg_viewer_window::closeEvent(QCloseEvent* event)
{
    pimpl->scan_setting_widget_.hide();

    pimpl->plugin_.close_log_file();
    pimpl->stop_viewing();
    pimpl->redraw_timer_.stop();
    event->accept();
}


void Urg_viewer_window::showEvent(QShowEvent* event)
{
    static_cast<void>(event);

    if (!pimpl->first_play_file_.isEmpty()) {
        if (!pimpl->set_play_file(pimpl->first_play_file_)) {
            pimpl->show_log_file_error_message(pimpl->first_play_file_);
        }
        pimpl->first_play_file_.clear();
    }
}


void Urg_viewer_window::dragEnterEvent(QDragEnterEvent* event)
{
    if (!pimpl->player_widget_.is_folder_enabled()) {
        return;
    }
    event->acceptProposedAction();
}


void Urg_viewer_window::dragMoveEvent(QDragEnterEvent* event)
{
    if (!pimpl->player_widget_.is_folder_enabled()) {
        return;
    }
    event->acceptProposedAction();
}


void Urg_viewer_window::dropEvent(QDropEvent* event)
{
    if (!pimpl->player_widget_.is_folder_enabled()) {
        return;
    }

    const QMimeData* mime_data = event->mimeData();
    if (!mime_data->hasUrls()) {
        return;
    }

    pimpl->stop_playing();
    QList<QUrl> url_list = mime_data->urls();
    for (int i = 0; i < url_list.count(); ++i) {
        QString file_path = url_list.at(i).path();

        // 先頭の '/' を取り除いて処理する
        file_path.replace(0, 1, "");
        if (!pimpl->set_play_file(file_path)) {
            pimpl->show_log_file_error_message(file_path);
        }
        return;
    }
}


void Urg_viewer_window::update_scan_setting(const Scan_setting& setting,
                                            int scan_interval)
{
    // 計測設定の変更を指示
    pimpl->next_scan_setting_.first_step = setting.first_step;
    pimpl->next_scan_setting_.last_step = setting.last_step;
    pimpl->next_scan_setting_.group_steps = setting.group_steps;
    pimpl->next_scan_interval_ = scan_interval;

    scan_config_changed(pimpl->next_scan_setting_.with_intensity,
                        pimpl->next_scan_setting_.is_multiecho);
}


void Urg_viewer_window::show_about(void)
{
    QString message =
        "<h2>URG Viewer " + QString(Version.c_str()) + "</h2>"
        "<p>The viewer application for URG sensor.</p>"
        "<p>Report bugs to &lt;satofumi@users.sourceforge.jp&gt;</p>";

    QMessageBox::about(this, tr("About Urg Viewer"), message);
}


void Urg_viewer_window::change_clicked(void)
{
    pimpl->change_clicked();
}


void Urg_viewer_window::serial_connect_clicked(const std::string& device,
                                               long baudrate)
{
    pimpl->connect_retry_count_ = 0;
    pimpl->last_ethernet_address_ = "";
    pimpl->next_state_ = State::Viewing;
    string actual_device_name = device.substr(0, device.find(' '));
    pimpl->connect_device(actual_device_name, baudrate, Urg_driver::Serial);
}


void Urg_viewer_window::ethernet_connect_clicked(const std::string& address,
                                                 long port)
{
    pimpl->connect_retry_count_ = 0;
    pimpl->last_ethernet_address_ = address;
    pimpl->next_state_ = State::Viewing;
    pimpl->connect_device(address, port, Urg_driver::Ethernet);
}


void Urg_viewer_window::disconnect_clicked(void)
{
    // 記録中の切断では、ファイルの保存を行う
    if (pimpl->current_state_ == State::Recording) {
        record_stop_clicked();
        return;
    }

    plugin_close_device();

    pimpl->stop_measurement();
    pimpl->urg_.close();
    pimpl->set_state_forms(State::Not_connected);
    pimpl->connection_widget_.set_connected(false);
    pimpl->step_value_widget_.restrict_config_buttons(true, true);
    pimpl->set_version_title();
}


void Urg_viewer_window::rescan_clicked(void)
{
    pimpl->rescan_serial_devices();
}


void Urg_viewer_window::folder_clicked(void)
{
    QString open_file =
        QFileDialog::getOpenFileName(this, tr("load log file"),
                                     pimpl->last_access_folder_,
                                     tr("text files (*.txt);;all files (*.*)"));
    if (open_file.isEmpty()) {
        // Cancel された場合は、状態を変更せずに戻る
        return;
    }

    pimpl->stop_playing();
    if (!pimpl->set_play_file(open_file)) {
        pimpl->show_log_file_error_message(open_file);
        return;
    }

    QFileInfo dir(open_file);
    pimpl->last_access_folder_ = dir.absoluteDir().path();
}


void Urg_viewer_window::record_clicked(void)
{
    pimpl->set_state_forms(State::Recording);
    pimpl->stop_measurement();
    if (!pimpl->start_recording()) {
        pimpl->set_state_forms(State::Not_connected);

        QMessageBox::warning(this, tr("Recording error"),
                             pimpl->error_message_.c_str());
        return;
    }
}


void Urg_viewer_window::record_stop_clicked(void)
{
    pimpl->stop_recording();
    QString save_file_name = pimpl->save_recorded_file();

#if 1
    static_cast<void>(save_file_name);

    // 直前の設定でセンサに接続する
    pimpl->reconnect_by_previous_setting();
#else
    // Playing 用にファイルを開く
    if (!pimpl->set_play_file(save_file_name)) {
        pimpl->set_state_forms(State::Not_connected);
    }
#endif
}


void Urg_viewer_window::urg_connected(void)
{
    switch (pimpl->next_state_) {
    case State::Viewing:
        pimpl->restrict_config_by_product_name();
        pimpl->scan_setting_widget_.load_sensor_setting(pimpl->urg_);
        if (pimpl->load_default_when_connected_) {
            pimpl->load_default_scan_setting();
        } else {
            pimpl->load_default_when_connected_ = true;
        }
        // throught down

    case State::Recording:
        pimpl->set_product_name_title();
        pimpl->set_state_forms(pimpl->next_state_);
        pimpl->plotter_2d_widget_.step_value_requested();
        pimpl->start_measurement();
        break;

    case State::Playing:
    case State::Not_connected:
        break;
    }
    pimpl->connection_widget_.set_connected(true);
    pimpl->save_last_connected_address();

    pimpl->plugin_.load_plugin_file("plugin.lua");
    plugin_open_device();
}


void Urg_viewer_window::urg_connect_failed(const QString& error_message)
{
    pimpl->connection_widget_.set_connected(false);
    QMessageBox::warning(this, tr("Connection error"), error_message);
}


void Urg_viewer_window::scan_config_changed(bool with_intensity,
                                            bool is_multiecho)
{
    pimpl->next_scan_setting_.with_intensity = with_intensity;
    pimpl->next_scan_setting_.is_multiecho = is_multiecho;

    switch (pimpl->current_state_) {
    case State::Viewing:
        pimpl->plotter_2d_widget_.step_value_requested();
        pimpl->restart_measurement();
        break;

    case State::Playing:
    case State::Recording:
    case State::Not_connected:
        // 何もしない
        break;
    }
}


void Urg_viewer_window::length_data_received(void)
{
    pimpl->plotter_2d_widget_.redraw();
}


void Urg_viewer_window::clicked_mm_point(long mm_x, long mm_y)
{
    if (!pimpl->urg_.is_open()) {
        return;
    }

    // クリック位置に対応する step を求める
    bool is_multiecho = pimpl->next_scan_setting_.is_multiecho;
    int max_echo_size = pimpl->urg_.max_echo_size();
    int echo_size = is_multiecho ? max_echo_size : 1;
    int last_index = pimpl->urg_.max_step() * echo_size;
    const double min_radian = pimpl->urg_.index2rad(0);
    const double max_radian = pimpl->urg_.index2rad(last_index);

    const double clicked_radian = atan2(-mm_x, mm_y);
    if ((clicked_radian < min_radian) || (clicked_radian > max_radian)) {
        // 計測範囲外のクリックなので、戻る
        return;
    }
    int current_step = pimpl->urg_.rad2index(clicked_radian);
    if (is_multiecho) {
        current_step /= max_echo_size;
    }

    emit show_step_data(current_step);
    pimpl->last_clicked_step_ = current_step;
}


void Urg_viewer_window::escape_pressed(void)
{
    switch (pimpl->current_state_) {
    case State::Playing:
        stop_playing_clicked();
        break;

    case State::Recording:
        record_stop_clicked();
        break;

    case State::Viewing:
        pimpl->stop_viewing();
        pimpl->set_not_connected();
        break;

    case State::Not_connected:
        // 何もしない
        break;
    }
}


void Urg_viewer_window::stop_playing_clicked(void)
{
    if (!pimpl->is_pausing_) {
        pimpl->pause_playing();
        pimpl->player_widget_.set_focus(Player_widget::Stop);

    } else {
        if (pimpl->player_widget_.slider_value() == 0) {
            // 再生するファイルが読み込まれた状態で停止が指示された場合、
            // 再生のモードを終了する
            pimpl->stop_playing();
            pimpl->set_not_connected();

        } else {
            // 再生するファイルが読み込まれた状態に戻す
            pimpl->receive_thread_.stop();
            pimpl->play_completed();
        }
    }
}


void Urg_viewer_window::start_playing_clicked(void)
{
    pimpl->resume_playing();
    pimpl->player_widget_.set_focus(Player_widget::Pause);
}


void Urg_viewer_window::pause_playing_clicked(void)
{
    pimpl->pause_playing();
    pimpl->player_widget_.set_focus(Player_widget::Play);
}


void Urg_viewer_window::play_completed(void)
{
    pimpl->play_completed();
    pimpl->player_widget_.set_focus(Player_widget::Play);
}


void Urg_viewer_window::slider_changed(int index)
{
    pimpl->receive_thread_.set_play_scan_index(index);
    pimpl->update_drawing_data();
}


void Urg_viewer_window::scan_forward_1(void)
{
    pimpl->receive_thread_.change_play_scan_index(+1);
    pimpl->update_drawing_data();
}


void Urg_viewer_window::scan_back_1(void)
{
    pimpl->receive_thread_.change_play_scan_index(-1);
    pimpl->update_drawing_data();
}


void Urg_viewer_window::scan_forward_10(void)
{
    pimpl->receive_thread_.change_play_scan_index(+10);
    pimpl->update_drawing_data();
}


void Urg_viewer_window::scan_back_10(void)
{
    pimpl->receive_thread_.change_play_scan_index(-10);
    pimpl->update_drawing_data();
}


void Urg_viewer_window::play_faster(void)
{
    pimpl->play_speed_magnification_
        = min(pimpl->play_speed_magnification_ * 2.0, Play_max_speed);

    pimpl->set_play_speed(pimpl->play_speed_magnification_);
}


void Urg_viewer_window::play_slower(void)
{
    pimpl->play_speed_magnification_
        = max(1.0, pimpl->play_speed_magnification_ / 2.0);
    pimpl->set_play_speed(pimpl->play_speed_magnification_);
}


void Urg_viewer_window::save_as_csv(void)
{
    // 記録中の場合は、戻る
    if (csv_progress_bar_->isVisible()) {
        return;
    }

    if (pimpl->current_state_ == State::Viewing) {
        // Step_value_widget の操作ができないようにする
        pimpl->step_value_widget_.set_state(State::Recording);

    } else if (pimpl->current_state_ == State::Playing) {
        // ポーズできないようにした上でポーズを解除し、記録を強制する
        pimpl->player_widget_.set_pause_enabled(false);
        pimpl->player_widget_.set_folder_enabled(false);
        pimpl->receive_thread_.resume();
    }

    csv_progress_bar_->setVisible(true);
    pimpl->receive_thread_.start_csv_recording();
}


void Urg_viewer_window::input_csv_save_file(void)
{
    // 記録前に変更したフォーム設定を元に戻す
    pimpl->stop_csv_recording();
    pimpl->step_value_widget_.set_state(pimpl->current_state_);

    QString save_file = QFileDialog::
        getSaveFileName(this, tr("save csv file"),
                        pimpl->last_access_folder_ + "/received.csv",
                        tr("csv files (*.csv);;all files (*.*)"));

    if (!save_file.isEmpty()) {
        string std_file_path = std_string_path(save_file);
        pimpl->receive_thread_.save_csv_file(std_file_path.c_str());
    }
    csv_progress_bar_->setVisible(false);
}


void Urg_viewer_window::set_step_value_visible(bool checked)
{
    pimpl->step_value_widget_.setVisible(checked);
}


void Urg_viewer_window::set_scan_setting_visible(bool checked)
{
    pimpl->scan_setting_widget_.setVisible(checked);
    action_scan_setting_window_->setChecked(checked);
}


void Urg_viewer_window::auto_update_chaned(bool auto_update)
{
    pimpl->plotter_2d_widget_.set_step_value_auto_update(auto_update);
}


void Urg_viewer_window::receive_failed(const char* message)
{
    if (pimpl->urg_.is_booting()) {
        enum { Max_booting_retry_times = 4 };
        if (++pimpl->connect_retry_count_ < Max_booting_retry_times) {
            pimpl->urg_.close();
            pimpl->reconnect_by_previous_setting();
            return;
        }
    }

    QMessageBox::warning(this, tr("Receiving error"), message);

    switch (pimpl->current_state_) {
    case State::Playing:
        pimpl->stop_playing();
        break;

    case State::Recording:
        pimpl->stop_recording();
        break;

    case State::Viewing:
        pimpl->stop_viewing();
        break;

    case State::Not_connected:
        // 何もしない
        break;
    }

    pimpl->set_not_connected();
}


void Urg_viewer_window::notify_play_time(long second, long index)
{
    static_cast<void>(second);

    plugin_log_index_updated(index);
}
