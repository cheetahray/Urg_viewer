/*!
  \file
  \brief Urg センサ

  \author Satofumi Kamimura

  $Id$
*/

#include <cstdlib>
#include <cstring>
#include <cmath>
#include "Urg_driver.h"
#include "Tcpip.h"
#include "Serial.h"
#include "connection_utils.h"

using namespace hrk;
using namespace std;


namespace
{
    enum {
        Expected_end = -1,
        Max_timeout = 140,
        Buffer_size = 64 + 2 + 6,
        Urg_max_echo = 3,

        Stop = Lidar::Multiecho_intensity + 16,
    };


    typedef enum {
        Urg_no_error = 0,
        Urg_unknown_error = -1000,
        Urg_not_connected,
        Urg_send_error,
        Urg_receive_error,
        Urg_no_response_error,
        Urg_invalid_response_error,
        Urg_checksum_error,
        Urg_not_detect_baudrate_error,
        Urg_invalid_parameter_error,
        Urg_invalid_state_error,
    } urg_error_t;


    typedef enum {
        Unknown,

        Booting,
        Idle,
        Sleep,
        Waking_up,
        Time_adjustment,
        Single_scan,
        Multi_scan,

        LN_Booting,
        LN_Measurement,
        LN_Responding,

        Product_responding,

        Error_detected,
        Firmware_update,
        Development,
    } state_t;


    typedef struct
    {
        int front_index;
        int first_index;
        int last_index;
        int area_resolution;
        long scan_usec;
        long min_distance;
        long max_distance;
    } sensor_parameter_t;


    typedef struct
    {
        bool is_multiecho;
        int first_step;
        int last_step;
        int skip_step;
        int skip_scan;
        int scan_times;
        int timeout;
    } indicated_setting_t;


    typedef struct
    {
        bool is_multiecho;
        int first_index;
        int last_index;
        int range_data_byte;
        int skip_step;
    } received_setting_t;


    long decode_scip(const char data[], int size)
    {
        const char* p = data;
        const char* last_p = p + size;

        int value = 0;
        while (p < last_p) {
            value <<= 6;
            value &= ~0x3f;
            value |= *p++ - 0x30;
        }
        return value;
    }
}


struct Urg_driver::pImpl
{
    string error_message_;
    Connection* created_connection_;
    Connection* connection_;
    int sensor_timeout_;
    bool is_receiving_;
    bool is_laser_on_;
    int remain_scan_times_;
    int skip_scan_;
    measurement_t measurement_type_;
    sensor_parameter_t sensor_;
    indicated_setting_t indicated_;
    received_setting_t received_;
    string sensor_product_type_;
    string sensor_product_version_;
    string sensor_product_serial_id_;
    bool is_booting_error_;


    pImpl(void)
        : error_message_("no error."),
          created_connection_(NULL), connection_(NULL),
          sensor_timeout_(Max_timeout),
          is_receiving_(true), is_laser_on_(false),
          remain_scan_times_(0), skip_scan_(0),
          measurement_type_(Distance), is_booting_error_(false)
    {
        indicated_.timeout = 0;

        received_.is_multiecho = false;
        received_.first_index = 0;
        received_.last_index = 0;
        received_.range_data_byte = 3;
        received_.skip_step = 0;
    }


    bool set_errno_and_return(urg_error_t error_number)
    {
        typedef struct
        {
            urg_error_t error_number;
            const char* message;
        } error_message_t;

        const error_message_t messages[] = {
            { Urg_no_error, "no error." },
            { Urg_unknown_error, "unknown error." },
            { Urg_not_connected, "sensor is not opened." },
            { Urg_send_error, "send error." },
            { Urg_receive_error, "receive error." },
            { Urg_no_response_error, "no response." },
            { Urg_invalid_response_error, "invalid response." },
            { Urg_checksum_error, "checksum error." },
            { Urg_not_detect_baudrate_error, "could not connect URG sensor." },
            { Urg_invalid_parameter_error, "invalid command parameter." },
            { Urg_invalid_state_error, "could not measurement in this state." },
        };

        size_t n = sizeof(messages) / sizeof(messages[0]);
        size_t i;
        for (i = 0; i < n; ++i) {

            if (messages[i].error_number == error_number) {
                error_message_ = messages[i].message;
                break;
            }
        }
        if (i >= n) {
            error_message_ = "unknown error.";
        }

        return (error_number == Urg_no_error) ? true : false;
    }


    bool open(const char* device_name_or_ip_address,
              long baudrate_or_port_number, connection_t type)
    {
        if (!open_device(device_name_or_ip_address,
                         baudrate_or_port_number, type)) {
            close();
            return false;
        }

        long urg_baudrate = (type == Ethernet) ?
            115200 : baudrate_or_port_number;
        if (!connect_urg_device(urg_baudrate)) {
            close();
            return false;
        }
        connection_ = created_connection_;

        return update_sensor_parameter();
    }


    bool open(Connection* connection)
    {
        enum { Urg_baudrate = 115200 };
        created_connection_ = connection;
        if (!connect_urg_device(Urg_baudrate)) {
            created_connection_ = NULL;
            close();
            return false;
        }
        connection_ = created_connection_;
        created_connection_ = NULL;

        return update_sensor_parameter();
    }


    void close(void)
    {
        if (connection_) {
            connection_->close();
        }
        if (created_connection_) {
            delete created_connection_;
            created_connection_ = NULL;
        }
        connection_ = NULL;
        sensor_product_type_.clear();
        sensor_product_version_.clear();
        sensor_product_serial_id_.clear();
    }


    bool open_device(const char* device_name_or_ip_address,
                     long baudrate_or_port_number, connection_t type)
    {
        if (type == Serial) {
            hrk::Serial* serial_connection = new hrk::Serial;
            if (!serial_connection->open(device_name_or_ip_address,
                                         baudrate_or_port_number)) {
                error_message_ = serial_connection->what();
                return false;
            }
            created_connection_ = serial_connection;

        } else if (type == Ethernet) {
            Tcpip* tcpip_connection = new Tcpip;
            if (!tcpip_connection->open(device_name_or_ip_address,
                                        baudrate_or_port_number)) {
                error_message_ = tcpip_connection->what();
                return false;
            }
            created_connection_ = tcpip_connection;

        } else {
            error_message_ = "unknown connection type.";
            return false;
        }

        return true;
    }


    //! チェックサムの計算
    char scip_checksum(const char buffer[], int size)
    {
        unsigned char sum = 0x00;
        for (int i = 0; i < size; ++i) {
            sum += buffer[i];
        }

        // 計算の意味は SCIP 仕様書を参照のこと
        return (sum & 0x3f) + 0x30;
    }


    // SCIP として受信データを処理し、受信した応答の行数を返す
    int scip_response(Connection* connection, const char* command,
                      const int expected_ret[], int timeout,
                      char *receive_buffer, int receive_buffer_max_size)
    {
        size_t write_size = strlen(command);
        int n = connection->write(command, write_size);
        if (n != static_cast<int>(write_size)) {
            return set_errno_and_return(Urg_send_error);
        }

        char *p = receive_buffer;
        if (p) {
            *p = '\0';
        }

        int filled_size = 0;
        int line_number = 0;
        int ret = Urg_no_error;
        do {
            char buffer[Buffer_size];
            n = readline(connection, buffer, Buffer_size, timeout);
            if (n < 0) {
                return set_errno_and_return(Urg_no_response_error);

            } else if (p && (line_number > 0)
                       && (n < (receive_buffer_max_size - filled_size))) {
                // エコーバックは別途、一致しているかのチェックを行うため、
                // 返すデータには格納しない
                memcpy(p, buffer, n);
                p += n;
                *p++ = '\0';
                filled_size += n;
            }

            if (line_number == 0) {
                // エコーバック文字列が、一致するかを確認する
                if (strncmp(buffer, command, write_size - 1)) {
                    return set_errno_and_return(Urg_invalid_response_error);
                }
            } else if (n > 0) {
                // エコーバック以外の行のチェックサムを評価する
                char checksum = buffer[n - 1];
                if ((checksum != scip_checksum(buffer, n - 1)) &&
                    (checksum != scip_checksum(buffer, n - 2))) {
                    return set_errno_and_return(Urg_checksum_error);
                }
            }

            // ステータス応答を評価して、戻り値を決定する
            if (line_number == 1) {
                if (n == 1) {
                    // SCIP 1.1 応答の場合は、正常応答とみなす
                    ret = 0;

                } else if (n != 3) {
                    return set_errno_and_return(Urg_invalid_response_error);

                } else {
                    int actual_ret = strtol(buffer, NULL, 10);
                    for (int i = 0; expected_ret[i] != Expected_end; ++i) {
                        if (expected_ret[i] == actual_ret) {
                            ret = Urg_no_error;
                            break;
                        }
                    }
                }
            }

            ++line_number;
        } while (n > 0);

        is_receiving_ = false;
        return (ret != Urg_no_error) ? ret : (line_number - 1);
    }


    bool connect_urg_device(long urg_baudrate)
    {
        long try_baudrate[] = { 19200, 38400, 115200 };
        size_t try_times = sizeof(try_baudrate) / sizeof(try_baudrate[0]);

        // 指示されたボーレートから接続を試す
        for (size_t i = 0; i < try_times; ++i) {
            if (try_baudrate[i] == urg_baudrate) {
                swap(try_baudrate[0], try_baudrate[i]);
                break;
            }
        }

        for (size_t i = 0; i < try_times; ++i) {
            enum { Receive_buffer_size = 4 };
            char receive_buffer[Receive_buffer_size + 1];

            created_connection_->change_baudrate(try_baudrate[i]);

            // QT を送信し、応答が返されるかでボーレートが
            // 一致しているかを確認する
            int qt_expected[] = { 0, Expected_end };
            int ret = scip_response(created_connection_,
                                    "QT\n", qt_expected, Max_timeout,
                                    receive_buffer, Receive_buffer_size);
            if (ret >= 0) {
                if (!strcmp(receive_buffer, "E")) {
                    int scip20_expected[] = { 0, Expected_end };

                    // QT 応答の最後の改行を読み飛ばす
                    ignore(created_connection_, Max_timeout);

                    // "E" が返された場合は、SCIP 1.1 とみなして
                    // "SCIP2.0" を送信する
                    ret = scip_response(created_connection_,
                                        "SCIP2.0\n", scip20_expected,
                                        Max_timeout, NULL, 0);

                    // SCIP2.0 応答の最後の改行を読み飛ばす
                    ignore(created_connection_, Max_timeout);

                    // ボーレートを変更して戻る
                    return change_sensor_baudrate(try_baudrate[i],
                                                  urg_baudrate);

                } else if (!strcmp(receive_buffer, "0Ee")) {
                    int tm2_expected[] = { 0, Expected_end };

                    // "0Ee" が返された場合は、TM モードとみなして
                    // "TM2" を送信する
                    scip_response(created_connection_,
                                  "TM2\n", tm2_expected, Max_timeout, NULL, 0);

                    // ボーレートを変更して戻る
                    return change_sensor_baudrate(try_baudrate[i],
                                                  urg_baudrate);
                }
            }

            if (ret <= 0) {
                if (ret == Urg_invalid_response_error) {
                    // 異常なエコーバックのときは、距離データ受信中とみなして
                    // データを読み飛ばす
                    send_qt_and_ignore_response(created_connection_,
                                                Max_timeout);

                    // ボーレートを変更して戻る
                    return change_sensor_baudrate(try_baudrate[i],
                                                  urg_baudrate);

                } else {
                    // 応答がないときは、ボーレートを変更して、再度接続を行う
                    ignore(created_connection_, Max_timeout);
                    continue;
                }
            } else if (!strcmp("00P", receive_buffer)) {
                // センサとホストのボーレートを変更して戻る
                return change_sensor_baudrate(try_baudrate[i], urg_baudrate);
            }
        }

        return set_errno_and_return(Urg_not_detect_baudrate_error);
    }


    bool change_sensor_baudrate(long current_baudrate, long next_baudrate)
    {
        if (current_baudrate == next_baudrate) {
            // 現在のボーレートと設定するボーレートが一緒ならば、戻る
            return set_errno_and_return(Urg_no_error);
        }

        // "SS" コマンドでボーレートを変更する
        enum { SS_command_size = 10 };
        char buffer[SS_command_size];
        int ss_expected[] = { 0, 3, 4, Expected_end };
        snprintf(buffer, SS_command_size, "SS%06ld\n", next_baudrate);
        int ret = scip_response(created_connection_,
                                buffer, ss_expected, sensor_timeout_, NULL, 0);

        // 0F 応答のときは Ethernet 用のセンサとみなし、正常応答を返す
        if (ret == -15) {
            return set_errno_and_return(Urg_no_error);
        }
        if (ret <= 0) {
            return set_errno_and_return(Urg_invalid_parameter_error);
        }

        // 正常応答ならば、ホスト側のボーレートを変更する
        ret = created_connection_->change_baudrate(next_baudrate);

        // センサ側の設定反映を待つために少しだけ待機する
        ignore(created_connection_, Max_timeout);

        return set_errno_and_return(Urg_no_error);
    }


    bool update_sensor_parameter(void)
    {
        enum {
            Receive_buffer_size = Buffer_size * 9,
            PP_response_lines = 10,
        };
        char receive_buffer[Receive_buffer_size];
        int pp_expected[] = { 0, Expected_end };
        int ret = scip_response(connection_, "PP\n", pp_expected, Max_timeout,
                                receive_buffer, Receive_buffer_size);
        if (ret < 0) {
            return false;
        } else if (ret < PP_response_lines) {
            send_qt_and_ignore_response(connection_, Max_timeout);
            return set_errno_and_return(Urg_invalid_response_error);
        }

        unsigned short received_bits = 0x0000;
        const char* p = receive_buffer;
        for (int i = 0; i < (ret - 1); ++i) {
            if (!strncmp(p, "MODL:", 5)) {
                int n = strlen(&p[5]);
                if (n > 2) {
                    sensor_product_type_ = string(&p[5]).substr(0, n - 2);
                    sensor_product_type_ =
                        remove_sub_information(sensor_product_type_);
                }

            } else if (!strncmp(p, "DMIN:", 5)) {
                sensor_.min_distance = strtol(p + 5, NULL, 10);
                received_bits |= 0x0001;

            } else if (!strncmp(p, "DMAX:", 5)) {
                sensor_.max_distance = strtol(p + 5, NULL, 10);
                received_bits |= 0x0002;

            } else if (!strncmp(p, "ARES:", 5)) {
                sensor_.area_resolution = strtol(p + 5, NULL, 10);
                received_bits |= 0x0004;

            } else if (!strncmp(p, "AMIN:", 5)) {
                sensor_.first_index = strtol(p + 5, NULL, 10);
                received_bits |= 0x0008;

            } else if (!strncmp(p, "AMAX:", 5)) {
                sensor_.last_index = strtol(p + 5, NULL, 10);
                received_bits |= 0x0010;

            } else if (!strncmp(p, "AFRT:", 5)) {
                sensor_.front_index = strtol(p + 5, NULL, 10);
                received_bits |= 0x0020;

            } else if (!strncmp(p, "SCAN:", 5)) {
                int rpm = strtol(p + 5, NULL, 10);
                // タイムアウト時間は、計測周期の 16 倍程度の値にする
                sensor_.scan_usec = 1000 * 1000 * 60 / rpm;
                if (indicated_.timeout > 0) {
                    sensor_timeout_ = indicated_.timeout;
                } else {
                    sensor_timeout_ = sensor_.scan_usec >> (10 - 4);
                }
                received_bits |= 0x0040;
            }
            p += strlen(p) + 1;
        }

        // 全てのパラメータを受信したか確認
        if (received_bits != 0x007f) {
            return set_errno_and_return(Urg_receive_error);
        }

        set_scanning_parameter(sensor_.first_index, sensor_.last_index, 1);

        return set_errno_and_return(Urg_no_error);
    }


    bool update_vv_information(void)
    {
        enum {
            Receive_buffer_size = Buffer_size * 7,
            VV_response_lines = 7,
        };
        char receive_buffer[Receive_buffer_size];
        int vv_expected[] = { 0, Expected_end };
        int ret = scip_response(connection_, "VV\n", vv_expected, Max_timeout,
                                receive_buffer, Receive_buffer_size);
        if (ret < 0) {
            return false;
        } else if (ret < VV_response_lines) {
            send_qt_and_ignore_response(connection_, Max_timeout);
            return set_errno_and_return(Urg_invalid_response_error);
        }

        const char* p = receive_buffer;
        for (int i = 0; i < (ret - 1); ++i) {
            if (!strncmp(p, "FIRM:", 5)) {
                int n = strlen(&p[5]);
                if (n > 2) {
                    sensor_product_version_ = string(&p[5]).substr(0, n - 2);
                    sensor_product_version_ =
                        remove_sub_information(sensor_product_version_);
                }
            } else if (!strncmp(p, "SERI:", 5)) {
                int n = strlen(&p[5]);
                if (n > 2) {
                    sensor_product_serial_id_ = string(&p[5]).substr(0, n - 2);
                }
            }
            p += strlen(p) + 1;
        }
        return true;
    }


    bool is_open(void)
    {
        return (connection_) ? connection_->is_open() : false;
    }


    void send_qt_and_ignore_response(Connection* connection, int timeout)
    {
        if (!is_receiving_) {
            // データを受信中でなければ、QT を送信せずに戻る
            return;
        }

        connection->write("QT\n", 3);
        ignore(connection, timeout);
        is_receiving_ = false;
    }


    bool start_measurement(measurement_t type, int scan_times, int skip_scan)
    {
        if ((skip_scan < 0) || (skip_scan > 9)) {
            send_qt_and_ignore_response(connection_, sensor_timeout_);
            return set_errno_and_return(Urg_invalid_parameter_error);
        }

        // 指定されたタイプのパケットを生成し、送信する
        bool ret = false;
        switch (type) {
        case Distance:
            ret = send_distance_command(scan_times, skip_scan,
                                        'G', 'M', 'D');
            break;

        case Distance_intensity:
            ret = send_distance_command(scan_times, skip_scan,
                                        'G', 'M', 'E');
            break;

        case Multiecho:
            ret = send_distance_command(scan_times, skip_scan,
                                        'H', 'N', 'D');
            break;

        case Multiecho_intensity:
            ret = send_distance_command(scan_times, skip_scan,
                                        'H', 'N', 'E');
            break;
        }

        indicated_.skip_scan = skip_scan;
        if (ret) {
            measurement_type_ = type;
        }
        return ret;
    }


    void stop_measurement(void)
    {
        if (!is_open()) {
            set_errno_and_return(Urg_not_connected);
            return;
        }

        // QT を発行する
        int n = connection_->write("QT\n", 3);
        if (n != 3) {
            set_errno_and_return(Urg_send_error);
            return;
        }

        enum { Max_read_times = 6 };
        for (int i = 0; i < Max_read_times; ++i) {
            // QT の応答が返されるまで、距離データを読み捨てる
            int ret = receive_data(NULL, NULL, NULL);
            if (ret == Urg_no_error) {
                // 正常応答
                is_laser_on_ = false;
                is_receiving_ = false;
                set_errno_and_return(Urg_no_error);
                return;
            }
        }
    }


    bool send_distance_command(int scan_times, int skip_scan,
                               char single_scan_ch, char continuous_scan_ch,
                               char scan_type_ch)
    {
        indicated_.scan_times = (scan_times < 0) ? 0 : scan_times;
        remain_scan_times_ = indicated_.scan_times;
        skip_scan_ = (skip_scan < 0) ? 0 : skip_scan;
        if (scan_times >= 100) {
            // 計測回数が 99 を越える場合は、無限回のスキャンを行う
            indicated_.scan_times = 0;
        }

        char buffer[Buffer_size];
        int write_size;
        if (remain_scan_times_ == 1) {
            // レーザ発光を指示
            if (!turn_on_laser()) {
                return false;
            }

            write_size = snprintf(buffer, Buffer_size, "%c%c%04d%04d%02d\n",
                                  single_scan_ch, scan_type_ch,
                                  indicated_.first_step,
                                  indicated_.last_step,
                                  indicated_.skip_step);
        } else {
            write_size = snprintf(buffer, Buffer_size,
                                  "%c%c%04d%04d%02d%01d%02d\n",
                                  continuous_scan_ch, scan_type_ch,
                                  indicated_.first_step,
                                  indicated_.last_step,
                                  indicated_.skip_step,
                                  skip_scan, indicated_.scan_times);
            is_receiving_ = true;
        }

        int n = connection_->write(buffer, write_size);
        if (n != write_size) {
            return set_errno_and_return(Urg_send_error);
        }

        return true;
    }


    bool set_scanning_parameter(int first_step, int last_step, int skip_step)
    {
        if ((first_step > last_step) || (first_step < sensor_.first_index) ||
            (last_step > sensor_.last_index) ||
            (skip_step > 99) || (skip_step < 0)) {
            return false;
        }

        indicated_.first_step = first_step;
        indicated_.last_step = last_step;
        indicated_.skip_step = skip_step;

        return true;
    }


    bool turn_on_laser(void)
    {
        if (is_laser_on_) {
            // 既にレーザが発光しているときは、コマンドを送信しない
            return set_errno_and_return(Urg_no_error);
        }

        int expected[] = { 0, 2, Expected_end };
        int ret = scip_response(connection_,
                                "BM\n", expected, sensor_timeout_, NULL, 0);
        if (ret >= 0) {
            is_laser_on_ = true;
            return set_errno_and_return(Urg_no_error);
        } else {
            return set_errno_and_return(Urg_send_error);
        }
    }


    int receive_data(long data[], unsigned short intensity[], long *time_stamp)
    {
        is_booting_error_ = false;
        int extended_timeout = sensor_timeout_
            + 2 * (sensor_.scan_usec * (indicated_.skip_scan) / 1000);

        // エコーバックの取得
        char buffer[Buffer_size];
        int n = readline(connection_, buffer, Buffer_size, extended_timeout);
        if (n <= 0) {
            return set_errno_and_return(Urg_no_response_error);
        }

        // エコーバックの解析
        Lidar::measurement_t type =
            static_cast<Lidar::measurement_t>(parse_distance_echoback(buffer));

        // 応答の取得
        n = readline(connection_, buffer, Buffer_size, sensor_timeout_);
        if (n != 3) {
            send_qt_and_ignore_response(connection_, sensor_timeout_);
            return set_errno_and_return(Urg_invalid_response_error);
        }

        if (buffer[n - 1] != scip_checksum(buffer, n - 1)) {
            // チェックサムの評価
            send_qt_and_ignore_response(connection_, sensor_timeout_);
            return set_errno_and_return(Urg_checksum_error);
        }

        if (type == static_cast<Lidar::measurement_t>(Stop)) {
            // QT 応答の場合には、最後の改行を読み捨て、正常応答として処理する
            n = readline(connection_, buffer, Buffer_size, sensor_timeout_);
            if (n == 0) {
                return 0;
            } else {
                return set_errno_and_return(Urg_invalid_response_error);
            }
        }

        if (!strncmp(buffer, "10", 2)) {
            is_booting_error_ = true;

            // 計測の準備ができていないときは、最後の空行を読み捨ててから
            // エラーを返す
            n = readline(connection_, buffer, Buffer_size, sensor_timeout_);
            if (n != 0) {
                send_qt_and_ignore_response(connection_, sensor_timeout_);
            }
            return set_errno_and_return(Urg_invalid_state_error);
        }

        if (indicated_.scan_times != 1) {
            if (!strncmp(buffer, "00", 2)) {
                // "00" 応答の場合は、エコーバック応答とみなし、
                // 最後の空行を読み捨て、次からのデータを返す
                n = readline(connection_, buffer, Buffer_size, sensor_timeout_);
                if (n != 0) {
                    send_qt_and_ignore_response(connection_, sensor_timeout_);
                    return set_errno_and_return(Urg_invalid_response_error);
                } else {
                    return receive_data(data, intensity, time_stamp);
                }
            }
        }

        // Gx, Hx のときは 00P が返されたときがデータ
        // Mx, Nx のときは 99b が返されたときがデータ
        if (((indicated_.scan_times == 1) && (strncmp(buffer, "00", 2))) ||
            ((indicated_.scan_times != 1) && (strncmp(buffer, "99", 2)))) {
#if 0
            // !!! エラーハンドリングを実装するときに、有効にして実装し直す
            if (urg->error_handler) {
                type = urg->error_handler(buffer, urg);
            }

            if (type == URG_UNKNOWN) {
                ignore_receive_data_with_qt(urg, urg->timeout);
                return set_errno_and_return(urg, URG_INVALID_RESPONSE);
            }
#endif
        }

        // タイムスタンプの取得
        n = readline(connection_, buffer, Buffer_size, sensor_timeout_);
        if (n > 0) {
            if (time_stamp) {
                *time_stamp = decode_scip(buffer, 4);
            }
        }

        // データの取得
        int ret = Urg_no_error;
        switch (static_cast<int>(type)) {
        case Lidar::Distance:
        case Lidar::Multiecho:
            ret = receive_length_data(data, NULL, type, buffer);
            break;

        case Lidar::Distance_intensity:
        case Lidar::Multiecho_intensity:
            ret = receive_length_data(data, intensity, type, buffer);
            break;

        case Stop:
            ret = Urg_no_error;
            break;
        }

        // specified_scan_times == 1 のときは Gx 系コマンドが使われるため
        // データを明示的に停止しなくてよい
        if ((indicated_.scan_times > 1) && (remain_scan_times_ > 0)) {
            if (--remain_scan_times_ <= 0) {
                // データの停止のみを行う
                stop_measurement();
            }
        }

        return ret;
    }


    int receive_length_data(long length[], unsigned short intensity[],
                            Lidar::measurement_t type, char buffer[])
    {
        int n;
        int step_filled = 0;
        int line_filled = 0;
        int multiecho_index = 0;

        const int each_size = received_.range_data_byte;
        int data_size = each_size;
        bool is_intensity = false;
        bool is_multiecho = false;
        int multiecho_max_size = 1;

        if ((type == Lidar::Distance_intensity) ||
            (type == Lidar::Multiecho_intensity)) {
            data_size *= 2;
            is_intensity = true;
        }
        received_.is_multiecho = false;
        if ((type == Lidar::Multiecho) ||
            (type == Lidar::Multiecho_intensity)) {
            received_.is_multiecho = true;
            is_multiecho = true;
            multiecho_max_size = max_echo_size();
        }

        int timeout = sensor_timeout_ + (skip_scan_ * sensor_.scan_usec / 1000);
        do {
            char *p = buffer;
            char *last_p;

            n = readline(connection_, &buffer[line_filled],
                         Buffer_size - line_filled, timeout);

            if (n > 0) {
                // チェックサムの評価
                if (buffer[line_filled + n - 1] !=
                    scip_checksum(&buffer[line_filled], n - 1)) {
                    send_qt_and_ignore_response(connection_, timeout);

                    return set_errno_and_return(Urg_checksum_error);
                }
            }

            if (n > 0) {
                line_filled += n - 1;
            }
            last_p = p + line_filled;

            while ((last_p - p) >= data_size) {
                int index;

                if (*p == '&') {
                    // 先頭文字が '&' だったときは、マルチエコーのデータとみなす

                    if ((last_p - (p + 1)) < data_size) {
                        // '&' を除いて、data_size 分データが無ければ抜ける
                        break;
                    }

                    --step_filled;
                    ++multiecho_index;
                    ++p;
                    --line_filled;

                } else {
                    // 次のデータ
                    multiecho_index = 0;
                }

                index = (step_filled * multiecho_max_size) + multiecho_index;

                if (step_filled >
                    (received_.last_index - received_.first_index)) {
                    // データが多過ぎる場合は、残りのデータを無視して戻る
                    send_qt_and_ignore_response(connection_, timeout);
                    return set_errno_and_return(Urg_receive_error);
                }

                if (is_multiecho && (multiecho_index == 0)) {
                    // マルチエコーのデータ格納先をダミーデータで埋める
                    int i;
                    if (length) {
                        for (i = 1; i < multiecho_max_size; ++i) {
                            length[index + i] = 0;
                        }
                    }
                    if (intensity) {
                        for (i = 1; i < multiecho_max_size; ++i) {
                            intensity[index + i] = 0;
                        }
                    }
                }

                // 距離データの格納
                if (length) {
                    length[index] = decode_scip(p, 3);
                }
                p += 3;

                // 強度データの格納
                if (is_intensity) {
                    if (intensity) {
                        intensity[index] =
                            static_cast<unsigned short>(decode_scip(p, 3));
                    }
                    p += 3;
                }

                ++step_filled;
                line_filled -= data_size;
            }

            // 次に処理する文字を退避
            memmove(buffer, p, line_filled);
        } while (n > 0);

        return step_filled;
    }


    int parse_distance_echoback(const char echoback[])
    {
        if (!strcmp("QT", echoback)) {
            return Stop;
        }

        size_t line_length = strlen(echoback);
        Lidar::measurement_t ret_type = static_cast<Lidar::measurement_t>(Stop);
        if ((line_length == 12) &&
            ((echoback[0] == 'G') || (echoback[0] == 'H'))) {
            ret_type = parse_distance_parameter(echoback);

        } else if ((line_length == 15) &&
                   ((echoback[0] == 'M') || (echoback[0] == 'N'))) {
            ret_type = parse_distance_parameter(echoback);
        }
        return ret_type;
    }


    Lidar::measurement_t parse_distance_parameter(const char echoback[])
    {
        Lidar::measurement_t ret_type = static_cast<Lidar::measurement_t>(Stop);

        received_.range_data_byte = 3;
        if (echoback[1] == 'S') {
            received_.range_data_byte = 2;
            ret_type = Lidar::Distance;

        } else if (echoback[1] == 'D') {
            if ((echoback[0] == 'G') || (echoback[0] == 'M')) {
                ret_type = Lidar::Distance;
            } else if ((echoback[0] == 'H') || (echoback[0] == 'N')) {
                ret_type = Lidar::Multiecho;
            }
        } else if (echoback[1] == 'E') {
            if ((echoback[0] == 'G') || (echoback[0] == 'M')) {
                ret_type = Lidar::Distance_intensity;
            } else if ((echoback[0] == 'H') || (echoback[0] == 'N')) {
                ret_type = Lidar::Multiecho_intensity;
            }
        }

        // パラメータの格納
        received_.first_index = parse_parameter(&echoback[2], 4);
        received_.last_index = parse_parameter(&echoback[6], 4);
        received_.skip_step = parse_parameter(&echoback[10], 2);

        return ret_type;
    }


    int parse_parameter(const char *parameter, int size)
    {
        char buffer[5];
        memcpy(buffer, parameter, size);
        buffer[size] = '\0';

        return strtol(buffer, NULL, 10);
    }


    int max_echo_size(void)
    {
        return Urg_max_echo;
    }


    string remove_sub_information(const string& line)
    {
        // '(' 以降の文字列を取り除く
        // 製品型式に生産者の名前がある場合には、取り除く
        string::size_type index = line.find('(');
        if (index == string::npos) {
            return line;
        }

        return line.substr(0, index);
    }


    state_t sensor_state(void)
    {
        enum { Receive_buffer_size = 64 };
        char receive_buffer[Receive_buffer_size];

        int st_expected[] = { 0, Expected_end };
        if (scip_response(connection_, "%ST\n", st_expected,
                          sensor_timeout_, receive_buffer,
                          Receive_buffer_size) < 0) {
            return Unknown;
        }

        typedef struct
        {
            state_t state;
            string response_code;
        } state_code_t;

        state_code_t state_codes[] = {
            { Idle, "000 Idle" },
            { Booting, "001 Booting" },
            { Time_adjustment, "002 Time_adjustment" },
            { Single_scan, "003 Single_scan" },
            { Multi_scan, "004 Multi_scan" },
            { Sleep, "005 Sleep" },
            { Waking_up, "006 Waking_up" },

            { LN_Booting, "050 LN_Booting" },
            { LN_Measurement, "051 LN_Measurement" },
            { LN_Responding, "052 LN_Responding" },
            { LN_Responding, "053 LN_Responding" },

            { Error_detected, "900 Error_detected" },
            { Firmware_update, "901 Firmware_update" },
            { Development, "902 Development" },
        };

        size_t n = sizeof(state_codes) / sizeof(state_codes[0]);
        for (size_t i = 0; i < n; ++i) {
            if (receive_buffer == state_codes[i].response_code) {
                return state_codes[i].state;
            }
        }
        return Unknown;
    }
};


Urg_driver::Urg_driver(void) : pimpl(new pImpl)
{
}


Urg_driver::~Urg_driver(void)
{
}


std::vector<std::string> Urg_driver::find_ports(void)
{
    vector<string> devices = Serial::find_ports();

    // URG と思われる device を vector の先頭に移動させる
    size_t replaced_index = 0;
    size_t n = devices.size();
    for (size_t i = 0; i < n; ++i) {
        if (is_urg_port(devices[i]) && (i > replaced_index)) {
            swap(devices[i], devices[replaced_index]);
            ++replaced_index;
        }
    }

    return devices;
}


bool Urg_driver::is_urg_port(const std::string& port)
{
    string driver_name = Serial::port_driver_name(port);
    if ((driver_name == "URG Series USB Device Driver") ||
        (driver_name == "URG-X002 USB Device Driver")) {
        return true;
    }
    return false;
}


const char* Urg_driver::what(void) const
{
    return pimpl->error_message_.c_str();
}


bool Urg_driver::open(const char* device_name_or_ip_address,
                      long baudrate_or_port_number, connection_t type)
{
    close();

    return pimpl->open(device_name_or_ip_address,
                       baudrate_or_port_number, type);
}


bool Urg_driver::open(Connection* connection)
{
    //close();
    return pimpl->open(connection);
}


bool Urg_driver::is_booting(void)
{
    return pimpl->is_booting_error_;
}


void Urg_driver::close(void)
{
    pimpl->close();
}


bool Urg_driver::is_open(void) const
{
    return pimpl->is_open();
}


void Urg_driver::set_connection(Connection* connection)
{
    close();
    pimpl->connection_ = connection;
}


Connection* Urg_driver::connection(void)
{
    return pimpl->connection_;
}


void Urg_driver::set_timeout_msec(int msec)
{
    pimpl->indicated_.timeout = msec;
    pimpl->sensor_timeout_ = msec;
}


bool Urg_driver::reboot(void)
{
    if (!is_open()) {
        return pimpl->set_errno_and_return(Urg_not_connected);
    }

    // ２回目の RB 送信後、接続を切断する
    int rb_expected[] = { 0, 1, Expected_end };
    for (int i = 0; i < 2; ++i) {
        if (pimpl->scip_response(pimpl->connection_, "RB\n", rb_expected,
                                 pimpl->sensor_timeout_, NULL, 0) < 0) {
            return pimpl->set_errno_and_return(Urg_invalid_response_error);
        }
    }
    close();
    return pimpl->set_errno_and_return(Urg_no_error);
}


bool Urg_driver::sleep(void)
{
    stop_measurement();
    int sl_expected[] = { 0, Expected_end };
    if (pimpl->scip_response(pimpl->connection_, "%SL\n", sl_expected,
                             pimpl->sensor_timeout_, NULL, 0) < 0) {
        return pimpl->set_errno_and_return(Urg_invalid_response_error);
    }
    return pimpl->set_errno_and_return(Urg_no_error);
}


void Urg_driver::wakeup(void)
{
    stop_measurement();
}


bool Urg_driver::start_measurement(measurement_t type,
                                   int scan_times, int skip_scan)
{
    if (!is_open()) {
        return pimpl->set_errno_and_return(Urg_not_connected);
    }

    pimpl->skip_scan_ = skip_scan;
    return pimpl->start_measurement(type, scan_times, skip_scan);
}


bool Urg_driver::get_distance(std::vector<long>& data, long *time_stamp)
{
    if (!is_open()) {
        return pimpl->set_errno_and_return(Urg_not_connected);
    }

    if (pimpl->measurement_type_ != Distance) {
        pimpl->error_message_ =
            "the type of start_measurement() is not Distance.";
        return false;
    }

    data.resize(max_data_size());
    return pimpl->receive_data(&data[0], NULL, time_stamp);
}


bool Urg_driver::get_distance_intensity(std::vector<long>& data,
                                        std::vector<unsigned short>& intensity,
                                        long *time_stamp)
{
    if (!is_open()) {
        return pimpl->set_errno_and_return(Urg_not_connected);
    }

    if (pimpl->measurement_type_ != Distance_intensity) {
        pimpl->error_message_ =
            "the type of start_measurement() is not Distance_intensity.";
        return false;
    }

    data.resize(max_data_size());
    intensity.resize(max_data_size());
    return pimpl->receive_data(&data[0], &intensity[0], time_stamp);
}


bool Urg_driver::get_multiecho(std::vector<long>& data_multiecho,
                               long* time_stamp)
{
    if (!is_open()) {
        return pimpl->set_errno_and_return(Urg_not_connected);
    }

    if (pimpl->measurement_type_ != Multiecho) {
        pimpl->error_message_ =
            "the type of start_measurement() is not Multiecho.";
        return false;
    }

    data_multiecho.resize(max_data_size() * pimpl->max_echo_size());
    return pimpl->receive_data(&data_multiecho[0], NULL, time_stamp);
}


bool Urg_driver::get_multiecho_intensity(std::vector<long>& data_multiecho,
                                         std::vector<unsigned short>&
                                         intensity_multiecho,
                                         long* time_stamp)
{
    if (!is_open()) {
        return pimpl->set_errno_and_return(Urg_not_connected);
    }

    if (pimpl->measurement_type_ != Multiecho_intensity) {
        pimpl->error_message_ =
            "the type of start_measurement() is not Multiecho_intensity.";
        return false;
    }

    data_multiecho.resize(max_data_size() * pimpl->max_echo_size());
    intensity_multiecho.resize(max_data_size() * pimpl->max_echo_size());
    return pimpl->receive_data(&data_multiecho[0],
                               &intensity_multiecho[0], time_stamp);
}


bool Urg_driver::set_scanning_parameter(int first_step, int last_step,
                                        int skip_step)
{
    return pimpl->set_scanning_parameter(first_step, last_step, skip_step);
}


void Urg_driver::stop_measurement(void)
{
    pimpl->stop_measurement();
}


bool Urg_driver::set_sensor_time_stamp(long time_stamp)
{
    (void)time_stamp;
    // !!!
    return false;
}


double Urg_driver::index2rad(int index) const
{
    if (pimpl->received_.is_multiecho) {
        index /= max_echo_size();
    }
    return step2rad(index);
}


double Urg_driver::index2deg(int index) const
{
    return index2rad(index) * 180.0 / M_PI;
}


int Urg_driver::rad2index(double radian) const
{
    int step = rad2step(radian);
    if (pimpl->received_.is_multiecho) {
        step *= max_echo_size();
    }
    return step;
}


int Urg_driver::deg2index(double degree) const
{
    return rad2index(degree * M_PI / 180.0);
}


double Urg_driver::step2rad(int step) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }

    int actual_index = min(max(0, step), pimpl->sensor_.last_index);
    int index = actual_index - pimpl->sensor_.front_index +
        pimpl->indicated_.first_step;
    return (2.0 * M_PI) * index / pimpl->sensor_.area_resolution;
}


double Urg_driver::step2deg(int step) const
{
    return step2rad(step) * 180.0 / M_PI;
}


int Urg_driver::rad2step(double radian) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }

    int step = (int)(floor((pimpl->sensor_.area_resolution *
                            radian / (2.0 * M_PI)) + 0.5)) +
        pimpl->sensor_.front_index - pimpl->indicated_.first_step;

    step = min(max(0, step), pimpl->sensor_.last_index);
    return step;
}


int Urg_driver::deg2step(double degree) const
{
    return rad2step(degree * M_PI / 180.0);
}


int Urg_driver::min_step(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }
    return pimpl->sensor_.first_index;
}


int Urg_driver::max_step(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }
    return pimpl->sensor_.last_index;
}


int Urg_driver::front_step(void) const
{
    return pimpl->sensor_.front_index;
}


int Urg_driver::total_steps(void) const
{
    return pimpl->sensor_.area_resolution;
}


long Urg_driver::min_distance(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }
    return pimpl->sensor_.min_distance;
}


long Urg_driver::max_distance(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }
    return pimpl->sensor_.max_distance;
}


long Urg_driver::scan_usec(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }

    return pimpl->sensor_.scan_usec;
}


int Urg_driver::max_data_size(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }
    return pimpl->sensor_.last_index + 1;
}


int Urg_driver::max_echo_size(void) const
{
    if (!is_open()) {
        pimpl->set_errno_and_return(Urg_not_connected);
        return -1;
    }

    return pimpl->max_echo_size();
}


std::string Urg_driver::sensor_product_type(void) const
{
    if (!is_open()) {
        return "";
    }

    return pimpl->sensor_product_type_;
}


std::string Urg_driver::sensor_product_version(void) const
{
    if (!is_open()) {
        return "";
    }

    if (pimpl->sensor_product_version_.empty()) {
        pimpl->update_vv_information();
    }
    return pimpl->sensor_product_version_;
}


std::string Urg_driver::serial_id(void)
{
    if (!is_open()) {
        return "";
    }

    if (pimpl->sensor_product_serial_id_.empty()) {
        pimpl->update_vv_information();
    }
    return pimpl->sensor_product_serial_id_;
}


long Urg_driver::decode_scip(const char* data, int data_byte)
{
    return ::decode_scip(data, data_byte);
}
