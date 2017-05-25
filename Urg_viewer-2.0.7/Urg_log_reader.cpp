/*!
  \file
  \brief URG ログの読みだし

  \author Satofumi Kamimura

  $Id$
*/

#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include "Urg_log_reader.h"
#include "Urg_driver.h"

using namespace hrk;
using namespace std;


namespace
{
    enum {
        Max_line_size = 64 + 1,
    };

    typedef struct
    {
        fstream::pos_type seek_position;
        long total_play_second;
        long msec_to_next_scan;
    } scan_t;
}


struct Urg_log_reader::pImpl
{
    string error_message_;
    string log_file_;
    ifstream* fin_;
    bool is_loaded_;
    vector<scan_t> scan_data_;
    bool with_intensity_;
    bool is_multiecho_;

    int first_step_;
    int last_step_;
    int skip_steps_;
    int scan_times_;
    int scan_skips_;


    pImpl(void)
        : error_message_("no error."), fin_(NULL), is_loaded_(false),
          with_intensity_(false), is_multiecho_(false),
          first_step_(0), last_step_(0), skip_steps_(1),
          scan_times_(0), scan_skips_(0)
    {
    }


    ~pImpl(void)
    {
        close();
    }


    bool load(const char* log_file)
    {
        close();

        log_file_ = log_file;
        if (!reset_fin()) {
            error_message_ = "open fail.";
            return false;
        }

        // データ量の読みだし
        enum { Invalid_timestamp = -1 };
        long last_timestamp = Invalid_timestamp;
        long total_msec = 0;
        fstream::pos_type last_position = 0;
        bool last_LF_detected = false;
        bool command_parsed = false;
        size_t count_in_command = 0;
        int is_distance_response = false;
        string line;
        long line_number = 0;

        while (getline(*fin_, line)) {
            ++line_number;

            // 読み出し中のデータフォーマットが適切かを確認する
            if (!check_format_error(line, last_LF_detected)) {
                error_message_ = set_parse_error(line_number, "format error.");
                return false;
            }

            ++count_in_command;

            if (last_LF_detected) {
                last_LF_detected = false;

                // 次の文字が '9' から始まる場合は "99b" の応答とみなして
                // その行のデータを距離データとして登録する
                is_distance_response = (fin_->peek() == '9') ? true : false;
                if (is_distance_response) {
                    scan_t scan;
                    scan.seek_position = last_position;
                    scan_data_.push_back(scan);
                    count_in_command = 1;

                    // 送信コマンドのタイプを読み出す
                    if (!command_parsed) {
                        command_parsed = parse_command(line);
                    }
                }
            }

            if (is_distance_response && (count_in_command == 3)) {
                // タイムスタンプとみなして、計測した時間を計算する
                long timestamp = Urg_driver::decode_scip(line.data(), 4);
                int scan_data_size = scan_data_.size();
                if (scan_data_size >= 2) {
                    long msec_to_next_scan =
                        static_cast<unsigned short>(timestamp - last_timestamp);

                    scan_t& previous = scan_data_[scan_data_size - 2];
                    previous.msec_to_next_scan = msec_to_next_scan;
                    previous.total_play_second = ceil(total_msec / 1000.0);
                    total_msec += msec_to_next_scan;
                }
                last_timestamp = timestamp;
            }

            // 空行があったら、次の位置をコマンドの先頭として扱う
            if (line.empty()) {
                last_LF_detected = true;
                last_position = fin_->tellg();
            }
        }
        is_loaded_ = scan_data_.empty() ? false : true;
        if (!is_loaded_) {
            error_message_ = "no scan data.";
            return false;
        }

        scan_t &last = scan_data_.back();
        last.total_play_second = ceil(total_msec / 1000.0);
        last.msec_to_next_scan = 0;

        if (!reset_fin()) {
            error_message_ = "reopen fail.";
            return false;
        }
        return true;
    }


    string set_parse_error(long line_number, const char* message)
    {
        ostringstream stream;
        stream << line_number << ": " << message;
        return stream.str();
    }


    bool check_format_error(const string& line, bool last_LF_detected)
    {
        // １行の最大文字数が適切かを確認する
        if (line.size() > Max_line_size) {
            return false;
        }

        // 改行が連続してないことを確認する
        if (last_LF_detected && line.empty()) {
            return false;
        }

        return true;
    }


    bool parse_command(const string& line)
    {
        enum {
            GX_COMMAND_SIZE = 12,
            MX_COMMAND_SIZE = 15,
        };

        if (line.size() == GX_COMMAND_SIZE) {
            // Gx コマンドとみなしてパースする
            with_intensity_ = (line[1] == 'E') ? true : false;
            is_multiecho_ = (line[0] == 'G') ? false : true;

        } else if (line.size() == MX_COMMAND_SIZE) {
            // Mx コマンドとみなしてパースする
            with_intensity_ = (line[1] == 'E') ? true : false;
            is_multiecho_ = (line[0] == 'M') ? false : true;

            // scan interval の読み出し
            scan_skips_ = atoi(line.substr(12, 1).c_str());
            scan_times_ = atoi(line.substr(13, 2).c_str());

        } else {
            return false;
        }

        // 計測範囲の読み出し
        first_step_ = atoi(line.substr(2, 4).c_str());
        last_step_ = atoi(line.substr(6, 4).c_str());
        skip_steps_ = atoi(line.substr(10, 2).c_str());

        return true;
    }


    void close(void)
    {
        delete fin_;
        fin_ = NULL;

        is_loaded_ = false;
        scan_data_.clear();
    }


    bool reset_fin(void)
    {
        delete fin_;
        fin_ = new ifstream(log_file_.c_str(), ios_base::binary);
        return fin_->is_open();
    }
};


Urg_log_reader::Urg_log_reader(void) : pimpl(new pImpl)
{
}


Urg_log_reader::~Urg_log_reader(void)
{
}


bool Urg_log_reader::load(const char* log_file)
{
    return pimpl->load(log_file);
}


void Urg_log_reader::log_measurement_type(bool& with_intensity,
                                          bool& is_multiecho)
{
    with_intensity = pimpl->with_intensity_;
    is_multiecho = pimpl->is_multiecho_;
}


void Urg_log_reader::log_range(int& first_step, int& last_step, int& skip_steps)
{
    first_step = pimpl->first_step_;
    last_step = pimpl->last_step_;
    skip_steps = pimpl->skip_steps_;
}


void Urg_log_reader::log_scans(int& scan_times, int& scan_skips)
{
    scan_skips = pimpl->scan_skips_;
    scan_times = pimpl->scan_times_;
}


size_t Urg_log_reader::total_sec(void) const
{
    if (!pimpl->is_loaded_) {
        return 0;
    }

    if (pimpl->scan_data_.empty()) {
        return 0;
    }

    return pimpl->scan_data_.back().total_play_second;
}


size_t Urg_log_reader::total_scans(void) const
{
    if (!pimpl->is_loaded_) {
        return 0;
    }

    return pimpl->scan_data_.size();
}


bool Urg_log_reader::scan_data(size_t scan_index,
                               long& total_play_second, long& msec_to_next_scan)
{
    if (!pimpl->is_loaded_) {
        return false;
    }

    if (pimpl->scan_data_.empty()) {
        return 0;
    }

    scan_t& scan = pimpl->scan_data_[scan_index];
    total_play_second = scan.total_play_second;
    msec_to_next_scan = scan.msec_to_next_scan;

    return true;
}


bool Urg_log_reader::reload(void)
{
    if (!pimpl->is_loaded_) {
        return false;
    }

    return pimpl->reset_fin();
}


bool Urg_log_reader::set_next_scans(int scan_index)
{
    if (!pimpl->is_loaded_) {
        return false;
    }

    if ((scan_index < 0) ||
        (scan_index >= static_cast<int>(pimpl->scan_data_.size()))) {
        return false;
    }

    scan_t& scan = pimpl->scan_data_[scan_index];
    pimpl->fin_->seekg(scan.seek_position);
    return true;
}


const char* Urg_log_reader::what(void) const
{
    return pimpl->error_message_.c_str();
}


bool Urg_log_reader::change_baudrate(long baudrate)
{
    static_cast<void>(baudrate);

    // 何もしない

    return true;
}


bool Urg_log_reader::is_open(void) const
{
    return pimpl->fin_->eof() ? false : true;
}


void Urg_log_reader::close(void)
{
    pimpl->close();
}


int Urg_log_reader::write(const char* data, size_t data_size)
{
    static_cast<void>(data);
    static_cast<void>(data_size);

    // 正常に書き込めたことにする
    // 特には何もしない

    return data_size;
}


int Urg_log_reader::read(char* data, size_t max_data_size, int timeout)
{
    static_cast<void>(timeout);

    if (pimpl->fin_->eof()) {
        return -1;
    }

    pimpl->fin_->read(data, max_data_size);
    return max_data_size;
}


void Urg_log_reader::ungetc(int ch)
{
    pimpl->fin_->putback(ch);
}
