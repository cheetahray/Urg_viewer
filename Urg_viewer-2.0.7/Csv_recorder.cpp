/*!
  \file
  \brief CSV 形式でのデータ保存

  \author Satofumi Kamimura

  $Id$
*/

#include <fstream>
#include "Csv_recorder.h"
#include "Scan_setting.h"

using namespace std;


namespace
{
    typedef vector<vector<long> > Distances;
    typedef vector<vector<unsigned short> > Intensities;

    const int Exel_max_column_size = 255;
    const int Max_record_times = 100;
}


struct Csv_recorder::pImpl
{
    string product_type_;
    Scan_setting setting_;
    int steps_;
    int max_echo_size_;
    Distances distances_;
    Intensities intensities_;


    pImpl(void)
        : steps_(0), max_echo_size_(1)
    {
        setting_.with_intensity = false;
        setting_.is_multiecho = false;
    }


    void clear_data(void)
    {
        distances_.clear();
        intensities_.clear();
    }


    int scan_column_size(void)
    {
        int column_size = 1;
        column_size *= setting_.with_intensity ? 2 : 1;
        column_size *= setting_.is_multiecho ? max_echo_size_ : 1;

        return column_size;
    }


    size_t recordable_scan_times(void)
    {
        int column_size = scan_column_size();
        return min(Exel_max_column_size / column_size, Max_record_times);
    }


    bool save_csv(ofstream& fout)
    {
        const size_t scan_times =
            min(recordable_scan_times(), distances_.size());

        save_header_line(fout, scan_times);
        for (int y = 0; y < steps_; ++y) {
            for (size_t x = 0; x < scan_times; ++x) {
                save_raw_data(fout, distances_[x], intensities_[x], y);
            }
            fout << endl;
        }

        return true;
    }


    void save_header_line(ofstream& fout, int scan_times)
    {
        int echo_size = setting_.is_multiecho ? max_echo_size_ : 1;
        int scan_cell_size = setting_.with_intensity ?
            echo_size * 2 : echo_size;

        for (int k = 0; k < scan_times; ++k) {
            for (int i = 0; i < scan_cell_size; ++i) {
                fout << "scan_" << k << ",";
            }
        }
        fout << endl;

        for (int k = 0; k < scan_times; ++k) {
            for (int i = 0; i < echo_size; ++i) {
                fout << "distance_" << i << ",";
            }
            if (setting_.with_intensity) {
                for (int i = 0; i < echo_size; ++i) {
                    fout << "intensity_" << i << ",";
                }
            }
        }
        fout << endl;
    }


    void save_raw_data(ofstream& fout,
                       const vector<long>& distance,
                       const vector<unsigned short>& intensity,
                       int step_index)
    {
        int echo_size = setting_.is_multiecho ? max_echo_size_ : 1;
        int index = step_index * (setting_.is_multiecho ? max_echo_size_ : 1);

        for (int i = 0; i < echo_size; ++i) {
            fout << distance[index + i] << ",";
        }
        if (setting_.with_intensity) {
            for (int i = 0; i < echo_size; ++i) {
                fout << intensity[index + i] << ",";
            }
        }
    }


    bool save_setting_information(ofstream& fout)
    {
        fout << product_type_ << ","
             << setting_.first_step << ","
             << setting_.last_step << ","
             << setting_.group_steps << endl;

        return true;
    }
};


Csv_recorder::Csv_recorder(void)
    : pimpl(new pImpl)
{
}


Csv_recorder::~Csv_recorder(void)
{
}


void Csv_recorder::set_scan_setting(const std::string& product_type,
                                    Scan_setting& setting, int max_echo_size)
{
    pimpl->clear_data();

    pimpl->product_type_ = product_type;
    pimpl->setting_ = setting;
    pimpl->steps_ = setting.last_step - setting.first_step + 1;
    pimpl->max_echo_size_ = max_echo_size;
}


size_t Csv_recorder::recordable_scan_times(void) const
{
    return pimpl->recordable_scan_times();
}


void Csv_recorder::set_receive_data(const std::vector<long>& distance,
                                    const std::vector<unsigned short>&
                                    intensity)
{
    pimpl->distances_.push_back(distance);
    if (pimpl->setting_.with_intensity) {
        pimpl->intensities_.push_back(intensity);
    }
}


bool Csv_recorder::save_file(const char* file_path)
{
    ofstream fout(file_path);
    if (!fout.is_open()) {
        return false;
    }

    // センサ情報の記録
    if (!pimpl->save_setting_information(fout)) {
        return false;
    }

    // 距離と強度データの記録
    return pimpl->save_csv(fout);
}
