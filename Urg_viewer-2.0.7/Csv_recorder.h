#ifndef CSV_RECORDER_H
#define CSV_RECORDER_H

/*!
  \file
  \brief CSV 形式でのデータ保存

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <vector>

class Scan_setting;


class Csv_recorder
{
 public:
    Csv_recorder(void);
    ~Csv_recorder(void);

    void set_scan_setting(const std::string& product_type,
                          Scan_setting& setting, int max_echo_size);
    size_t recordable_scan_times(void) const;
    void set_receive_data(const std::vector<long>& distance,
                          const std::vector<unsigned short>& intensity);
    bool save_file(const char* file_path);

 private:
    Csv_recorder(const Csv_recorder& rhs);
    Csv_recorder& operator = (const Csv_recorder& rhs);

    struct pImpl;
    std::auto_ptr<pImpl> pimpl;
};

#endif
