#ifndef HRK_URG_LOG_READER_H
#define HRK_URG_LOG_READER_H

/*!
  \file
  \brief URG ログの読みだし

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include "Connection.h"


namespace hrk
{
    class Urg_log_reader : public hrk::Connection
    {
    public:
        Urg_log_reader(void);
        ~Urg_log_reader(void);

        bool load(const char* log_file);
        void log_measurement_type(bool& with_intensity, bool& is_multiecho);
        void log_range(int& first_step, int& last_step, int& skip_steps);
        void log_scans(int& scan_times, int& scan_skips);

        size_t total_sec(void) const;
        size_t total_scans(void) const;
        bool scan_data(size_t scan_index,
                       long& total_play_second, long& msec_to_next_scan);

        bool reload(void);
        bool set_next_scans(int scan_index);

        const char* what(void) const;
        bool change_baudrate(long baudrate);
        bool is_open(void) const;
        void close(void);
        int write(const char* data, size_t data_size);
        int read(char* data, size_t max_data_size, int timeout);
        void ungetc(int ch);

    private:
        Urg_log_reader(const Urg_log_reader& rhs);
        Urg_log_reader& operator = (const Urg_log_reader& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
