#ifndef HRK_URG_DRIVER_H
#define HRK_URG_DRIVER_H

/*!
  \file
  \brief Urg センサ

  \author Satofumi Kamimura

  $Id$
*/

#include <memory>
#include <string>
#include "Lidar.h"


namespace hrk
{
    class Urg_driver : public Lidar
    {
    public:
        enum {
            Default_baudrate = 115200,
            Default_port = 10940,
            Infinity_scan_times = -1,
        };

        typedef enum {
            Serial,
            Ethernet,
        } connection_t;

        Urg_driver(void);
        virtual ~Urg_driver(void);

        static std::vector<std::string> find_ports(void);
        static bool is_urg_port(const std::string& port);

        const char* what(void) const;

        bool open(const char* device_name_or_ip_address,
                  long baudrate_or_port_number, connection_t type = Serial);
        bool open(Connection* connection);

        // センサが起動中かどうかを返す
        bool is_booting(void);

        void close(void);
        bool is_open(void) const;
        void set_connection(Connection* connection);
        Connection* connection(void);

        void set_timeout_msec(int msec);

        bool reboot(void);

        bool sleep(void);
        void wakeup(void);

        bool start_measurement(measurement_t type = Distance,
                               int scan_times = Infinity_scan_times,
                               int skip_scan = 0);
        bool get_distance(std::vector<long>& data, long *time_stamp = NULL);
        bool get_distance_intensity(std::vector<long>& data,
                                    std::vector<unsigned short>& intensity,
                                    long *time_stamp = NULL);
        bool get_multiecho(std::vector<long>& data_multiecho,
                           long* time_stamp = NULL);
        bool get_multiecho_intensity(std::vector<long>& data_multiecho,
                                     std::vector<unsigned short>&
                                     intensity_multiecho,
                                     long* time_stamp = NULL);
        bool set_scanning_parameter(int first_step, int last_step,
                                    int skip_step = 1);
        void stop_measurement(void);
        bool set_sensor_time_stamp(long time_stamp);

        double index2rad(int index) const;
        double index2deg(int index) const;
        int rad2index(double radian) const;
        int deg2index(double degree) const;

        double step2rad(int step) const;
        double step2deg(int step) const;
        int rad2step(double radian) const;
        int deg2step(double degree) const;

        int min_step(void) const;
        int max_step(void) const;
        int front_step(void) const;
        int total_steps(void) const;
        long min_distance(void) const;
        long max_distance(void) const;
        long scan_usec(void) const;
        int max_data_size(void) const;
        int max_echo_size(void) const;

        std::string sensor_product_type(void) const;
        std::string sensor_product_version(void) const;
        std::string serial_id(void);
        static long decode_scip(const char* data, int data_byte);

    private:
        Urg_driver(const Urg_driver& rhs);
        Urg_driver& operator = (const Urg_driver& rhs);

        struct pImpl;
        std::auto_ptr<pImpl> pimpl;
    };
}

#endif
