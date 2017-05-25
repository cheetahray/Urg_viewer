#ifndef HRK_LIDAR_H
#define HRK_LIDAR_H

/*!
  \file
  \brief Lidar インターフェース

  \author Satofumi KAMIMURA

  $Id: lidar.h 1937 2010-10-25 01:12:49Z satofumi $
*/

#include <vector>
#include <cstddef>


namespace hrk
{
    class Connection;

    //! Lidar インターフェース
    class Lidar
    {
    public:
        typedef enum {
            Distance,            //!< 距離
            Distance_intensity,  //!< 距離 + 強度
            Multiecho,           //!< マルチエコーの距離
            Multiecho_intensity, //!< マルチエコーの(距離 + 強度)
        } measurement_t;

        virtual ~Lidar(void)
        {
        }

        /*!
          \brief 状態を示すメッセージを返す

          \return メッセージ文字列
        */
        virtual const char* what(void) const = 0;

        //! 接続を閉じる
        virtual void close(void) = 0;

        //! 接続しているかを返す
        virtual bool is_open(void) const = 0;

        //! 接続オブジェクトを設定する
        virtual void set_connection(Connection* connection) = 0;

        //! 接続オブジェクトを返す
        virtual Connection* connection(void) = 0;

        virtual bool start_measurement(measurement_t type,
                                       int scan_times, int skip_scan) = 0;
        virtual bool get_distance(std::vector<long>& data,
                                  long *time_stamp) = 0;
        virtual bool get_distance_intensity(std::vector<long>& data,
                                            std::vector<unsigned short>&
                                            intensity,
                                            long *time_stamp) = 0;
        virtual bool get_multiecho(std::vector<long>& data_multi,
                                   long* time_stamp) = 0;
        virtual bool get_multiecho_intensity(std::vector<long>& data_multiecho,
                                             std::vector<unsigned short>&
                                             intensity_multiecho,
                                             long* time_stamp) = 0;
        virtual bool set_scanning_parameter(int first_step, int last_step,
                                            int skip_step) = 0;
        virtual void stop_measurement(void) = 0;

        virtual double index2rad(int index) const = 0;
        virtual double index2deg(int index) const = 0;
        virtual int rad2index(double radian) const = 0;
        virtual int deg2index(double degree) const = 0;

        virtual double step2rad(int step) const = 0;
        virtual double step2deg(int step) const = 0;
        virtual int rad2step(double radian) const = 0;
        virtual int deg2step(double degree) const = 0;

        virtual int min_step(void) const = 0;
        virtual int max_step(void) const = 0;
        virtual int front_step(void) const = 0;
        virtual int total_steps(void) const = 0;
        virtual long min_distance(void) const = 0;
        virtual long max_distance(void) const = 0;
        virtual long scan_usec(void) const = 0;
        virtual int max_data_size(void) const = 0;
        virtual int max_echo_size(void) const = 0;
    };
}

#endif /* !HRK_LIDAR_H */
