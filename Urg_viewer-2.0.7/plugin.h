#ifndef PLUGIN_H
#define PLUGIN_H

/*!
  \file
  \brief プラグインの処理

  \author Satofumi Kamimura

  $Id$
*/

#include "Lidar.h"

class Plotter_2d_widget;


extern void plugin_register_plotter(Plotter_2d_widget* plotter);
extern bool plguin_load_plugin_file(const char* plugin_file);

extern void plugin_open_device(void);
extern void plugin_get_measurement_data(const hrk::Lidar::measurement_t& type,
                                        int data_size,
                                        const long* distance,
                                        const unsigned short* intensity,
                                        long timestamp);
extern void plugin_close_device(void);

extern bool plugin_open_log_file(const char* log_file);
extern void plugin_close_log_file(void);
extern void plugin_log_index_updated(int index);

#endif
