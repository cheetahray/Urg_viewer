/*!
  \file
  \brief プラグインの処理

  \author Satofumi Kamimura

  $Id$
*/

#if defined(NO_LIBLUABIND)
#include "plugin.h"
#else
#include <iostream>
#include <sstream>
#include <luabind/luabind.hpp>
#include "plugin.h"
#include "Graph_widget.h"
#include "Plotter_2d_widget.h"
#include "luabind_initializer.h"
#include "Lua_handler.h"
#include "file_utils.h"
#include "geometry_bind.h"
#include "PointF.h"
#include "RectF.h"

using namespace hrk;
using namespace luabind;
using namespace std;
#endif


namespace
{
#if defined(NO_LIBLUABIND)
#else
    lua_State* lua_ = NULL;
#endif
    Plotter_2d_widget* plotter_ = NULL;


    void initialize_lua(void)
    {
#if defined(NO_LIBLUABIND)
        return;
#else
        if (lua_) {
            return;
        }

        lua_ = luabind_initializer();

        geometry_bind(lua_);
        module(lua_)
            [
             def("dirname", hrk::dirname),
             def("basename", hrk::basename),

             class_<QWidget>("QWidget")
             .def(constructor<QWidget*>()),

             class_<Graph_widget, QWidget>("Graph_widget")
             .def(constructor<int, int, QWidget*>())
             .def("set_title", &Graph_widget::set_title)
             .def("show", &Graph_widget::show)
             .def("hide", &Graph_widget::hide)
             .def("set_draw_range", &Graph_widget::set_draw_range)
             .def("clear", &Graph_widget::clear)
             .def("add_line_point", &Graph_widget::add_line_point)
             .def("draw_text", &Graph_widget::draw_text)
             .def("redraw", &Graph_widget::redraw)
             ];
#endif
    }
}


void plugin_register_plotter(Plotter_2d_widget* plotter)
{
    plotter_ = plotter;
}


bool plguin_load_plugin_file(const char* plugin_file)
{
#if defined(NO_LIBLUABIND)
    static_cast<void>(plugin_file);
    return false;
#else
    if (!is_file_exist(plugin_file)) {
        return false;
    }

    initialize_lua();

    if (!Lua_handler::dofile(lua_, plugin_file)) {
        cerr << "plguin_load_plugin_file(): " << Lua_handler::what(lua_)
             << endl;
        return false;
    }

    return true;
#endif
}


void plugin_open_device(void)
{
#if defined(NO_LIBLUABIND)
#else
    initialize_lua();

    ostringstream stream;
    stream << "if plugin_open_device then plugin_open_device() end";

    if (!Lua_handler::dostring(lua_, stream.str().c_str())) {
        cerr << "plugin_open_device(): " << Lua_handler::what(lua_) << endl;
    }
#endif
}


void plugin_get_measurement_data(const hrk::Lidar::measurement_t& type,
                                 int data_size,
                                 const long* distance,
                                 const unsigned short* intensity,
                                 long timestamp)
{
#if defined(NO_LIBLUABIND)
    static_cast<void>(type);
    static_cast<void>(data_size);
    static_cast<void>(distance);
    static_cast<void>(intensity);
    static_cast<void>(timestamp);
#else
    (void)type;
    (void)data_size;
    (void)distance;
    (void)intensity;
    (void)timestamp;

    ostringstream stream;
    stream << "if plugin_get_measurement_data then"
        " plugin_get_measurement_data(type, data_size, "
        "distance, intensity, timestamp) end";

    if (!Lua_handler::dostring(lua_, stream.str().c_str())) {
        cerr << "plugin_get_measurement_data(): "
             << Lua_handler::what(lua_) << endl;
    }
#endif
}


void plugin_close_device(void)
{
#if defined(NO_LIBLUABIND)
#else
    ostringstream stream;
    stream << "if plugin_close_device then plugin_close_device() end";

    if (!Lua_handler::dostring(lua_, stream.str().c_str())) {
        cerr << "plugin_close_device(): " << Lua_handler::what(lua_) << endl;
    }
#endif
}


bool plugin_open_log_file(const char* log_file)
{
#if defined(NO_LIBLUABIND)
    static_cast<void>(log_file);
    return false;
#else
    if (!lua_) {
        return false;
    }

    // plugin.lua の plugin_open_log_file() を呼び出す
    ostringstream stream;
    stream << "if plugin_open_log_file then plugin_open_log_file(\""
           << log_file << "\") end";

    if (!Lua_handler::dostring(lua_, stream.str().c_str())) {
        cerr << "plugin_open_log_file(): " << Lua_handler::what(lua_) << endl;
        return false;
    }

    return true;
#endif
}


void plugin_close_log_file(void)
{
#if defined(NO_LIBLUABIND)
    return;
#else
    if (!lua_) {
        return;
    }

    // plugin.lua の plugin_close_log_file() を呼び出す
    if (!Lua_handler::dostring(lua_, "if plugin_close_log_file then "
                              "plugin_close_log_file() end")) {
        cerr << "plugin_close_log_file(): " << Lua_handler::what(lua_) << endl;
        return;
    }
#endif
}


void plugin_log_index_updated(int index)
{
#if defined(NO_LIBLUABIND)
    static_cast<void>(index);
#else
    if (!lua_) {
        return;
    }

    // plugin.lua の plugin_log_index_updated() を呼び出す
    ostringstream stream;
    stream << "if plugin_log_index_updated then plugin_log_index_updated("
           << index << ") end";

    if (!Lua_handler::dostring(lua_, stream.str().c_str())) {
        cerr << "plugin_log_index_updated(): " << Lua_handler::what(lua_)
             << endl;
        return;
    }
#endif
}
