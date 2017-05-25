/****************************************************************************
** Meta object code from reading C++ file 'Urg_viewer_window.h'
**
** Created: Fri Apr 5 16:57:00 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Urg_viewer_window.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Urg_viewer_window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Urg_viewer_window[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      36,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      24,   19,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
      66,   44,   18,   18, 0x0a,
     104,   18,   18,   18, 0x08,
     117,   18,   18,   18, 0x08,
     150,  134,   18,   18, 0x08,
     204,  191,   18,   18, 0x08,
     247,   18,   18,   18, 0x08,
     268,   18,   18,   18, 0x08,
     285,   18,   18,   18, 0x08,
     302,   18,   18,   18, 0x08,
     319,   18,   18,   18, 0x08,
     341,   18,   18,   18, 0x08,
     371,  357,   18,   18, 0x08,
     427,  399,   18,   18, 0x08,
     458,   18,   18,   18, 0x08,
     491,  481,   18,   18, 0x08,
     519,   18,   18,   18, 0x08,
     536,   18,   18,   18, 0x08,
     559,   18,   18,   18, 0x08,
     583,   18,   18,   18, 0x08,
     607,   18,   18,   18, 0x08,
     630,  624,   18,   18, 0x08,
     650,   18,   18,   18, 0x08,
     667,   18,   18,   18, 0x08,
     681,   18,   18,   18, 0x08,
     699,   18,   18,   18, 0x08,
     714,   18,   18,   18, 0x08,
     728,   18,   18,   18, 0x08,
     742,   18,   18,   18, 0x08,
     756,   18,   18,   18, 0x08,
     786,  778,   18,   18, 0x08,
     815,  778,   18,   18, 0x08,
     846,   18,   18,   18, 0x28,
     885,  873,   18,   18, 0x08,
     918,  910,   18,   18, 0x08,
     959,  946,   18,   18, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Urg_viewer_window[] = {
    "Urg_viewer_window\0\0step\0show_step_data(int)\0"
    "setting,scan_interval\0"
    "update_scan_setting(Scan_setting,int)\0"
    "show_about()\0change_clicked()\0"
    "device,baudrate\0"
    "serial_connect_clicked(std::string,long)\0"
    "address,port\0ethernet_connect_clicked(std::string,long)\0"
    "disconnect_clicked()\0rescan_clicked()\0"
    "folder_clicked()\0record_clicked()\0"
    "record_stop_clicked()\0urg_connected()\0"
    "error_message\0urg_connect_failed(QString)\0"
    "with_intensity,is_multiecho\0"
    "scan_config_changed(bool,bool)\0"
    "length_data_received()\0mm_x,mm_y\0"
    "clicked_mm_point(long,long)\0"
    "escape_pressed()\0stop_playing_clicked()\0"
    "start_playing_clicked()\0pause_playing_clicked()\0"
    "play_completed()\0index\0slider_changed(int)\0"
    "scan_forward_1()\0scan_back_1()\0"
    "scan_forward_10()\0scan_back_10()\0"
    "play_faster()\0play_slower()\0save_as_csv()\0"
    "input_csv_save_file()\0checked\0"
    "set_step_value_visible(bool)\0"
    "set_scan_setting_visible(bool)\0"
    "set_scan_setting_visible()\0auto_update\0"
    "auto_update_chaned(bool)\0message\0"
    "receive_failed(const char*)\0second,index\0"
    "notify_play_time(long,long)\0"
};

void Urg_viewer_window::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Urg_viewer_window *_t = static_cast<Urg_viewer_window *>(_o);
        switch (_id) {
        case 0: _t->show_step_data((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->update_scan_setting((*reinterpret_cast< const Scan_setting(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->show_about(); break;
        case 3: _t->change_clicked(); break;
        case 4: _t->serial_connect_clicked((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 5: _t->ethernet_connect_clicked((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 6: _t->disconnect_clicked(); break;
        case 7: _t->rescan_clicked(); break;
        case 8: _t->folder_clicked(); break;
        case 9: _t->record_clicked(); break;
        case 10: _t->record_stop_clicked(); break;
        case 11: _t->urg_connected(); break;
        case 12: _t->urg_connect_failed((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 13: _t->scan_config_changed((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 14: _t->length_data_received(); break;
        case 15: _t->clicked_mm_point((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 16: _t->escape_pressed(); break;
        case 17: _t->stop_playing_clicked(); break;
        case 18: _t->start_playing_clicked(); break;
        case 19: _t->pause_playing_clicked(); break;
        case 20: _t->play_completed(); break;
        case 21: _t->slider_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 22: _t->scan_forward_1(); break;
        case 23: _t->scan_back_1(); break;
        case 24: _t->scan_forward_10(); break;
        case 25: _t->scan_back_10(); break;
        case 26: _t->play_faster(); break;
        case 27: _t->play_slower(); break;
        case 28: _t->save_as_csv(); break;
        case 29: _t->input_csv_save_file(); break;
        case 30: _t->set_step_value_visible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 31: _t->set_scan_setting_visible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 32: _t->set_scan_setting_visible(); break;
        case 33: _t->auto_update_chaned((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 34: _t->receive_failed((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 35: _t->notify_play_time((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Urg_viewer_window::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Urg_viewer_window::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_Urg_viewer_window,
      qt_meta_data_Urg_viewer_window, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Urg_viewer_window::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Urg_viewer_window::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Urg_viewer_window::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Urg_viewer_window))
        return static_cast<void*>(const_cast< Urg_viewer_window*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int Urg_viewer_window::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 36)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 36;
    }
    return _id;
}

// SIGNAL 0
void Urg_viewer_window::show_step_data(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
