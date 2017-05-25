/****************************************************************************
** Meta object code from reading C++ file 'Serial_connection_widget.h'
**
** Created: Fri Apr 5 16:57:06 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Serial_connection_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Serial_connection_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_hrk__Serial_connection_widget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      47,   31,   30,   30, 0x05,
      81,   30,   30,   30, 0x05,
     102,   30,   30,   30, 0x05,

 // slots: signature, parameters, type, tag, flags
     119,   30,   30,   30, 0x08,
     144,   30,   30,   30, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_hrk__Serial_connection_widget[] = {
    "hrk::Serial_connection_widget\0\0"
    "device,baudrate\0connect_clicked(std::string,long)\0"
    "disconnect_clicked()\0rescan_clicked()\0"
    "connect_button_clicked()\0"
    "rescan_button_clicked()\0"
};

void hrk::Serial_connection_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Serial_connection_widget *_t = static_cast<Serial_connection_widget *>(_o);
        switch (_id) {
        case 0: _t->connect_clicked((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 1: _t->disconnect_clicked(); break;
        case 2: _t->rescan_clicked(); break;
        case 3: _t->connect_button_clicked(); break;
        case 4: _t->rescan_button_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData hrk::Serial_connection_widget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject hrk::Serial_connection_widget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_hrk__Serial_connection_widget,
      qt_meta_data_hrk__Serial_connection_widget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &hrk::Serial_connection_widget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *hrk::Serial_connection_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *hrk::Serial_connection_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_hrk__Serial_connection_widget))
        return static_cast<void*>(const_cast< Serial_connection_widget*>(this));
    return QWidget::qt_metacast(_clname);
}

int hrk::Serial_connection_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void hrk::Serial_connection_widget::connect_clicked(const std::string & _t1, long _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void hrk::Serial_connection_widget::disconnect_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void hrk::Serial_connection_widget::rescan_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
