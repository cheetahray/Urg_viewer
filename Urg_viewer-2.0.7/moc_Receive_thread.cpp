/****************************************************************************
** Meta object code from reading C++ file 'Receive_thread.h'
**
** Created: Fri Apr 5 16:57:05 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Receive_thread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Receive_thread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Receive_thread[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: signature, parameters, type, tag, flags
      30,   16,   15,   15, 0x05,
      58,   15,   15,   15, 0x05,
      93,   69,   15,   15, 0x05,
     137,  111,   15,   15, 0x05,
     157,   15,   15,   15, 0x05,
     182,  174,   15,   15, 0x05,
     209,   15,   15,   15, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Receive_thread[] = {
    "Receive_thread\0\0error_message\0"
    "receive_failed(const char*)\0received()\0"
    "total_second,scan_index\0played(long,long)\0"
    "recorded_times,loss_times\0recorded(long,long)\0"
    "play_completed()\0percent\0"
    "csv_recording_percent(int)\0"
    "csv_recording_completed()\0"
};

void Receive_thread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Receive_thread *_t = static_cast<Receive_thread *>(_o);
        switch (_id) {
        case 0: _t->receive_failed((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 1: _t->received(); break;
        case 2: _t->played((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 3: _t->recorded((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 4: _t->play_completed(); break;
        case 5: _t->csv_recording_percent((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->csv_recording_completed(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Receive_thread::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Receive_thread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Receive_thread,
      qt_meta_data_Receive_thread, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Receive_thread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Receive_thread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Receive_thread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Receive_thread))
        return static_cast<void*>(const_cast< Receive_thread*>(this));
    return QThread::qt_metacast(_clname);
}

int Receive_thread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void Receive_thread::receive_failed(const char * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Receive_thread::received()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void Receive_thread::played(long _t1, long _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Receive_thread::recorded(long _t1, long _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Receive_thread::play_completed()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void Receive_thread::csv_recording_percent(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Receive_thread::csv_recording_completed()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}
QT_END_MOC_NAMESPACE
