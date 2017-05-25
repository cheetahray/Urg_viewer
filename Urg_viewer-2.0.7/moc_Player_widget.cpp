/****************************************************************************
** Meta object code from reading C++ file 'Player_widget.h'
**
** Created: Fri Apr 5 16:57:02 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Player_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Player_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Player_widget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x05,
      32,   14,   14,   14, 0x05,
      47,   14,   14,   14, 0x05,
      62,   14,   14,   14, 0x05,
      84,   78,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
     126,  104,   14,   14, 0x0a,
     157,  151,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Player_widget[] = {
    "Player_widget\0\0folder_clicked()\0"
    "play_clicked()\0stop_clicked()\0"
    "pause_clicked()\0index\0slider_changed(int)\0"
    "second,progress_index\0set_play_time(long,long)\0"
    "value\0slider_value_changed(int)\0"
};

void Player_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Player_widget *_t = static_cast<Player_widget *>(_o);
        switch (_id) {
        case 0: _t->folder_clicked(); break;
        case 1: _t->play_clicked(); break;
        case 2: _t->stop_clicked(); break;
        case 3: _t->pause_clicked(); break;
        case 4: _t->slider_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->set_play_time((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 6: _t->slider_value_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Player_widget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Player_widget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Player_widget,
      qt_meta_data_Player_widget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Player_widget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Player_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Player_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Player_widget))
        return static_cast<void*>(const_cast< Player_widget*>(this));
    if (!strcmp(_clname, "State"))
        return static_cast< State*>(const_cast< Player_widget*>(this));
    return QWidget::qt_metacast(_clname);
}

int Player_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
void Player_widget::folder_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Player_widget::play_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void Player_widget::stop_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void Player_widget::pause_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void Player_widget::slider_changed(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
