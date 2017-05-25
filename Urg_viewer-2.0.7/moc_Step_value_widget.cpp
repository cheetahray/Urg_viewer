/****************************************************************************
** Meta object code from reading C++ file 'Step_value_widget.h'
**
** Created: Fri Apr 5 16:57:01 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Step_value_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Step_value_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Step_value_widget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      47,   19,   18,   18, 0x05,
      73,   18,   18,   18, 0x05,
      97,   92,   18,   18, 0x05,
     132,  120,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
     158,   18,   18,   18, 0x08,
     182,   92,   18,   18, 0x08,
     258,  202,   18,   18, 0x08,
     294,  120,   18,   18, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Step_value_widget[] = {
    "Step_value_widget\0\0with_intensity,is_multiecho\0"
    "config_changed(bool,bool)\0update_requested()\0"
    "step\0selection_changed(int)\0auto_update\0"
    "auto_update_changed(bool)\0"
    "config_button_changed()\0show_step_data(int)\0"
    "current_row,current_column,previous_row,previous_column\0"
    "currentCellChanged(int,int,int,int)\0"
    "auto_checkbox_changed(bool)\0"
};

void Step_value_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Step_value_widget *_t = static_cast<Step_value_widget *>(_o);
        switch (_id) {
        case 0: _t->config_changed((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 1: _t->update_requested(); break;
        case 2: _t->selection_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->auto_update_changed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->config_button_changed(); break;
        case 5: _t->show_step_data((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->currentCellChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 7: _t->auto_checkbox_changed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Step_value_widget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Step_value_widget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Step_value_widget,
      qt_meta_data_Step_value_widget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Step_value_widget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Step_value_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Step_value_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Step_value_widget))
        return static_cast<void*>(const_cast< Step_value_widget*>(this));
    if (!strcmp(_clname, "State"))
        return static_cast< State*>(const_cast< Step_value_widget*>(this));
    return QWidget::qt_metacast(_clname);
}

int Step_value_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void Step_value_widget::config_changed(bool _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Step_value_widget::update_requested()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void Step_value_widget::selection_changed(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Step_value_widget::auto_update_changed(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
