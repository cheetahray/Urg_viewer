/****************************************************************************
** Meta object code from reading C++ file 'Plotter_2d_widget.h'
**
** Created: Fri Apr 5 16:57:01 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Plotter_2d_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Plotter_2d_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Plotter_2d_widget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      29,   19,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
      57,   18,   18,   18, 0x0a,
      85,   80,   18,   18, 0x0a,
     105,   18,   18,   18, 0x0a,
     116,   18,   18,   18, 0x0a,
     126,   18,   18,   18, 0x0a,
     163,  137,   18,   18, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Plotter_2d_widget[] = {
    "Plotter_2d_widget\0\0mm_x,mm_y\0"
    "clicked_mm_point(long,long)\0"
    "step_value_requested()\0step\0"
    "show_step_data(int)\0zoom_out()\0zoom_in()\0"
    "zoom_fit()\0recorded_times,loss_times\0"
    "recorded(long,long)\0"
};

void Plotter_2d_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Plotter_2d_widget *_t = static_cast<Plotter_2d_widget *>(_o);
        switch (_id) {
        case 0: _t->clicked_mm_point((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        case 1: _t->step_value_requested(); break;
        case 2: _t->show_step_data((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->zoom_out(); break;
        case 4: _t->zoom_in(); break;
        case 5: _t->zoom_fit(); break;
        case 6: _t->recorded((*reinterpret_cast< long(*)>(_a[1])),(*reinterpret_cast< long(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Plotter_2d_widget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Plotter_2d_widget::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_Plotter_2d_widget,
      qt_meta_data_Plotter_2d_widget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Plotter_2d_widget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Plotter_2d_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Plotter_2d_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Plotter_2d_widget))
        return static_cast<void*>(const_cast< Plotter_2d_widget*>(this));
    if (!strcmp(_clname, "State"))
        return static_cast< State*>(const_cast< Plotter_2d_widget*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int Plotter_2d_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
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
void Plotter_2d_widget::clicked_mm_point(long _t1, long _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
