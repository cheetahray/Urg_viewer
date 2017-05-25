/****************************************************************************
** Meta object code from reading C++ file 'Scan_setting_widget.h'
**
** Created: Fri Apr 5 16:57:02 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Scan_setting_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Scan_setting_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Scan_setting_widget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x05,
      37,   20,   20,   20, 0x05,
      74,   52,   20,   20, 0x05,

 // slots: signature, parameters, type, tag, flags
     119,  113,   20,   20, 0x08,
     144,   20,   20,   20, 0x08,
     151,   20,   20,   20, 0x08,
     169,   20,   20,   20, 0x08,
     187,   20,   20,   20, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Scan_setting_widget[] = {
    "Scan_setting_widget\0\0close_nortify()\0"
    "quit_nortify()\0setting,scan_interval\0"
    "scan_setting_updated(Scan_setting,int)\0"
    "event\0closeEvent(QCloseEvent*)\0quit()\0"
    "default_clicked()\0setting_changed()\0"
    "apply_clicked()\0"
};

void Scan_setting_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Scan_setting_widget *_t = static_cast<Scan_setting_widget *>(_o);
        switch (_id) {
        case 0: _t->close_nortify(); break;
        case 1: _t->quit_nortify(); break;
        case 2: _t->scan_setting_updated((*reinterpret_cast< const Scan_setting(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 4: _t->quit(); break;
        case 5: _t->default_clicked(); break;
        case 6: _t->setting_changed(); break;
        case 7: _t->apply_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Scan_setting_widget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Scan_setting_widget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Scan_setting_widget,
      qt_meta_data_Scan_setting_widget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Scan_setting_widget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Scan_setting_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Scan_setting_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Scan_setting_widget))
        return static_cast<void*>(const_cast< Scan_setting_widget*>(this));
    return QWidget::qt_metacast(_clname);
}

int Scan_setting_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void Scan_setting_widget::close_nortify()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Scan_setting_widget::quit_nortify()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void Scan_setting_widget::scan_setting_updated(const Scan_setting & _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
