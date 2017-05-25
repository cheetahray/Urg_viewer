/****************************************************************************
** Meta object code from reading C++ file 'Recorder_widget.h'
**
** Created: Fri Apr 5 16:57:03 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Recorder_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Recorder_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Recorder_widget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   16,   16,   16, 0x05,
      34,   16,   16,   16, 0x05,

 // slots: signature, parameters, type, tag, flags
      49,   16,   16,   16, 0x08,
      73,   16,   16,   16, 0x08,
      95,   16,   16,   16, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Recorder_widget[] = {
    "Recorder_widget\0\0record_clicked()\0"
    "stop_clicked()\0record_button_clicked()\0"
    "stop_button_clicked()\0update_timer_timeout()\0"
};

void Recorder_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Recorder_widget *_t = static_cast<Recorder_widget *>(_o);
        switch (_id) {
        case 0: _t->record_clicked(); break;
        case 1: _t->stop_clicked(); break;
        case 2: _t->record_button_clicked(); break;
        case 3: _t->stop_button_clicked(); break;
        case 4: _t->update_timer_timeout(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData Recorder_widget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Recorder_widget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Recorder_widget,
      qt_meta_data_Recorder_widget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Recorder_widget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Recorder_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Recorder_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Recorder_widget))
        return static_cast<void*>(const_cast< Recorder_widget*>(this));
    if (!strcmp(_clname, "State"))
        return static_cast< State*>(const_cast< Recorder_widget*>(this));
    return QWidget::qt_metacast(_clname);
}

int Recorder_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void Recorder_widget::record_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Recorder_widget::stop_clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
