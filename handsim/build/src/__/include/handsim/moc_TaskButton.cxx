/****************************************************************************
** Meta object code from reading C++ file 'TaskButton.hh'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../include/handsim/TaskButton.hh"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TaskButton.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_haptix_gazebo_plugins__TaskButton[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      39,   35,   34,   34, 0x05,

 // slots: signature, parameters, type, tag, flags
      53,   34,   34,   34, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_haptix_gazebo_plugins__TaskButton[] = {
    "haptix_gazebo_plugins::TaskButton\0\0"
    "_id\0SendTask(int)\0OnButton()\0"
};

void haptix_gazebo_plugins::TaskButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TaskButton *_t = static_cast<TaskButton *>(_o);
        switch (_id) {
        case 0: _t->SendTask((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 1: _t->OnButton(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData haptix_gazebo_plugins::TaskButton::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject haptix_gazebo_plugins::TaskButton::staticMetaObject = {
    { &QToolButton::staticMetaObject, qt_meta_stringdata_haptix_gazebo_plugins__TaskButton,
      qt_meta_data_haptix_gazebo_plugins__TaskButton, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &haptix_gazebo_plugins::TaskButton::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *haptix_gazebo_plugins::TaskButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *haptix_gazebo_plugins::TaskButton::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_haptix_gazebo_plugins__TaskButton))
        return static_cast<void*>(const_cast< TaskButton*>(this));
    return QToolButton::qt_metacast(_clname);
}

int haptix_gazebo_plugins::TaskButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QToolButton::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void haptix_gazebo_plugins::TaskButton::SendTask(const int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
