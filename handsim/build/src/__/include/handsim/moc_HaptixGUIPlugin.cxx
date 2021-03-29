/****************************************************************************
** Meta object code from reading C++ file 'HaptixGUIPlugin.hh'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../include/handsim/HaptixGUIPlugin.hh"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'HaptixGUIPlugin.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_haptix_gazebo_plugins__HaptixGUIPlugin[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      60,   40,   39,   39, 0x05,
     100,   92,   39,   39, 0x05,

 // slots: signature, parameters, type, tag, flags
     124,   40,   39,   39, 0x08,
     162,  158,   39,   39, 0x08,
     178,   39,   39,   39, 0x08,
     194,   39,   39,   39, 0x08,
     211,   39,   39,   39, 0x08,
     240,  233,   39,   39, 0x08,
     262,  233,   39,   39, 0x08,
     293,  233,   39,   39, 0x08,
     312,   92,   39,   39, 0x08,
     338,  233,   39,   39, 0x08,
     368,  359,   39,   39, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_haptix_gazebo_plugins__HaptixGUIPlugin[] = {
    "haptix_gazebo_plugins::HaptixGUIPlugin\0"
    "\0_contactName,_value\0"
    "SetContactForce(QString,double)\0_status\0"
    "MocapStatusChanged(int)\0"
    "OnSetContactForce(QString,double)\0_id\0"
    "OnTaskSent(int)\0OnNextClicked()\0"
    "OnResetClicked()\0OnResetSceneClicked()\0"
    "_state\0OnLocalCoordMove(int)\0"
    "OnViewpointRotationsCheck(int)\0"
    "OnStereoCheck(int)\0OnMocapStatusChanged(int)\0"
    "OnScalingSlider(int)\0_checked\0"
    "OnStartStopMocap(bool)\0"
};

void haptix_gazebo_plugins::HaptixGUIPlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        HaptixGUIPlugin *_t = static_cast<HaptixGUIPlugin *>(_o);
        switch (_id) {
        case 0: _t->SetContactForce((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->MocapStatusChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->OnSetContactForce((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 3: _t->OnTaskSent((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 4: _t->OnNextClicked(); break;
        case 5: _t->OnResetClicked(); break;
        case 6: _t->OnResetSceneClicked(); break;
        case 7: _t->OnLocalCoordMove((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->OnViewpointRotationsCheck((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->OnStereoCheck((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->OnMocapStatusChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->OnScalingSlider((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->OnStartStopMocap((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData haptix_gazebo_plugins::HaptixGUIPlugin::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject haptix_gazebo_plugins::HaptixGUIPlugin::staticMetaObject = {
    { &gazebo::GUIPlugin::staticMetaObject, qt_meta_stringdata_haptix_gazebo_plugins__HaptixGUIPlugin,
      qt_meta_data_haptix_gazebo_plugins__HaptixGUIPlugin, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &haptix_gazebo_plugins::HaptixGUIPlugin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *haptix_gazebo_plugins::HaptixGUIPlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *haptix_gazebo_plugins::HaptixGUIPlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_haptix_gazebo_plugins__HaptixGUIPlugin))
        return static_cast<void*>(const_cast< HaptixGUIPlugin*>(this));
    typedef gazebo::GUIPlugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int haptix_gazebo_plugins::HaptixGUIPlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef gazebo::GUIPlugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void haptix_gazebo_plugins::HaptixGUIPlugin::SetContactForce(QString _t1, double _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void haptix_gazebo_plugins::HaptixGUIPlugin::MocapStatusChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
