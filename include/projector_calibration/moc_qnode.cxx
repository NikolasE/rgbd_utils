/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created: Mon Dec 10 13:19:53 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../projector_calibration/include/projector_calibration/qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_projector_calibration__QNode[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      14,       // signalCount

 // signals: signature, parameters, type, tag, flags
      30,   29,   29,   29, 0x05,
      47,   29,   29,   29, 0x05,
      61,   29,   29,   29, 0x05,
      82,   29,   29,   29, 0x05,
     107,   29,   29,   29, 0x05,
     153,  124,   29,   29, 0x05,
     174,   29,   29,   29, 0x05,
     191,   29,   29,   29, 0x05,
     212,  208,   29,   29, 0x05,
     224,   29,   29,   29, 0x05,
     242,  240,   29,   29, 0x05,
     277,  240,   29,   29, 0x05,
     310,  240,   29,   29, 0x05,
     351,  343,   29,   29, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_projector_calibration__QNode[] = {
    "projector_calibration::QNode\0\0"
    "loggingUpdated()\0rosShutdown()\0"
    "received_col_Image()\0update_projector_image()\0"
    "model_computed()\0secs_since_last_static_image\0"
    "scene_static(double)\0process_events()\0"
    "new_light(float)\0ant\0newAnt(Ant)\0"
    "sl_update_ant()\0,\0sig_grasp_started(cv::Point2f,int)\0"
    "sig_grasp_moved(cv::Point2f,int)\0"
    "sig_grasp_ended(cv::Point2f,int)\0"
    "visible\0sig_handvisible(bool)\0"
};

const QMetaObject projector_calibration::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_projector_calibration__QNode,
      qt_meta_data_projector_calibration__QNode, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &projector_calibration::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *projector_calibration::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *projector_calibration::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_projector_calibration__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int projector_calibration::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: loggingUpdated(); break;
        case 1: rosShutdown(); break;
        case 2: received_col_Image(); break;
        case 3: update_projector_image(); break;
        case 4: model_computed(); break;
        case 5: scene_static((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: process_events(); break;
        case 7: new_light((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 8: newAnt((*reinterpret_cast< Ant(*)>(_a[1]))); break;
        case 9: sl_update_ant(); break;
        case 10: sig_grasp_started((*reinterpret_cast< cv::Point2f(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 11: sig_grasp_moved((*reinterpret_cast< cv::Point2f(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: sig_grasp_ended((*reinterpret_cast< cv::Point2f(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 13: sig_handvisible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 14;
    }
    return _id;
}

// SIGNAL 0
void projector_calibration::QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void projector_calibration::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void projector_calibration::QNode::received_col_Image()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void projector_calibration::QNode::update_projector_image()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void projector_calibration::QNode::model_computed()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void projector_calibration::QNode::scene_static(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void projector_calibration::QNode::process_events()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}

// SIGNAL 7
void projector_calibration::QNode::new_light(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void projector_calibration::QNode::newAnt(Ant _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void projector_calibration::QNode::sl_update_ant()
{
    QMetaObject::activate(this, &staticMetaObject, 9, 0);
}

// SIGNAL 10
void projector_calibration::QNode::sig_grasp_started(cv::Point2f _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void projector_calibration::QNode::sig_grasp_moved(cv::Point2f _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void projector_calibration::QNode::sig_grasp_ended(cv::Point2f _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void projector_calibration::QNode::sig_handvisible(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}
QT_END_MOC_NAMESPACE
