/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Mon Dec 10 13:19:55 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../projector_calibration/include/projector_calibration/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_projector_calibration__MouseHandler_points[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      44,   43,   43,   43, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_projector_calibration__MouseHandler_points[] = {
    "projector_calibration::MouseHandler_points\0"
    "\0new_point()\0"
};

const QMetaObject projector_calibration::MouseHandler_points::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_projector_calibration__MouseHandler_points,
      qt_meta_data_projector_calibration__MouseHandler_points, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &projector_calibration::MouseHandler_points::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *projector_calibration::MouseHandler_points::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *projector_calibration::MouseHandler_points::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_projector_calibration__MouseHandler_points))
        return static_cast<void*>(const_cast< MouseHandler_points*>(this));
    return QObject::qt_metacast(_clname);
}

int projector_calibration::MouseHandler_points::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: new_point(); break;
        default: ;
        }
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void projector_calibration::MouseHandler_points::new_point()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
static const uint qt_meta_data_projector_calibration__MouseHandler[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      37,   36,   36,   36, 0x05,
      52,   36,   36,   36, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_projector_calibration__MouseHandler[] = {
    "projector_calibration::MouseHandler\0"
    "\0redraw_image()\0marker_area_changed()\0"
};

const QMetaObject projector_calibration::MouseHandler::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_projector_calibration__MouseHandler,
      qt_meta_data_projector_calibration__MouseHandler, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &projector_calibration::MouseHandler::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *projector_calibration::MouseHandler::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *projector_calibration::MouseHandler::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_projector_calibration__MouseHandler))
        return static_cast<void*>(const_cast< MouseHandler*>(this));
    return QObject::qt_metacast(_clname);
}

int projector_calibration::MouseHandler::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: redraw_image(); break;
        case 1: marker_area_changed(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void projector_calibration::MouseHandler::redraw_image()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void projector_calibration::MouseHandler::marker_area_changed()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
static const uint qt_meta_data_projector_calibration__GL_Mesh_Viewer[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_projector_calibration__GL_Mesh_Viewer[] = {
    "projector_calibration::GL_Mesh_Viewer\0"
};

const QMetaObject projector_calibration::GL_Mesh_Viewer::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_projector_calibration__GL_Mesh_Viewer,
      qt_meta_data_projector_calibration__GL_Mesh_Viewer, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &projector_calibration::GL_Mesh_Viewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *projector_calibration::GL_Mesh_Viewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *projector_calibration::GL_Mesh_Viewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_projector_calibration__GL_Mesh_Viewer))
        return static_cast<void*>(const_cast< GL_Mesh_Viewer*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int projector_calibration::GL_Mesh_Viewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_projector_calibration__MainWindow[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      54,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      35,   34,   34,   34, 0x0a,
      54,   34,   34,   34, 0x0a,
      80,   34,   34,   34, 0x0a,
     101,   34,   34,   34, 0x0a,
     128,   34,   34,   34, 0x0a,
     155,   34,   34,   34, 0x0a,
     177,   34,   34,   34, 0x0a,
     207,   34,   34,   34, 0x0a,
     223,   34,   34,   34, 0x0a,
     245,  243,   34,   34, 0x0a,
     271,  267,   34,   34, 0x0a,
     301,  295,   34,   34, 0x0a,
     329,  320,   34,   34, 0x0a,
     361,  351,   34,   34, 0x0a,
     387,   34,   34,   34, 0x0a,
     410,   34,   34,   34, 0x0a,
     430,   34,   34,   34, 0x0a,
     450,   34,   34,   34, 0x0a,
     470,   34,   34,   34, 0x0a,
     491,   34,   34,   34, 0x0a,
     506,  502,   34,   34, 0x0a,
     523,   34,   34,   34, 0x0a,
     536,   34,   34,   34, 0x0a,
     553,   34,   34,   34, 0x0a,
     567,   34,   34,   34, 0x0a,
     591,  583,   34,   34, 0x0a,
     614,  612,   34,   34, 0x0a,
     648,  612,   34,   34, 0x0a,
     680,  612,   34,   34, 0x0a,
     712,   34,   34,   34, 0x0a,
     731,   34,   34,   34, 0x0a,
     748,   34,   34,   34, 0x0a,
     779,   34,   34,   34, 0x0a,
     812,   34,   34,   34, 0x0a,
     844,   34,   34,   34, 0x0a,
     881,   34,   34,   34, 0x0a,
     912,   34,   34,   34, 0x0a,
     931,   34,   34,   34, 0x0a,
     962,   34,   34,   34, 0x0a,
     986,   34,   34,   34, 0x0a,
    1007,   34,   34,   34, 0x0a,
    1029,   34,   34,   34, 0x0a,
    1057,   34,   34,   34, 0x0a,
    1082,   34,   34,   34, 0x0a,
    1100,   34,   34,   34, 0x0a,
    1118,   34,   34,   34, 0x0a,
    1145,   34,   34,   34, 0x0a,
    1165,   34,   34,   34, 0x0a,
    1179,   34,   34,   34, 0x0a,
    1198,   34,   34,   34, 0x0a,
    1218,   34,   34,   34, 0x0a,
    1238,   34,   34,   34, 0x0a,
    1261,   34,   34,   34, 0x0a,
    1285,   34,   34,   34, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_projector_calibration__MainWindow[] = {
    "projector_calibration::MainWindow\0\0"
    "mouse_new_points()\0show_fullscreen_pattern()\0"
    "select_marker_area()\0project_black_background()\0"
    "project_white_background()\0"
    "marker_size_changed()\0"
    "load_kinect_trafo_from_file()\0"
    "compute_trafo()\0save_kinect_trafo()\0"
    "z\0manual_z_changed(int)\0yaw\0"
    "manual_yaw_changed(int)\0z_max\0"
    "z_max_changed(int)\0min_dist\0"
    "min_dist_changed(int)\0threshold\0"
    "sl_threshold_changed(int)\0"
    "find_projection_area()\0update_proj_image()\0"
    "learn_environment()\0show_model_openGL()\0"
    "scene_static(double)\0ant_demo()\0ant\0"
    "got_new_ant(Ant)\0update_ant()\0"
    "save_model_obj()\0test_expmap()\0"
    "new_expmap(int)\0visible\0sl_handvisible(bool)\0"
    ",\0sl_grasp_started(cv::Point2f,int)\0"
    "sl_grasp_moved(cv::Point2f,int)\0"
    "sl_grasp_ended(cv::Point2f,int)\0"
    "setLightPos(float)\0process_events()\0"
    "user_interaction_toggled(bool)\0"
    "depth_visualzation_toggled(bool)\0"
    "pattern_auto_size_toggled(bool)\0"
    "foreGroundVisualizationToggled(bool)\0"
    "gl_visualization_toggled(bool)\0"
    "show_texture(bool)\0water_simulation_toggled(bool)\0"
    "show_height_lines(bool)\0compute_homography()\0"
    "add_new_observation()\0compute_projection_matrix()\0"
    "save_projection_matrix()\0save_homography()\0"
    "delete_last_img()\0restart_water_simulation()\0"
    "projection_opencv()\0detect_disc()\0"
    "evaluate_pattern()\0updateLoggingView()\0"
    "sl_received_image()\0pattern_size_changed()\0"
    "color_slider_moved(int)\0loadParameters()\0"
};

const QMetaObject projector_calibration::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_projector_calibration__MainWindow,
      qt_meta_data_projector_calibration__MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &projector_calibration::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *projector_calibration::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *projector_calibration::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_projector_calibration__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int projector_calibration::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: mouse_new_points(); break;
        case 1: show_fullscreen_pattern(); break;
        case 2: select_marker_area(); break;
        case 3: project_black_background(); break;
        case 4: project_white_background(); break;
        case 5: marker_size_changed(); break;
        case 6: load_kinect_trafo_from_file(); break;
        case 7: compute_trafo(); break;
        case 8: save_kinect_trafo(); break;
        case 9: manual_z_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: manual_yaw_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: z_max_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: min_dist_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: sl_threshold_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: find_projection_area(); break;
        case 15: update_proj_image(); break;
        case 16: learn_environment(); break;
        case 17: show_model_openGL(); break;
        case 18: scene_static((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 19: ant_demo(); break;
        case 20: got_new_ant((*reinterpret_cast< Ant(*)>(_a[1]))); break;
        case 21: update_ant(); break;
        case 22: save_model_obj(); break;
        case 23: test_expmap(); break;
        case 24: new_expmap((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 25: sl_handvisible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 26: sl_grasp_started((*reinterpret_cast< cv::Point2f(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 27: sl_grasp_moved((*reinterpret_cast< cv::Point2f(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 28: sl_grasp_ended((*reinterpret_cast< cv::Point2f(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 29: setLightPos((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 30: process_events(); break;
        case 31: user_interaction_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 32: depth_visualzation_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 33: pattern_auto_size_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 34: foreGroundVisualizationToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 35: gl_visualization_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 36: show_texture((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 37: water_simulation_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 38: show_height_lines((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 39: compute_homography(); break;
        case 40: add_new_observation(); break;
        case 41: compute_projection_matrix(); break;
        case 42: save_projection_matrix(); break;
        case 43: save_homography(); break;
        case 44: delete_last_img(); break;
        case 45: restart_water_simulation(); break;
        case 46: projection_opencv(); break;
        case 47: detect_disc(); break;
        case 48: evaluate_pattern(); break;
        case 49: updateLoggingView(); break;
        case 50: sl_received_image(); break;
        case 51: pattern_size_changed(); break;
        case 52: color_slider_moved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 53: loadParameters(); break;
        default: ;
        }
        _id -= 54;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
