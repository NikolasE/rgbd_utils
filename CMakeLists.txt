cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()





# QT used only for gl_mesh_viewer (creates own lib)
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})
include_directories(${QT_QTOPENGL_INCLUDE_DIR})

QT4_WRAP_CPP(QT_MOC_HPP include/rgbd_utils/gl_mesh_viewer.hpp)
set (QT_SOURCES src/gl_mesh_viewer.cpp)
# < QT used only for gl_mesh_viewer (creates own lib) >



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas -Wno-deprecated -Wno-unused-function -Wno-unused-but-set-variable")



#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()



rosbuild_add_boost_directories()

  
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


option(WITH_LIBGEOMETRY "Inclusion of LibGeometry")
SET(LibGeometry_Base "/work/home/engelhan/code/libgeometry" CACHE FILEPATH "Location of LibGeometry")

#add_definitions(-DWITH_LIBGEOMETRY)

if (WITH_LIBGEOMETRY)
add_definitions(-DWITH_LIBGEOMETRY)
MESSAGE(STATUS "Compiling WITH LibGeometry at ${LibGeometry_Base}")
include_directories(${LibGeometry_Base})
include_directories(${LibGeometry_Base}/WildMagic4/SDK/Include)
include_directories( ${includes})
include_directories(${LibGeometry_Base}/Testing ${LibGeometry_Base}/mesh ${LibGeometry_Base}/base)
include_directories(${LibGeometry_Base}/parameterization ${LibGeometry_Base}/geometry ${LibGeometry_Base}/pointset)
include_directories(${LibGeometry_Base}/spatial)


#file(GLOB includes ${GEO_FOLDER} )

ELSE (WITH_LIBGEOMETRY)
MESSAGE(STATUS "Compiling WITHOUT LibGeometry")
ENDIF (WITH_LIBGEOMETRY)


rosbuild_add_library(gl_viewer ${QT_SOURCES} ${QT_MOC_HPP})


rosbuild_add_library(rgbd_utils src/calibration_utils.cpp src/gaussian_model.cpp src/projector_calibrator.cpp src/pinch_detection.cpp src/meshing.cpp src/surface_modeler.cpp src/ants.cpp )#src/exp_map.cpp)

rosbuild_link_boost(rgbd_utils thread signals system)

#rosbuild_add_executable(detection_demo src/main_test.cpp src/gaussian_model.cpp src/pinch_detection.cpp)
#target_link_libraries(detection_demo rgbd_utils)

#rosbuild_add_executable(ants src/main_ant.cpp src/ants.cpp)
#target_link_libraries(ants rgbd_utils)

if (WITH_LIBGEOMETRY)
target_link_libraries(rgbd_utils  ${LibGeometry_Base}/Testing/liblibGeometry.a ${LibGeometry_Base}/WildMagic4/SDK/Library/Release/libWm4Foundation.a )
ENDIF (WITH_LIBGEOMETRY)


#rosbuild_add_executable(boost_test src/ants.cpp)
#rosbuild_add_executable(a_star_test src/a_star.cpp)
#rosbuild_link_boost(boost_test thread signals system)

