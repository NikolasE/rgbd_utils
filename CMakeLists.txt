cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



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


#add_definitions(-DWITH_LIBGEOMETRY)

#if(DEFINED $WITH_LIBGEOMETRY)

#MESSAGE(STATUS "Compiling WITH LibGeometry")

# link to libgeometry for exmap-computation
#set(GEO_FOLDER "/work/home/engelhan/code/libgeometry")
#include_directories(${GEO_FOLDER})
#include_directories(${GEO_FOLDER}/WildMagic4/SDK/Include)
#file(GLOB includes ${GEO_FOLDER}/* )
#include_directories( ${includes})
#ELSE(DEFINED WITH_$LIBGEOMETRY)
#MESSAGE(STATUS "Compiling WITHOUT LibGeometry")
#ENDIF(DEFINED $WITH_LIBGEOMETRY)



rosbuild_add_library(rgbd_utils src/calibration_utils.cpp src/meshing.cpp src/surface_modeler.cpp src/ants.cpp src/exp_map.cpp)
rosbuild_link_boost(rgbd_utils thread signals system)

#target_link_libraries(rgbd_utils  ${GEO_FOLDER}/Testing/liblibGeometry.a ${GEO_FOLDER}/WildMagic4/SDK/Library/Release/libWm4Foundation.a )



#rosbuild_add_executable(boost_test src/ants.cpp)
#rosbuild_add_executable(a_star_test src/a_star.cpp)
#rosbuild_link_boost(boost_test thread signals system)

