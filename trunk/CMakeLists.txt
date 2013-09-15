cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#find_package(PCL 1.5 REQUIRED COMPONENTS)
FIND_PACKAGE(OpenSceneGraph REQUIRED osgDB osgViewer osgGA osgUtil osgSim osgFX osgText)

include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
rosbuild_add_executable(vvindow src/main.cpp)
target_link_libraries(vvindow ${OPENSCENEGRAPH_LIBRARIES})

OPTION(BUILD_EXAMPLES "Build examples" OFF)
IF(BUILD_EXAMPLES)

rosbuild_add_executable(osgshape osgshape.cpp)
target_link_libraries(osgshape ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(vertices vertices.cpp)
target_link_libraries(vertices ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osggeometryshader osggeometryshader.cpp)
target_link_libraries(osggeometryshader ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osglight osglight.cpp)
target_link_libraries(osglight ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osglightpoint osglightpoint.cpp)
target_link_libraries(osglightpoint ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(basiclightning basiclightning.cpp)
target_link_libraries(basiclightning ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osgcubemap osgcubemap.cpp)
target_link_libraries(osgcubemap ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osgrenderercubemap osgrenderercubemap.cpp)
target_link_libraries(osgrenderercubemap ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osgcompositeviewer osgcompositeviewer.cpp)
target_link_libraries(osgcompositeviewer ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osgwindows osgwindows.cpp)
target_link_libraries(osgwindows ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osgcamera osgcamera.cpp)
target_link_libraries(osgcamera ${OPENSCENEGRAPH_LIBRARIES})

rosbuild_add_executable(osgkeyboard osgkeyboard.cpp)
target_link_libraries(osgkeyboard ${OPENSCENEGRAPH_LIBRARIES})

ENDIF(BUILD_EXAMPLES)
