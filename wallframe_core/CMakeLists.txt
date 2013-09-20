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

### QT ###
find_package(Qt4 REQUIRED)
if(QT_FOUND)
	set (QT_USE_QTWEBKIT TRUE)
	set (QT_USE_QTOPENGL TRUE)
	include(${QT_USE_FILE})
	add_definitions(${QT_DEFINITIONS})
	add_definitions(-DQT_NO_KEYWORDS)
else(QT_FOUND)
	message(FATAL: QT NOT FOUND)
endif(QT_FOUND)

#uncomment if you have defined services
rosbuild_gensrv()

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## MOC
set(_moc ${CMAKE_CURRENT_BINARY_DIR}/${_current_MOC})

## QT Wrapping
set(modulair_core_sources src/modulair_app_base.cpp
													src/modulair_app_base_qt.cpp)
set(modulair_core_headers include/${PROJECT_NAME}/modulair_app_base.h
													include/${PROJECT_NAME}/modulair_app_base_qt.h)

qt4_wrap_cpp(modulair_core_headers_moc ${modulair_core_headers})

# Create MODULAIR_CORE library (libmodulair_core)
rosbuild_add_library(${PROJECT_NAME}	${modulair_core_headers}
										${modulair_core_sources}
										${modulair_core_headers_moc})
