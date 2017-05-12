# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)
project(gazebo_tools)

find_package(YARP         REQUIRED)
find_package(ICUBcontrib  REQUIRED)
find_package(gazebo REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()


include_directories(${YARP_INCLUDE_DIRS})

# add_definitions(-D_USE_MATH_DEFINES)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_executable(apply_force apply_force.cpp)
target_link_libraries(apply_force ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)
