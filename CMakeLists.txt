cmake_minimum_required(VERSION 2.8.3)
project(camlidar_calibration)

add_definitions(-std=c++11)
set(CMAKE_AUTOMOC ON)
# Find includes in corresponding build directories
# Adds in each directory ${CMAKE_CURRENT_SOURCE_DIR} and ${CMAKE_CURRENT_BINARY_DIR}
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (HEADER
    include/camlidar_calibration/FileLoader.h
)

set (SRC
    src/FileLoader.cpp
    src/main.cpp
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package (Ceres REQUIRED)
find_package(Glog)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    image_transport
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
      std_msgs
      image_transport
)

include_directories(
    include
    ${SRC_DIR}
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CERES_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
)

add_executable (camlidar_calibration ${HEADER} ${SRC})
target_link_libraries (camlidar_calibration
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Glog_LIBRARIES}
    ${OpenCV_LIBS}
)

install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
