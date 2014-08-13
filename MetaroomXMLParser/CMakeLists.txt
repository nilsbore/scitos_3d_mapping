cmake_minimum_required(VERSION 2.8.3)
project(metaroom_xml_parser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs pcl_ros qt_build image_geometry)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS image_geometry
   DEPENDS libqt4-dev
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(metaroomXMLparser src/simpleXMLparser.cpp include/simpleXMLparser.h)

 target_link_libraries(metaroomXMLparser
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

