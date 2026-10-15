cmake_minimum_required(VERSION 3.0.2)
project(loop_optimization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  std_srvs
  tf
  nodelet
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost COMPONENTS timer REQUIRED QUIET)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES loop_optimization
  CATKIN_DEPENDS roscpp nodelet std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#set(LIB_OPENCV_IMGPROC_DIR /usr/lib/x86_64-linux-gnu)
#add_library(libtbb SHARED IMPORTED)
#set_target_properties(libtbb PROPERTIES IMPORTED_LOCATION ${LIB_OPENCV_IMGPROC_DIR}/libtbb.so.2)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  #${LIB_OPENCV_IMGPROC_DIR}/include
  src
)
if (PCL_FOUND)
  message("Hint: PCL is found")
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINATIONS})
else()
  message("Hint: PCL is not found!!")
endif()

add_executable(loop_optimization_node src/loop_optimization_node.cpp)
#add_library(loop_optimization_node src/loop_optimization_node.cpp)

target_link_libraries(loop_optimization_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  gtsam crypto
  #libtbb
)

option(LTAOM_BUILD_TRANSACTION_TESTS "Build isolated fault-injection backend executable" OFF)
option(LTAOM_BUILD_ABLATION "Build isolated fixed-input ablation executable" OFF)
option(LTAOM_BUILD_ARCHIVE_TOOLS "Build raw factor archive reconstruction tool" OFF)
if(LTAOM_BUILD_ARCHIVE_TOOLS)
  add_executable(rebuild_factor_archive src/rebuild_factor_archive.cpp)
  target_link_libraries(rebuild_factor_archive gtsam ${catkin_LIBRARIES})
endif()
if(LTAOM_BUILD_ABLATION)
  add_executable(loop_optimization_ablation_node src/loop_optimization_node.cpp)
  target_compile_definitions(loop_optimization_ablation_node PRIVATE LTAOM_ABLATION=1)
  target_link_libraries(loop_optimization_ablation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam crypto)
endif()
if(LTAOM_BUILD_TRANSACTION_TESTS)
  add_executable(loop_optimization_transaction_test_node src/loop_optimization_node.cpp)
  target_compile_definitions(loop_optimization_transaction_test_node PRIVATE LTAOM_TRANSACTION_TEST_HOOKS=1)
  target_link_libraries(loop_optimization_transaction_test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam crypto)
endif()

#add_library(loop_optimization_plugin
#  src/loop_optimization_plugin.cpp
#)

#target_link_libraries(loop_optimization_plugin loop_optimization_node ${catkin_LIBRARIES})
#if(catkin_EXPORTED_LIBRARIES)
#  add_dependencies(loop_optimization_plugin ${catkin_EXPORTED_LIBRARIES})
#endif()

#install(TARGETS loop_optimization_plugin
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
