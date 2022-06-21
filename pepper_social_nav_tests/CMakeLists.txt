cmake_minimum_required(VERSION 3.0.2)
project(pepper_social_nav_tests)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  actionlib
  actionlib_msgs
  esc_move_base_msgs
  #mbf_msgs
  nav_msgs
  pedsim_msgs
)

find_package(ompl REQUIRED)
find_package(Octomap REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)

find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs tf tf2 tf2_ros actionlib esc_move_base_msgs
  #mbf_msgs 
  nav_msgs
  #pedsim_msgs
  # DEPENDS OMPL
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
add_dependencies(odom_publisher ${catkin_EXPORTED_TARGETS} esc_move_base_msgs_generate_messages_cpp ${esc_move_base_msgs_EXPORTED_TARGETS})

add_executable(collision_counter src/collision_counter.cpp)
target_link_libraries(collision_counter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(collision_counter ${catkin_EXPORTED_TARGETS} esc_move_base_msgs_generate_messages_cpp ${esc_move_base_msgs_EXPORTED_TARGETS})