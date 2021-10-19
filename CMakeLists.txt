cmake_minimum_required(VERSION 3.0.2)
project(extended_kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  sensor_msgs
  nav_msgs
  eigen_conversions
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
   INCLUDE_DIRS include
  )

find_package(roslaunch)

find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ekf src/main.cpp src/extendedKalmanFilter.cpp src/interface.cpp src/utilities.cpp include/extendedKalmanFilter.hpp include/interface.hpp include/utilities.hpp include/measurement.hpp )
target_link_libraries(ekf ${catkin_LIBRARIES})
add_dependencies(ekf  ${catkin_EXPORTED_TARGETS})

add_executable(covariance src/setMeasurementCovariance.cpp include/setMeasurementCovariance.hpp)
target_link_libraries(covariance ${catkin_LIBRARIES})
add_dependencies(covariance  ${catkin_EXPORTED_TARGETS})