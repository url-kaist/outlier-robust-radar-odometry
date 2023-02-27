project(orora)
cmake_minimum_required(VERSION 3.13)
add_compile_options(-std=c++17)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(OMP_NUM_THREADS 8)

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" OFF)
include(3rdparty/find_dependencies.cmake)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(orora_src
	src/graph.cc
	src/features.cpp
	src/radar_utils.cpp
	src/association.cpp
	src/visualization.cpp
	include/conversion.hpp
)

add_executable(orora_odom src/odometry.cpp ${orora_src})
target_link_libraries(orora_odom
#    features
#    radar_utils
#    association
#	correspond
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_INCLUDE_DIRS}
	${OpenMP_CXX_FLAGS}
    Eigen3::Eigen
	stdc++fs
	pmc::pmc
	TBB::tbb
)

add_executable(mulran_generate_gt src/mulran_generate_gt.cpp src/radar_utils.cpp)
target_link_libraries(mulran_generate_gt
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		${PCL_LIBRARIES}
		${Boost_INCLUDE_DIRS}
		Eigen3::Eigen
		)
