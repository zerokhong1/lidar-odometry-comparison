cmake_minimum_required(VERSION 3.8)
project(genz_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# OpenMP
find_package(OpenMP QUIET)
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=3)
endif()

# Python (matplotlib)
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  pcl_conversions
  livox_interfaces
)

# Third-party
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Reuse FAST-LIO2's ikd-Tree and IKFoM (copy via symlink or reference)
set(FASTLIO2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../FAST_LIO2)

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(genz_lio_node
  src/genz_lio_node.cpp
  src/preprocess.cpp
  ${FASTLIO2_DIR}/include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(genz_lio_node PUBLIC
  # Our include comes first to shadow FAST-LIO2's common_lib.h
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # FAST-LIO2 headers (ikd-Tree, IKFoM, so3_math, etc.) come after
  ${FASTLIO2_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(genz_lio_node
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  Eigen3::Eigen
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(genz_lio_node ${cpp_typesupport_target})

ament_target_dependencies(genz_lio_node ${dependencies})

install(TARGETS genz_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
