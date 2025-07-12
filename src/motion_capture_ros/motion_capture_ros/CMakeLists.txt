cmake_minimum_required(VERSION 3.5)
project(motion_capture_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Libmotioncapture options
set(LIBMOTIONCAPTURE_ENABLE_VICON ON CACHE BOOL "" FORCE)

set(LIBMOTIONCAPTURE_ENABLE_QUALISYS OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_ENABLE_OPTITRACK OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_ENABLE_NOKOV OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_ENABLE_VRPN OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_ENABLE_FZMOTION OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/libmotioncapture
            ${CMAKE_BINARY_DIR}/external/libmotioncapture)


find_package(catkin REQUIRED COMPONENTS roscpp tf motion_capture_ros_msgs)
# find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf
  DEPENDS Eigen3
)

add_executable(motion_capture_node src/motion_capture.cpp)

target_include_directories(motion_capture_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/libmotioncapture/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/libmotioncapture/deps/vicon-datastream-sdk/Vicon/CrossMarket/DataStream
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(motion_capture_node ${catkin_LIBRARIES} libmotioncapture)

