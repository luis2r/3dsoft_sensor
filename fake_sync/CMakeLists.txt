cmake_minimum_required(VERSION 2.8.3)
project(fake_sync)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
  OpenCV
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fake_sync
  CATKIN_DEPENDS cv_bridge image_geometry roscpp sensor_msgs std_msgs camera_info_manager image_transport
  DEPENDS system_lib
)

set( CMAKE_CXX_FLAGS "-Wall -O3 -msse3 -fopenmp" )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fake_sync src/nodes/fake_sync.cpp)

target_link_libraries(fake_sync
   ${catkin_LIBRARIES}
 )
