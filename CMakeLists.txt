cmake_minimum_required(VERSION 2.8.3)
project(kitti_player)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  src
  include
  "/usr/include/eigen3"
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(kitti_player
  src/node.cpp
  src/kitti_player.cpp
)


target_link_libraries(kitti_player
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS kitti_player
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

