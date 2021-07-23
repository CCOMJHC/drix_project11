cmake_minimum_required(VERSION 3.0.2)
project(drix_project11)

find_package(catkin REQUIRED
  drix_msgs
  project11_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robobox_interface src/robobox_interface.cpp)

target_link_libraries(robobox_interface ${catkin_LIBRARIES})

add_dependencies(robobox_interface ${catkin_EXPORTED_TARGETS})
