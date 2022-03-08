cmake_minimum_required(VERSION 3.0.2)
project(drix_project11)

find_package(catkin REQUIRED
  drix_msgs
  project11
  project11_msgs
  roscpp
  rospy
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS roscpp project11 project11_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robobox_interface src/robobox_interface.cpp)
target_link_libraries(robobox_interface ${catkin_LIBRARIES})
add_dependencies(robobox_interface ${catkin_EXPORTED_TARGETS})

add_executable(geopoint_to_asv src/geopoint_to_asv.cpp)
target_link_libraries(geopoint_to_asv ${catkin_LIBRARIES})
add_dependencies(geopoint_to_asv ${catkin_EXPORTED_TARGETS})

add_executable(geopose_to_asv src/geopose_to_asv.cpp)
target_link_libraries(geopose_to_asv ${catkin_LIBRARIES})
add_dependencies(geopose_to_asv ${catkin_EXPORTED_TARGETS})
