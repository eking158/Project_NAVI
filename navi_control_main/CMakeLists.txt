cmake_minimum_required(VERSION 3.0.2)
project(navi_control_main)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  navi_humanoid_msgs
  roscpp
  std_msgs
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navi_main_control
#  CATKIN_DEPENDS geometry_msgs navi_humanoid_msgs roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS} )

 add_executable(navi_control_main_node src/navi_control_main_node.cpp)
 target_link_libraries(navi_control_main_node ${catkin_LIBRARIES} yaml-cpp)
 add_dependencies(navi_control_main_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
