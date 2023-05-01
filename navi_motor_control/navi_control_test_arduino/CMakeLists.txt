cmake_minimum_required(VERSION 3.0.2)
project(navi_control_test_arduino)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  navi_old_humanoid_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navi_control_test
  CATKIN_DEPENDS roscpp std_msgs navi_old_humanoid_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

 add_executable(navi_control_test_arduino_node src/navi_control_test_arduino_node.cpp)
 target_link_libraries(navi_control_test_arduino_node ${catkin_LIBRARIES})
 add_dependencies(navi_control_test_arduino_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
