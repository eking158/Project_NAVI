################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(navi_control_dynamixel)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    message_generation
    dynamixel_sdk
    roslib
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SetPosition.msg
    SyncSetPosition.msg
    BulkSetItem.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    GetPosition.srv
    SyncGetPosition.srv
    BulkGetItem.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
    roslib
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(navi_dynamixel_up_node src/navi_dynamixel_up_node.cpp)
add_dependencies(navi_dynamixel_up_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navi_dynamixel_up_node ${catkin_LIBRARIES} yaml-cpp)

add_executable(navi_dynamixel_middle_node src/navi_dynamixel_middle_node.cpp)
add_dependencies(navi_dynamixel_middle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navi_dynamixel_middle_node ${catkin_LIBRARIES} yaml-cpp)

add_executable(navi_dynamixel_down_node src/navi_dynamixel_down_node.cpp)
add_dependencies(navi_dynamixel_down_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navi_dynamixel_down_node ${catkin_LIBRARIES} yaml-cpp)

################################################################################
# Install
################################################################################
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
