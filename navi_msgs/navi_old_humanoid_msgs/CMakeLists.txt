cmake_minimum_required(VERSION 3.0.2)
project(navi_old_humanoid_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


add_message_files(
  FILES
  Head.msg
  Leftarm.msg
  Rightarm.msg
  Hand.msg
  Vector3.msg
  Humanoid.msg
)


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navi_old_humanoid_msgs
#  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
