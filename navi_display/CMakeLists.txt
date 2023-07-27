cmake_minimum_required(VERSION 3.0.2)
project(navi_display)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport sensor_msgs std_msgs)


catkin_package(CATKIN_DEPENDS cv_bridge image_transport sensor_msgs std_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the display control node
add_executable(navi_display_head_node src/navi_display_head_node.cpp)
add_dependencies(navi_display_head_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(navi_display_head_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)


# Mark executables and/or libraries for installation
install(TARGETS navi_display_head_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
