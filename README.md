# NAVI Node Command

### Communicate with Unity
roslaunch file_server ros_sharp_communication.launch

### Dynamixel Control Node
rosrun navi_control_dynamixel navi_dynamixel_up_node

rosrun navi_control_dynamixel navi_dynamixel_middle_node

rosrun navi_control_dynamixel navi_dynamixel_down_node

### Arduino Control Node
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1 _baud:=57600

### MD Motor Control Node
roslaunch md md_robot_node.launch 

### Main Control Node
rosrun navi_control_main navi_control_main_node

### Streaming Camera Node
rosrun navi_video_streaming navi_video_streaming_node 0