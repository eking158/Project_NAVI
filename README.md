# NAVI Node Command

### Communicate with Unity
roslaunch file_server ros_sharp_communication.launch

### Main Control Node
rosrun navi_control_main navi_control_main_node

### Streaming Camera Node
rosrun navi_video_streaming navi_video_streaming_node 0

### Show Face gif at Display Node
rosrun navi_display navi_display_head_node

### Control All Dynamixel node Launch
roslaunch navi_control_main navi_dynamixel.launch

### Control All Arduino node Launch
roslaunch navi_control_main navi_arduino.launch

# 개별 노드
### Dynamixel Control Node
rosrun navi_control_dynamixel navi_dynamixel_up_node

rosrun navi_control_dynamixel navi_dynamixel_middle_node

rosrun navi_control_dynamixel navi_dynamixel_down_node

### Arduino Control Node
rosrun rosserial_python serial_node.py _port:=/dev/navi_hand _baud:=57600 __node:=navi_hand

rosrun rosserial_python serial_node.py _port:=/dev/navi_dc _baud:=57600  __node:=navi_dc

### MD Motor Control Node
roslaunch md md_robot_node.launch 