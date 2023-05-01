# navi_main
NAVI project

roslaunch file_server ros_sharp_communication.launch

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1 _baud:=57600

rosrun navi_control_main navi_control_main_node

rosrun dynamixel_sdk_examples navi_dynamixel_up_node
rosrun dynamixel_sdk_examples navi_dynamixel_down_node 
