#include <iostream>
#include <ros/ros.h>
#include <navi_old_humanoid_msgs/Humanoid.h>
#include <stdio.h>

ros::Publisher ControlPublisher; //init publisher
ros::Subscriber GetDataSubscriber; //init subscriber

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void counterCallback(const navi_old_humanoid_msgs::Humanoid& angle_data){
    ROS_INFO("servo_head_1: %f", angle_data.servo_head.servo1);
    ROS_INFO("servo_head_2: %f", angle_data.servo_head.servo2);
    ROS_INFO("servo_head_3: %f", angle_data.servo_head.servo3);

    ROS_INFO("servo_Left_Arm_1: %f", angle_data.servo_left_arm.servo1);
    ROS_INFO("servo_Left_Arm_2: %f", angle_data.servo_left_arm.servo2);
    ROS_INFO("servo_Left_Arm_3: %f", angle_data.servo_left_arm.servo3);
    ROS_INFO("servo_Left_Arm_4: %f", angle_data.servo_left_arm.servo4);
    ROS_INFO("servo_Left_Arm_5: %f", angle_data.servo_left_arm.servo5);
    ROS_INFO("servo_Left_Arm_6: %f", angle_data.servo_left_arm.servo6);
    ROS_INFO("servo_Left_Arm_7: %f", angle_data.servo_left_arm.servo7);

    ROS_INFO("servo_Right_Arm_1: %f", angle_data.servo_right_arm.servo1);
    ROS_INFO("servo_Right_Arm_2: %f", angle_data.servo_right_arm.servo2);
    ROS_INFO("servo_Right_Arm_3: %f", angle_data.servo_right_arm.servo3);
    ROS_INFO("servo_Right_Arm_4: %f", angle_data.servo_right_arm.servo4);
    ROS_INFO("servo_Right_Arm_5: %f", angle_data.servo_right_arm.servo5);
    ROS_INFO("servo_Right_Arm_6: %f", angle_data.servo_right_arm.servo6);
    ROS_INFO("servo_Right_Arm_7: %f", angle_data.servo_right_arm.servo7);

    ControlPublisher.publish(angle_data);
}

int main(int argc, char** argv)
{
  //init the ROS node
  ROS_INFO_STREAM("scanning...");
  ros::init(argc, argv, "navi_control_test_arduino_node");  //init node
  ros::NodeHandle nodeHandle;
  //operatorPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  GetDataSubscriber = nodeHandle.subscribe("/NAVI_angles",1000,counterCallback);
  ControlPublisher = nodeHandle.advertise<navi_old_humanoid_msgs::Humanoid>("/NAVI_Arduino",1);
  ros::spin();
  return 0;
}
