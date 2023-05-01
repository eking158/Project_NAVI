#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

//ros_communication_message type
#include <geometry_msgs/Twist.h>
#include <navi_humanoid_msgs/Humanoid.h>
#include <navi_humanoid_msgs/Hands.h>
#include <dynamixel_sdk_examples/SyncSetPosition.h>

//custom header

//ros communication
ros::Publisher dynamixel_up_pub;    //up dynamixel control publisher
ros::Publisher dynamixel_middle_pub;    //middle dynamixel control publisher
ros::Publisher dynamixel_down_pub;  //down dynamixel control publisher
ros::Publisher hand_pub;            //hands fingers control publisher
ros::Publisher cmd_vel_pub;         //mobile robot velocity control publisher
ros::Subscriber data_sub;


//ros msg
dynamixel_sdk_examples::SyncSetPosition dynamixel_up_msg;    //up dynamixel control msg
dynamixel_sdk_examples::SyncSetPosition dynamixel_middle_msg;  //middle dynamixel control msg
dynamixel_sdk_examples::SyncSetPosition dynamixel_down_msg;  //down dynamixel control msg
geometry_msgs::Twist velocity_msg;                           //mobile robot velocity control msg
navi_humanoid_msgs::Hands hands_msg;                         //hands fingers control msg

//variables



//function
void initialize();

//callback
void GetDataCallback(const navi_humanoid_msgs::Humanoid& msg);

