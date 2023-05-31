/*
head_yaw -> head_pitch -> head_roll
1 -> 2 -> 3

Left_shoulder_pitch -> Left_shoulder_roll -> Left_shoulder_yaw -> Left_elbow_pitch -> Left_elbow_yaw -> Left_wrist_pitch -> Left_wrist_roll
5 -> 7 -> 9 -> 11 -> 13 -> 15 -> 17

Right_shoulder_pitch -> Right_shoulder_roll -> Right_shoulder_yaw -> Right_elbow_pitch -> Right_elbow_yaw -> Right_wrist_pitch -> Right_wrist_roll
4 -> 6 -> 8 -> 10- > 12 -> 14 -> 16
*/

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <yaml-cpp/yaml.h> 
#include <ros/package.h>
#include <string>

//ros_communication_message type
#include <geometry_msgs/Twist.h>
#include <navi_humanoid_msgs/Humanoid.h>
#include <navi_humanoid_msgs/Hands.h>
#include <navi_control_dynamixel/SyncSetPosition.h>
#include <std_msgs/Int16.h>

//custom header

//ros communication
ros::Publisher dynamixel_up_pub;    //up dynamixel control publisher
ros::Publisher dynamixel_middle_pub;    //middle dynamixel control publisher
ros::Publisher dynamixel_down_pub;  //down dynamixel control publisher
ros::Publisher hand_pub;            //hands fingers control publisher
ros::Publisher cmd_vel_pub;         //mobile robot velocity control publisher
ros::Publisher display_pub;         //display face show control publisher
ros::Subscriber data_sub;


//ros msg
navi_control_dynamixel::SyncSetPosition dynamixel_up_msg;    //up dynamixel control msg
navi_control_dynamixel::SyncSetPosition dynamixel_middle_msg;  //middle dynamixel control msg
navi_control_dynamixel::SyncSetPosition dynamixel_down_msg;  //down dynamixel control msg
geometry_msgs::Twist velocity_msg;                           //mobile robot velocity control msg
navi_humanoid_msgs::Hands hands_msg;                         //hands fingers control msg
std_msgs::Int16 display_msg;                                 //display face control msg

//variables
int base_head_yaw;
int base_head_pitch;
int base_head_roll;

int base_left_shoulder_pitch;
int base_left_shoulder_roll;
int base_left_shoulder_yaw;
int base_left_elbow_pitch;
int base_left_elbow_yaw;
int base_left_wrist_pitch;
int base_left_wrist_roll;

int base_right_shoulder_pitch;
int base_right_shoulder_roll;
int base_right_shoulder_yaw;
int base_right_elbow_pitch;
int base_right_elbow_yaw;
int base_right_wrist_pitch;
int base_right_wrist_roll;

//function
void initialize();
void GetBaseYaml();

//callback
void GetDataCallback(const navi_humanoid_msgs::Humanoid& msg);

