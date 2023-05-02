#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

//ros_communication_message type
#include <geometry_msgs/Twist.h>
#include <navi_proto_humanoid_msgs/Humanoid.h>
#include <navi_control_dynamixel/SyncSetPosition.h>

//custom header

//ros communication
ros::Publisher angle_pub; //init publisher
ros::Publisher velocity_pub; //init publisher
ros::Subscriber data_sub;


//ros msg
navi_control_dynamixel::SyncSetPosition angle_msg;
geometry_msgs::Twist velocity_msg;

//variables



//function
void initialize();

//callback
void GetDataCallback(const navi_proto_humanoid_msgs::Humanoid& msg);

