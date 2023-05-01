#include <iostream>
#include <ros/ros.h>
#include <navi_proto_humanoid_msgs/Humanoid.h>
#include <stdio.h>

ros::Publisher ControlPublisher; //init publisher
ros::Subscriber GetDataSubscriber; //init subscriber

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void counterCallback(const navi_proto_humanoid_msgs::Humanoid& angle_data){
    ROS_INFO("servo_head_1: %f", angle_data.servo_head[0]);

    ControlPublisher.publish(angle_data);
}

int main(int argc, char** argv)
{
  //init the ROS node
  ROS_INFO_STREAM("scanning...");
  ros::init(argc, argv, "navi_control_test_opnerb_node");  //init node
  ros::NodeHandle nodeHandle;
  //operatorPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  GetDataSubscriber = nodeHandle.subscribe("/navi/unity",1000,counterCallback);
  ControlPublisher = nodeHandle.advertise<navi_proto_humanoid_msgs::Humanoid>("/navi/openrb",1);
  ros::spin();
  return 0;
}
