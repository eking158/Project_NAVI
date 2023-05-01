/*
motor velocity contril: use dynamixel_workbench (wheel operator)
*/



#include <navi_control_test_dynamixel/navi_control_test_dynamixel_node.h>

void initialize(){
    angle_msg.id1=99;
    angle_msg.id2=2;
    angle_msg.id3=3;
    angle_msg.id4=4;
    angle_msg.id5=5;
    angle_msg.id6=6;
    angle_msg.id7=7;
    angle_msg.id8=8;
    angle_msg.id9=9;

    angle_msg.position2=2040;
    angle_msg.position4=2040;
    angle_msg.position6=1246;

    angle_msg.position3=2040;
    angle_msg.position5=2040;
    angle_msg.position7=1246;

    angle_msg.position8=2040;
    angle_msg.position9=2040;

    velocity_msg.linear.x=0;
    velocity_msg.angular.z=0;
}


void GetDataCallback(const navi_proto_humanoid_msgs::Humanoid& msg){
    angle_msg.id2=2;
    angle_msg.id3=3;
    angle_msg.id4=4;
    angle_msg.id5=5;
    angle_msg.id6=6;
    angle_msg.id7=7;
    angle_msg.id8=8;
    angle_msg.id9=9;

    angle_msg.position2=msg.servo_left[0];
    angle_msg.position4=msg.servo_left[1];
    angle_msg.position6=msg.servo_left[2];

    angle_msg.position3=msg.servo_right[0];
    angle_msg.position5=msg.servo_right[1];
    angle_msg.position7=msg.servo_right[2];

    angle_msg.position8=msg.servo_head[0];
    angle_msg.position9=msg.servo_head[1];

    velocity_msg.linear.x=msg.linear;
    velocity_msg.angular.z=msg.angular;
}

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Start");
  ros::init(argc, argv, "navi_control_test_dynamixel_node");  //init node
  ros::NodeHandle nh;
  initialize();

  angle_pub = nh.advertise<dynamixel_sdk_examples::SyncSetPosition>("/dynamicxel_set_position",1);
  velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  data_sub = nh.subscribe("/navi/unity",1000,GetDataCallback);

  ros::Rate loopRate(30);
  
  while(ros::ok()){
    ros::spinOnce();
    angle_pub.publish(angle_msg);
    velocity_pub.publish(velocity_msg);
    loopRate.sleep();
    }
  return 0;
}
