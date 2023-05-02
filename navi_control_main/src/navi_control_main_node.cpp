/*
head_yaw -> head_pitch -> head_roll
1 -> 2 -> 3

Left_shoulder_pitch -> Left_shoulder_roll -> Left_shoulder_yaw -> Left_elbow_pitch -> Left_elbow_yaw -> Left_wrist_pitch -> Left_wrist_roll
5 -> 7 -> 9 -> 11 -> 13 -> 15 -> 17

Right_shoulder_pitch -> Right_shoulder_roll -> Right_shoulder_yaw -> Right_elbow_pitch -> Right_elbow_yaw -> Right_wrist_pitch -> Right_wrist_roll
4 -> 6 -> 8 -> 10- > 12 -> 14 -> 16
*/

/*
<dynamixel up>->xm540 
Left_shoulder_pitch -> Left_shoulder_roll
5 -> 7

Right_shoulder_pitch -> Right_shoulder_roll
4 -> 6
-----------------------------------------------------------------
<dynamixel middle>->xm430
Left_shoulder_yaw -> Left_elbow_pitch
9 -> 11

Right_shoulder_yaw -> Right_elbow_pitch
8 -> 10
-----------------------------------------------------------------
<dynamixel down>->mx28 xl430
head_yaw -> head_pitch -> head_roll
1 -> 2 -> 3

Left_elbow_yaw -> Left_wrist_pitch -> Left_wrist_roll
13 -> 15 -> 17

Right_elbow_yaw -> Right_wrist_pitch -> Right_wrist_roll
12 -> 14 -> 16
*/



#include <navi_control_main/navi_control_main_node.h>

void initialize(){
  //up dynamixel id setting
    dynamixel_up_msg.id1=5;
    dynamixel_up_msg.id2=7;
    dynamixel_up_msg.id3=4;
    dynamixel_up_msg.id4=6;

  //middle dynamixel id setting
    dynamixel_middle_msg.id1=9;
    dynamixel_middle_msg.id2=11;
    dynamixel_middle_msg.id3=8;
    dynamixel_middle_msg.id4=10;

  //down dynamixel id setting
    dynamixel_down_msg.id1=1;
    dynamixel_down_msg.id2=2;
    dynamixel_down_msg.id3=3;
    dynamixel_down_msg.id4=13;
    dynamixel_down_msg.id5=15;
    dynamixel_down_msg.id6=17;
    dynamixel_down_msg.id7=12;
    dynamixel_down_msg.id8=14;
    dynamixel_down_msg.id9=16;

  //up dynamixel init position setting
    dynamixel_up_msg.position1=2040;
    dynamixel_up_msg.position2=1658;
    dynamixel_up_msg.position3=2040;
    dynamixel_up_msg.position4=2407;

  //middle dynamixel init position setting
    dynamixel_middle_msg.position1=2454;
    dynamixel_middle_msg.position2=2040;
    dynamixel_middle_msg.position3=1870;
    dynamixel_middle_msg.position4=2040;

  //down dynamixel init position setting
    dynamixel_down_msg.position1=2040;
    dynamixel_down_msg.position2=2040;
    dynamixel_down_msg.position3=2040;
    dynamixel_down_msg.position4=2040;
    dynamixel_down_msg.position5=2040;
    dynamixel_down_msg.position6=2040;
    dynamixel_down_msg.position7=2040;
    dynamixel_down_msg.position8=2040;
    dynamixel_down_msg.position9=2040;

  //init velocity setting
    velocity_msg.linear.x=0;
    velocity_msg.linear.y=0;
    velocity_msg.angular.z=0;

  //init fingers setting
    hands_msg.left_thumb = 0;
    hands_msg.left_index = 90;
    hands_msg.left_middle = 105;
    hands_msg.left_ring = 55;
    hands_msg.left_pinky = 90;
    hands_msg.right_thumb = 0;
    hands_msg.right_index = 0;
    hands_msg.right_middle = 0;
    hands_msg.right_ring = 180;
    hands_msg.right_pinky = 135;
}


void GetDataCallback(const navi_humanoid_msgs::Humanoid& msg){
  //up dynamixel position setting
    dynamixel_up_msg.position1 = msg.servo_left[0];
    dynamixel_up_msg.position2 = msg.servo_left[1];
    dynamixel_up_msg.position3 = msg.servo_right[0];
    dynamixel_up_msg.position4 = msg.servo_right[1];

  //middle dynamixel position setting
    dynamixel_middle_msg.position1 = msg.servo_left[2];
    dynamixel_middle_msg.position2 = msg.servo_left[3];
    dynamixel_middle_msg.position3 = msg.servo_right[2];
    dynamixel_middle_msg.position4 = msg.servo_right[3];

  //down dynamixel position setting
    dynamixel_down_msg.position1 = msg.servo_head[0];
    dynamixel_down_msg.position2 = msg.servo_head[1];
    dynamixel_down_msg.position3 = msg.servo_head[2];
    dynamixel_down_msg.position4 = msg.servo_left[4];
    dynamixel_down_msg.position5 = msg.servo_left[5];
    dynamixel_down_msg.position6 = msg.servo_left[6];
    dynamixel_down_msg.position7 = msg.servo_right[4];
    dynamixel_down_msg.position8 = msg.servo_right[5];
    dynamixel_down_msg.position9 = msg.servo_right[6];

  //velocity setting
    velocity_msg.linear.x = msg.cmd[0];
    velocity_msg.linear.y = msg.cmd[1];
    velocity_msg.angular.z = msg.cmd[2];

  //fingers setting
    hands_msg.left_thumb = msg.left_hands[0];
    hands_msg.left_index = msg.left_hands[1];
    hands_msg.left_middle = msg.left_hands[2];
    hands_msg.left_ring = msg.left_hands[3];
    hands_msg.left_pinky = msg.left_hands[4];
    hands_msg.right_thumb = msg.right_hands[0];
    hands_msg.right_index = msg.right_hands[1];
    hands_msg.right_middle = msg.right_hands[2];
    hands_msg.right_ring = msg.right_hands[3];
    hands_msg.right_pinky = msg.right_hands[4];
}

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Start");
  ros::init(argc, argv, "navi_control_main_node");  //init node
  ros::NodeHandle nh;
  initialize();

  dynamixel_up_pub = nh.advertise<navi_control_dynamixel::SyncSetPosition>("/navi/dynamicxel_set_position_up",1);
  dynamixel_middle_pub = nh.advertise<navi_control_dynamixel::SyncSetPosition>("/navi/dynamicxel_set_position_middle",1);
  dynamixel_down_pub = nh.advertise<navi_control_dynamixel::SyncSetPosition>("/navi/dynamicxel_set_position_down",1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/navi/cmd_vel",1);
  hand_pub = nh.advertise<navi_humanoid_msgs::Hands>("/navi/hand",1);
  
  data_sub = nh.subscribe("/navi/unity",1000,GetDataCallback);

  ros::Rate loopRate(30);
  
  while(ros::ok()){
    ros::spinOnce();
    
    dynamixel_up_pub.publish(dynamixel_up_msg);       //control up dynamixel motors
    dynamixel_middle_pub.publish(dynamixel_middle_msg);       //control middle dynamixel motors
    dynamixel_down_pub.publish(dynamixel_down_msg);   //control down dynamixel motors
    cmd_vel_pub.publish(velocity_msg);                //control dc motors with arduino
    hand_pub.publish(hands_msg);                   //control micro_servos with arduino
  
    loopRate.sleep();
    }
  return 0;
}
