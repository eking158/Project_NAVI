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
Left_shoulder_pitch -> Left_shoulder_roll --> Left_elbow_pitch
5 -> 7 --> 11

Right_shoulder_pitch -> Right_shoulder_roll  --> Right_elbow_pitch
4 -> 6 --> 10
-----------------------------------------------------------------
<dynamixel middle>->xm430
Left_shoulder_yaw -> Left_wrist_pitch 
9 -> 15

Right_shoulder_yaw -> Right_wrist_pitch
8 -> 14
-----------------------------------------------------------------
<dynamixel down>->mx28 xl430
head_yaw -> head_pitch -> head_roll
1 -> 2 -> 3

Left_elbow_yaw -> Left_wrist_roll
13 -> 17

Right_elbow_yaw -> Right_wrist_roll
12 -> 16
*/



#include <navi_control_main/navi_control_main_node.h>
void GetBaseYaml(){
  std::string Base_path = ros::package::getPath("navi_control_main") + "/config/base_position.yaml"; //AB param yaml
  YAML::Node Base_doc = YAML::LoadFile(Base_path);
  base_head_yaw     = Base_doc["base_head_yaw"].as<int>();
  base_head_pitch     = Base_doc["base_head_pitch"].as<int>();
  base_head_roll     = Base_doc["base_head_roll"].as<int>();

  base_right_shoulder_pitch     = Base_doc["base_right_shoulder_pitch"].as<int>();
  base_right_shoulder_roll     = Base_doc["base_right_shoulder_roll"].as<int>();
  base_right_shoulder_yaw     = Base_doc["base_right_shoulder_yaw"].as<int>();
  base_right_elbow_pitch     = Base_doc["base_right_elbow_pitch"].as<int>();
  base_right_elbow_yaw     = Base_doc["base_right_elbow_yaw"].as<int>();
  base_right_wrist_pitch     = Base_doc["base_right_wrist_pitch"].as<int>();
  base_right_wrist_roll     = Base_doc["base_right_wrist_roll"].as<int>();

  base_left_shoulder_pitch     = Base_doc["base_left_shoulder_pitch"].as<int>();
  base_left_shoulder_roll     = Base_doc["base_left_shoulder_roll"].as<int>();
  base_left_shoulder_yaw     = Base_doc["base_left_shoulder_yaw"].as<int>();
  base_left_elbow_pitch     = Base_doc["base_left_elbow_pitch"].as<int>();
  base_left_elbow_yaw     = Base_doc["base_left_elbow_yaw"].as<int>();
  base_left_wrist_pitch     = Base_doc["base_left_wrist_pitch"].as<int>();
  base_left_wrist_roll     = Base_doc["base_left_wrist_roll"].as<int>();

  ROS_INFO("base_head_yaw: %d", base_head_yaw);
  ROS_INFO("base_head_pitch: %d", base_head_pitch);
  ROS_INFO("base_head_roll: %d", base_head_roll);

  ROS_INFO("base_right_shoulder_pitch: %d", base_right_shoulder_pitch);
  ROS_INFO("base_right_shoulder_roll: %d", base_right_shoulder_roll);
  ROS_INFO("base_right_shoulder_yaw: %d", base_right_shoulder_yaw);
  ROS_INFO("base_right_elbow_pitch: %d", base_right_elbow_pitch);
  ROS_INFO("base_right_elbow_yaw: %d", base_right_elbow_yaw);
  ROS_INFO("base_right_wrist_pitch: %d", base_right_wrist_pitch);
  ROS_INFO("base_right_wrist_roll: %d", base_right_wrist_roll);

  ROS_INFO("base_left_shoulder_pitch: %d", base_left_shoulder_pitch);
  ROS_INFO("base_left_shoulder_roll: %d", base_left_shoulder_roll);
  ROS_INFO("base_left_shoulder_yaw: %d", base_left_shoulder_yaw);
  ROS_INFO("base_left_elbow_pitch: %d", base_left_elbow_pitch);
  ROS_INFO("base_left_elbow_yaw: %d", base_left_elbow_yaw);
  ROS_INFO("base_left_wrist_pitch: %d", base_left_wrist_pitch);
  ROS_INFO("base_left_wrist_roll: %d", base_left_wrist_roll);
}

void initialize(){
  //up dynamixel id setting
  dynamixel_up_base_msg.id1=5;
  dynamixel_up_base_msg.id2=7;
  dynamixel_up_base_msg.id3=11;
  dynamixel_up_base_msg.id4=4;
  dynamixel_up_base_msg.id5=6;
  dynamixel_up_base_msg.id6=10;
  //middle dynamixel id setting
  dynamixel_middle_base_msg.id1=9;
  dynamixel_middle_base_msg.id2=15;
  dynamixel_middle_base_msg.id3=8;
  dynamixel_middle_base_msg.id4=14;
  //down dynamixel id setting
  dynamixel_down_base_msg.id1=1;
  dynamixel_down_base_msg.id2=2;
  dynamixel_down_base_msg.id3=3;
  dynamixel_down_base_msg.id4=13;
  dynamixel_down_base_msg.id5=17;
  dynamixel_down_base_msg.id6=12;
  dynamixel_down_base_msg.id7=16;
  //up dynamixel init position setting
  dynamixel_up_base_msg.position1=base_left_shoulder_pitch;  //id 5
  dynamixel_up_base_msg.position2=base_left_shoulder_roll;  //id 7
  dynamixel_up_base_msg.position3=base_left_elbow_pitch;  //id 11  (2000)
  dynamixel_up_base_msg.position4=base_right_shoulder_pitch;  //id 4
  dynamixel_up_base_msg.position5=base_right_shoulder_roll;  //id 6
  dynamixel_up_base_msg.position6=base_right_elbow_pitch;  //id 10  (2000)
  //middle dynamixel init position setting
  dynamixel_middle_base_msg.position1=base_left_shoulder_yaw;  //id 9
  dynamixel_middle_base_msg.position2=base_left_wrist_pitch;  //id 15
  dynamixel_middle_base_msg.position3=base_right_shoulder_yaw;  //id 8
  dynamixel_middle_base_msg.position4=base_right_wrist_pitch;  //id 14
  //down dynamixel init position setting
  dynamixel_down_base_msg.position1=base_head_yaw;  //id 1
  dynamixel_down_base_msg.position2=base_head_pitch;  //id 2
  dynamixel_down_base_msg.position3=base_head_roll;  //id 3
  dynamixel_down_base_msg.position4=base_left_elbow_yaw;  //id 13
  dynamixel_down_base_msg.position5=base_left_wrist_roll;  //id 17
  dynamixel_down_base_msg.position6=base_right_elbow_yaw;  //id 12
  dynamixel_down_base_msg.position7=base_right_wrist_roll;  //id 16
  //init velocity setting
  velocity_base_msg.linear.x=0;
  velocity_base_msg.linear.y=0;
  velocity_base_msg.angular.z=0;
  //init fingers setting
  hands_base_msg.left_thumb = 0;
  hands_base_msg.left_index = 90;
  hands_base_msg.left_middle = 105;
  hands_base_msg.left_ring = 55;
  hands_base_msg.left_pinky = 90;
  hands_base_msg.right_thumb = 0;
  hands_base_msg.right_index = 0;
  hands_base_msg.right_middle = 0;
  hands_base_msg.right_ring = 180;
  hands_base_msg.right_pinky = 135;
  //display setting
  display_base_msg.data = 1;
  //copy & paste base to unity
  dynamixel_up_msg = dynamixel_up_base_msg;
  dynamixel_middle_msg = dynamixel_middle_base_msg;
  dynamixel_down_msg = dynamixel_down_base_msg;
  velocity_msg = velocity_base_msg;
  hands_msg = hands_base_msg;
  display_msg = display_base_msg;
  //for callback check
  unity_callback_flag = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GetDataCallback(const navi_humanoid_msgs::Humanoid& msg){
    //up dynamixel position setting
    dynamixel_up_msg.position1 = msg.servo_left[0];  //5
    dynamixel_up_msg.position2 = msg.servo_left[1];  //7
    dynamixel_up_msg.position3 = msg.servo_left[3];  //11
    dynamixel_up_msg.position4 = msg.servo_right[0]; //4
    dynamixel_up_msg.position5 = msg.servo_right[1]; //6
    dynamixel_up_msg.position6 = msg.servo_right[3]; //10
    //middle dynamixel position setting
    dynamixel_middle_msg.position1 = msg.servo_left[2];  //9
    dynamixel_middle_msg.position2 = msg.servo_left[5];  //15
    dynamixel_middle_msg.position3 = msg.servo_right[2]; //8
    dynamixel_middle_msg.position4 = msg.servo_right[5]; //4
    //down dynamixel position setting
    dynamixel_down_msg.position1 = msg.servo_head[0];  //1
    dynamixel_down_msg.position2 = msg.servo_head[1];  //2
    dynamixel_down_msg.position3 = msg.servo_head[2];  //3
    dynamixel_down_msg.position4 = msg.servo_left[4];  //13
    dynamixel_down_msg.position5 = msg.servo_left[6];  //17
    dynamixel_down_msg.position6 = msg.servo_right[4]; //12
    dynamixel_down_msg.position7 = msg.servo_right[6]; //16
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
    //display setting
    display_msg.data = msg.etc[0];
}
//----------------------------------------------------------------------------------------------------------
void GetOnOffCallback(const std_msgs::Bool& msg){
  unity_callback_flag = msg.data;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Start");
  ros::init(argc, argv, "navi_control_main_node");  //init node
  ros::NodeHandle nh;
  GetBaseYaml();
  initialize();

  dynamixel_up_pub = nh.advertise<navi_control_dynamixel::SyncSetPosition>("/navi/dynamicxel_set_position_up",1);
  dynamixel_middle_pub = nh.advertise<navi_control_dynamixel::SyncSetPosition>("/navi/dynamicxel_set_position_middle",1);
  dynamixel_down_pub = nh.advertise<navi_control_dynamixel::SyncSetPosition>("/navi/dynamicxel_set_position_down",1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/navi/cmd_vel",1);
  hand_pub = nh.advertise<navi_humanoid_msgs::Hands>("/navi/hand",1);
  display_pub = nh.advertise<std_msgs::Int16>("/navi/display",1);
  
  data_sub = nh.subscribe("/navi/unity",1000,GetDataCallback);
  on_off_sub = nh.subscribe("/navi/on_off",1000,GetOnOffCallback);

  ros::Rate loopRate(HZ);
  
  while(ros::ok()){
    ros::spinOnce();
    if(unity_callback_flag){
      ROS_INFO("Unity");
      dynamixel_up_pub.publish(dynamixel_up_msg);                      //control up dynamixel motors (unity)
      dynamixel_middle_pub.publish(dynamixel_middle_msg);             //control middle dynamixel motors (unity)
      dynamixel_down_pub.publish(dynamixel_down_msg);                 //control down dynamixel motors (unity)
      cmd_vel_pub.publish(velocity_msg);                //control dc motors with arduino
      hand_pub.publish(hands_msg);                   //control micro_servos with arduino
      display_pub.publish(display_msg);                   //control display for showing face
    }
    else{
      ROS_INFO("Base");
      dynamixel_up_pub.publish(dynamixel_up_base_msg);               //control up dynamixel motors (base)
      dynamixel_middle_pub.publish(dynamixel_middle_base_msg);       //control middle dynamixel motors (base)
      dynamixel_down_pub.publish(dynamixel_down_base_msg);           //control down dynamixel motors (base)
      cmd_vel_pub.publish(velocity_base_msg);                //control dc motors with arduino
      hand_pub.publish(hands_base_msg);                   //control micro_servos with arduino
      display_pub.publish(display_base_msg);                   //control display for showing face
    }
    loopRate.sleep();
  }
  return 0;
}
