//#define USE_USBCON
#define BAUD 57600

#define LEFT_THUMB 21
#define LEFT_INDEX 8
#define LEFT_MIDDLE 9
#define LEFT_RING 10
#define LEFT_PINKY 11

#define RIGHT_THUMB 31
#define RIGHT_INDEX 3
#define RIGHT_MIDDLE 4
#define RIGHT_RING 5
#define RIGHT_PINKY 6
/////////////////////////////////////////////////////////////
#include <ros.h>
#include <Servo.h>
#include <navi_humanoid_msgs/Hands.h>

ros::NodeHandle  nh;
Servo left_thumb, left_index, left_middle, left_ring, left_pinky;
Servo right_thumb, right_index, right_middle, right_ring, right_pinky;
/////////////////////////////////////////////////////////////
float left_fingers[5] = {0, 0, 0, 0, 0};
float right_fingers[5] = {0, 0, 0, 0, 0};
/////////////////////////////////////////////////////////////

void navi_hand_callback(const navi_humanoid_msgs::Hands& msg){
  //get left fingers angle data
  left_fingers[0] = msg.left_thumb;
  left_fingers[1] = msg.left_index;
  left_fingers[2] = msg.left_middle;
  left_fingers[3] = msg.left_ring;
  left_fingers[4] = msg.left_pinky;
  //get right fingers angle data
  right_fingers[0] = msg.right_thumb;
  right_fingers[1] = msg.right_index;
  right_fingers[2] = msg.right_middle;
  right_fingers[3] = msg.right_ring;
  right_fingers[4] = msg.right_pinky;

  //control left fingers
  left_thumb.write(left_fingers[0]);
  left_index.write(left_fingers[1]);
  left_middle.write(left_fingers[2]);
  left_ring.write(left_fingers[3]);
  left_pinky.write(left_fingers[4]);
  //control right fingers
  right_thumb.write(right_fingers[0]);
  right_index.write(right_fingers[1]);
  right_middle.write(right_fingers[2]);
  right_ring.write(right_fingers[3]);
  right_pinky.write(right_fingers[4]);
}


ros::Subscriber<navi_humanoid_msgs::Hands> sub("/navi/hand", navi_hand_callback);

void setup(){
  nh.initNode();
  nh.subscribe(sub);

  left_thumb.attach(LEFT_THUMB);
  left_index.attach(LEFT_INDEX);
  left_middle.attach(LEFT_MIDDLE);
  left_ring.attach(LEFT_RING);
  left_pinky.attach(LEFT_PINKY);

  right_thumb.attach(RIGHT_THUMB);
  right_index.attach(RIGHT_INDEX);
  right_middle.attach(RIGHT_MIDDLE);
  right_ring.attach(RIGHT_RING);
  right_pinky.attach(RIGHT_PINKY);
}

void loop(){
  nh.spinOnce();
  delay(1);
}

//////////////////////////////////////////////////////////////////////////
