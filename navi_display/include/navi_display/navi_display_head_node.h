#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include <unistd.h>  //파일 존재 여부 확인용
using namespace std;

//ros_communication_message type
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>

//define
#define PUB_HZ 50

//custom header

//ros communication
ros::Subscriber data_sub;
ros::Subscriber left_eye_sub;
ros::Subscriber right_eye_sub;

//ros msg

//variables
int display_head;
int time_stack;
int num_gif;
string num_gif_char;
string file_path_origin;
string file_path_1;
string file_path_2;
string gif_file_path_1;
string gif_file_path_2;
string normal_face_path;
string face_path_1;
string face_path_2;
string face_path_3;
string face_path_4;
string face_path_5;
string face_path_6;
string face_path_7;

cv::Mat normal_face;
cv::Mat normal_face_rotate;
cv::Mat image_1;
cv::Mat image_2;
geometry_msgs::Pose2D left_eye_base;    //왼쪽 눈의 기본 위치
geometry_msgs::Pose2D right_eye_base;   //오른쪽 눈의 기본 위치

geometry_msgs::Pose2D left_eye_msgs;
geometry_msgs::Pose2D right_eye_msgs;

//function
void initialize();
void CheckImage(const char* path);
void GIFShow(string path, int* now_time, int frame_num, float frame_time);

//callback
void GetDataCallback(const std_msgs::Int16& msg);
void GetLeftEyeCallback(const geometry_msgs::Pose2D& msg);
void GetRightEyeCallback(const geometry_msgs::Pose2D& msg);

