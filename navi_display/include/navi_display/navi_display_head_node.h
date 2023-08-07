#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <yaml-cpp/yaml.h> 
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
#define PUB_HZ 30

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

//gif frame parameters
int gif_frame_num_1;
int gif_frame_num_2;
int gif_frame_num_3;
int gif_frame_num_4;
int gif_frame_num_5;
int gif_frame_num_6;
int gif_frame_num_7;

float gif_frame_time_1;
float gif_frame_time_2;
float gif_frame_time_3;
float gif_frame_time_4;
float gif_frame_time_5;
float gif_frame_time_6;
float gif_frame_time_7;

//function
void initialize();
void CheckImage(const char* path);
void GIFShow(string path, int* now_time, int frame_num, float frame_time);
void GetEyeParamYaml();

//callback
void GetDataCallback(const std_msgs::Int16& msg);

