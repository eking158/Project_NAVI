#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include <unistd.h>  //파일 존재 여부 확인용
using namespace std;

//ros_communication_message type
#include <std_msgs/Int16.h>

//define
#define PUB_HZ 50

//custom header

//ros communication
ros::Subscriber data_sub;

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

cv::Mat normal_face;
cv::Mat normal_face_rotate;
cv::Mat image_1;
cv::Mat image_2;


//function
void initialize();
void CheckImage(const char* path);
void GIFShow(string path, int* now_time, int frame_num, float frame_time);

//callback
void GetDataCallback(const std_msgs::Int16& msg);

