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
string moving_eye_face_path_1;

cv::Mat normal_face;
cv::Mat normal_face_rotate;
cv::Mat moving_eye_face_1;
cv::Mat moving_eye_face_rotate_1;

//eye parameters
geometry_msgs::Pose2D left_eye_base;    //왼쪽 눈의 기본 위치
float left_eye_major_length;            //왼쪽 눈의 장축 길이
float left_eye_minor_length;            //왼쪽 눈의 단축 길이
float left_eye_rotate_angle;            //왼쪽 눈의 회전 각도
float left_eye_r;                       //왼쪽 눈의 색상(R)
float left_eye_g;                       //왼쪽 눈의 색상(G)
float left_eye_b;                       //왼쪽 눈의 색상(B)
float left_eye_thick;                   //왼쪽 눈의 굵기 (-1이면 내부 채움)

geometry_msgs::Pose2D right_eye_base;    //오른쪽 눈의 기본 위치
float right_eye_major_length;            //오른쪽 눈의 장축 길이
float right_eye_minor_length;            //오른쪽 눈의 단축 길이
float right_eye_rotate_angle;            //오른쪽 눈의 회전 각도
float right_eye_r;                       //오른쪽 눈의 색상(R)
float right_eye_g;                       //오른쪽 눈의 색상(G)
float right_eye_b;                       //오른쪽 눈의 색상(B)
float right_eye_thick;                   //오른쪽 눈의 굵기 (-1이면 내부 채움)

float eye_tracking_x_min;                  //unity에서 출력되는 eye의 x 최솟값
float eye_tracking_x_max;                  //unity에서 출력되는 eye의 x 최댓값
float eye_tracking_y_min;                  //unity에서 출력되는 eye의 y 최솟값
float eye_tracking_y_max;                  //unity에서 출력되는 eye의 y 최댓값

float eye_moving_x_min;                  //ui에서 실제 움직이는 eye의 x 최솟값
float eye_moving_x_max;                  //ui에서 실제 움직이는 eye의 x 최댓값
float eye_moving_y_min;                  //ui에서 실제 움직이는 eye의 y 최솟값
float eye_moving_y_max;                  //ui에서 실제 움직이는 eye의 y 최댓값


geometry_msgs::Pose2D left_eye_msgs;
geometry_msgs::Pose2D right_eye_msgs;

//function
void initialize();
void CheckImage(const char* path);
void GIFShow(string path, int* now_time, int frame_num, float frame_time);
void GetEyeParamYaml();
float float_map(float x, float in_min, float in_max, float out_min, float out_max);

//callback
void GetDataCallback(const std_msgs::Int16& msg);
void GetLeftEyeCallback(const geometry_msgs::Pose2D& msg);
void GetRightEyeCallback(const geometry_msgs::Pose2D& msg);

