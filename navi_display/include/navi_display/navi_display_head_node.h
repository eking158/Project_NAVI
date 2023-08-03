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
string moving_eye_face_path_1;
string moving_eye_face_path_2;
string moving_eye_face_path_3;

cv::Mat normal_face;
cv::Mat normal_face_rotate;
cv::Mat moving_eye_face_1;
cv::Mat moving_eye_face_2;
cv::Mat moving_eye_face_3;
cv::Mat moving_eye_face_rotate_1;
cv::Mat moving_eye_face_rotate_2;
cv::Mat moving_eye_face_rotate_3;

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

//eye parameters (ver_1)
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

float eye_blink_time;                    //ui에서 눈을 감는 시간
float eye_open_time;                     //ui에서 눈을 뜨고 있는 시간

float eye_blink_major_ratio;             //ui에서 눈 감을때의 장축 감소 비율
float eye_blink_minor_ratio;             //ui에서 눈 감을때의 단축 감소 비율

//eye parameters (ver_2 minions)
geometry_msgs::Pose2D minions_eye_base_1;    //미니언즈 눈의 기본 위치
float minions_eye_major_length_1;            //미니언즈 눈의 장축 길이
float minions_eye_minor_length_1;            //미니언즈 눈의 단축 길이
float minions_eye_rotate_angle_1;            //미니언즈 눈의 회전 각도
float minions_eye_r_1;                       //미니언즈 눈의 색상(R)
float minions_eye_g_1;                       //미니언즈 눈의 색상(G)
float minions_eye_b_1;                       //미니언즈 눈의 색상(B)
float minions_eye_thick_1;                   //미니언즈 눈의 굵기 (-1이면 내부 채움)

geometry_msgs::Pose2D minions_eye_base_2;    //미니언즈 눈의 기본 위치
float minions_eye_major_length_2;            //미니언즈 눈의 장축 길이
float minions_eye_minor_length_2;            //미니언즈 눈의 단축 길이
float minions_eye_rotate_angle_2;            //미니언즈 눈의 회전 각도
float minions_eye_r_2;                       //미니언즈 눈의 색상(R)
float minions_eye_g_2;                       //미니언즈 눈의 색상(G)
float minions_eye_b_2;                       //미니언즈 눈의 색상(B)
float minions_eye_thick_2;                   //미니언즈 눈의 굵기 (-1이면 내부 채움)

geometry_msgs::Pose2D minions_eye_base_3;    //미니언즈 눈의 기본 위치
float minions_eye_major_length_3;            //미니언즈 눈의 장축 길이
float minions_eye_minor_length_3;            //미니언즈 눈의 단축 길이
float minions_eye_rotate_angle_3;            //미니언즈 눈의 회전 각도
float minions_eye_r_3;                       //미니언즈 눈의 색상(R)
float minions_eye_g_3;                       //미니언즈 눈의 색상(G)
float minions_eye_b_3;                       //미니언즈 눈의 색상(B)
float minions_eye_thick_3;                   //미니언즈 눈의 굵기 (-1이면 내부 채움)

geometry_msgs::Pose2D minions_eye_base_4;    //미니언즈 눈의 기본 위치
float minions_eye_major_length_4;            //미니언즈 눈의 장축 길이
float minions_eye_minor_length_4;            //미니언즈 눈의 단축 길이
float minions_eye_rotate_angle_4;            //미니언즈 눈의 회전 각도
float minions_eye_r_4;                       //미니언즈 눈의 색상(R)
float minions_eye_g_4;                       //미니언즈 눈의 색상(G)
float minions_eye_b_4;                       //미니언즈 눈의 색상(B)
float minions_eye_thick_4;                   //미니언즈 눈의 굵기 (-1이면 내부 채움)

float minions_eye_moving_x_min;              //ui에서 실제 움직이는 eye의 x 최솟값
float minions_eye_moving_x_max;              //ui에서 실제 움직이는 eye의 x 최댓값
float minions_eye_moving_y_min;              //ui에서 실제 움직이는 eye의 y 최솟값
float minions_eye_moving_y_max;              //ui에서 실제 움직이는 eye의 y 최댓값

float minions_eye_blink_time;                //ui에서 눈을 감는 시간
float minions_eye_open_time;                 //ui에서 눈을 뜨고 있는 시간

float eye_shine_ratio;                       //ui에서 눈 주변 빛 번짐의 정도

//eye parameters (ver_3 thomas)
geometry_msgs::Pose2D thomas_left_eye_base;    //왼쪽 눈의 기본 위치
float thomas_left_eye_major_length;            //왼쪽 눈의 장축 길이
float thomas_left_eye_minor_length;            //왼쪽 눈의 단축 길이
float thomas_left_eye_rotate_angle;            //왼쪽 눈의 회전 각도
float thomas_left_eye_r;                       //왼쪽 눈의 색상(R)
float thomas_left_eye_g;                       //왼쪽 눈의 색상(G)
float thomas_left_eye_b;                       //왼쪽 눈의 색상(B)
float thomas_left_eye_thick;                   //왼쪽 눈의 굵기 (-1이면 내부 채움)

geometry_msgs::Pose2D thomas_right_eye_base;    //오른쪽 눈의 기본 위치
float thomas_right_eye_major_length;            //오른쪽 눈의 장축 길이
float thomas_right_eye_minor_length;            //오른쪽 눈의 단축 길이
float thomas_right_eye_rotate_angle;            //오른쪽 눈의 회전 각도
float thomas_right_eye_r;                       //오른쪽 눈의 색상(R)
float thomas_right_eye_g;                       //오른쪽 눈의 색상(G)
float thomas_right_eye_b;                       //오른쪽 눈의 색상(B)
float thomas_right_eye_thick;                   //오른쪽 눈의 굵기 (-1이면 내부 채움)

float thomas_eye_tracking_x_min;                  //unity에서 출력되는 eye의 x 최솟값
float thomas_eye_tracking_x_max;                  //unity에서 출력되는 eye의 x 최댓값
float thomas_eye_tracking_y_min;                  //unity에서 출력되는 eye의 y 최솟값
float thomas_eye_tracking_y_max;                  //unity에서 출력되는 eye의 y 최댓값

float thomas_eye_moving_x_min;                  //ui에서 실제 움직이는 eye의 x 최솟값
float thomas_eye_moving_x_max;                  //ui에서 실제 움직이는 eye의 x 최댓값
float thomas_eye_moving_y_min;                  //ui에서 실제 움직이는 eye의 y 최솟값
float thomas_eye_moving_y_max;                  //ui에서 실제 움직이는 eye의 y 최댓값

float thomas_eye_blink_time;                    //ui에서 눈을 감는 시간
float thomas_eye_open_time;                     //ui에서 눈을 뜨고 있는 시간

geometry_msgs::Pose2D left_eye_msgs;
geometry_msgs::Pose2D right_eye_msgs;

//function
void initialize();
void CheckImage(const char* path);
void GIFShow(string path, int* now_time, int frame_num, float frame_time);
void MovingEyeMinions(string path, int* now_time, float frame_time_blink, float frame_time_rest);
void MovingEyeNormal(string path, int* now_time, float frame_time_blink, float frame_time_rest);
void GetEyeParamYaml();
float float_map(float x, float in_min, float in_max, float out_min, float out_max);

//callback
void GetDataCallback(const std_msgs::Int16& msg);
void GetLeftEyeCallback(const geometry_msgs::Pose2D& msg);
void GetRightEyeCallback(const geometry_msgs::Pose2D& msg);

