#include <navi_display/navi_display_head_node.h>

///////////////////////////////////////////////////////////////////////////////////////////
void CheckImage(const char* path){
  int ret = access(path, F_OK);
  if (ret == 0) {
	  ROS_INFO("yes");
  } 
  else {
    ROS_INFO("No");
  }
}
//-----------------------------------------------------------------------------------------
float float_map(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
///////////////////////////////////////////////////////////////////////////////////////////
void GIFShow(string path, int* now_time, int frame_num, float frame_time){
  if(*now_time > frame_time*PUB_HZ){
        if(num_gif>=frame_num) num_gif=1;
        else num_gif++;
        //ROS_INFO("%d", num_gif);
        num_gif_char=to_string(num_gif);
        cv::Mat gif = cv::imread(path+num_gif_char+".png", cv::IMREAD_COLOR);
        cv::Mat gif_rotate;
        cv::rotate(gif, gif_rotate, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow("face", gif_rotate);
        cv::waitKey(10);
        *now_time=0;
      }
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetEyeParamYaml(){
  std::string Eye_path = ros::package::getPath("navi_display") + "/config/eye_data.yaml"; //AB param yaml
  YAML::Node Eye_doc = YAML::LoadFile(Eye_path);
  left_eye_base.x           = Eye_doc["left_eye_base_x"].as<int>();
  left_eye_base.y           = Eye_doc["left_eye_base_y"].as<int>();
  left_eye_major_length     = Eye_doc["left_eye_major_length"].as<int>();
  left_eye_minor_length     = Eye_doc["left_eye_minor_length"].as<int>();
  left_eye_rotate_angle     = Eye_doc["left_eye_rotate_angle"].as<int>();
  left_eye_r                = Eye_doc["left_eye_r"].as<int>();
  left_eye_g                = Eye_doc["left_eye_g"].as<int>();
  left_eye_b                = Eye_doc["left_eye_b"].as<int>();
  left_eye_thick            = Eye_doc["left_eye_thick"].as<int>();

  right_eye_base.x           = Eye_doc["right_eye_base_x"].as<int>();
  right_eye_base.y           = Eye_doc["right_eye_base_y"].as<int>();
  right_eye_major_length     = Eye_doc["right_eye_major_length"].as<int>();
  right_eye_minor_length     = Eye_doc["right_eye_minor_length"].as<int>();
  right_eye_rotate_angle     = Eye_doc["right_eye_rotate_angle"].as<int>();
  right_eye_r                = Eye_doc["right_eye_r"].as<int>();
  right_eye_g                = Eye_doc["right_eye_g"].as<int>();
  right_eye_b                = Eye_doc["right_eye_b"].as<int>();
  right_eye_thick            = Eye_doc["right_eye_thick"].as<int>();

  eye_tracking_x_min            = Eye_doc["eye_tracking_x_min"].as<int>();
  eye_tracking_x_max            = Eye_doc["eye_tracking_x_max"].as<int>();
  eye_tracking_y_min            = Eye_doc["eye_tracking_y_min"].as<int>();
  eye_tracking_y_max            = Eye_doc["eye_tracking_y_max"].as<int>();

  eye_moving_x_min            = Eye_doc["eye_moving_x_min"].as<int>();
  eye_moving_x_max            = Eye_doc["eye_moving_x_max"].as<int>();
  eye_moving_y_min            = Eye_doc["eye_moving_y_min"].as<int>();
  eye_moving_y_max            = Eye_doc["eye_moving_y_max"].as<int>();
}
///////////////////////////////////////////////////////////////////////////////////////////
void initialize(){
  display_head=0;
  time_stack = 0;
  num_gif = 1;
  //file path setting for switching MCU
  //file_path_origin = "/home/ubuntu/catkin_ws/src/Project_NAVI/navi_display/src/";    //for rpi
  //file_path_origin = "/home/eking/NAVI_ws/src/navi_main/navi_display/src/";      //for Notebook
  std::string file_path_origin = ros::package::getPath("navi_display")+"/src/";
  //image file path (for test)
  gif_file_path_1 = file_path_origin+"images/gif_test_1/";
  gif_file_path_2 = file_path_origin+"images/gif_test_2/";
  //image file path (for face)
  normal_face_path = file_path_origin+"images/normal_face.png";
  face_path_1 = file_path_origin+"images/face_1/";
  face_path_2 = file_path_origin+"images/face_2/";
  face_path_3 = file_path_origin+"images/face_3/";
  face_path_4 = file_path_origin+"images/face_4/";
  face_path_5 = file_path_origin+"images/face_5/";
  face_path_6 = file_path_origin+"images/face_6/";
  face_path_7 = file_path_origin+"images/face_7/";
  moving_eye_face_path_1 = file_path_origin+"images/moving_eye_1.png";

  //CheckImage(moving_eye_face_path_1);
  normal_face = cv::imread(normal_face_path, cv::IMREAD_COLOR);
  moving_eye_face_1 = cv::imread(moving_eye_face_path_1, cv::IMREAD_COLOR);
  left_eye_msgs.x = 0;
  left_eye_msgs.y = 0;
  right_eye_msgs.x = 0;
  right_eye_msgs.y = 0;
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetDataCallback(const std_msgs::Int16& msg){
    display_head = msg.data;
    time_stack++;
    switch(display_head){
      case 0:
      //cv::rotate(normal_face, normal_face_rotate, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::rotate(moving_eye_face_1, moving_eye_face_rotate_1, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::ellipse(moving_eye_face_rotate_1, cv::Point(left_eye_base.y+left_eye_msgs.y, left_eye_base.x+left_eye_msgs.x), cv::Size(left_eye_major_length, left_eye_minor_length), left_eye_rotate_angle, 0, 360, cv::Scalar(left_eye_b, left_eye_g, left_eye_r), left_eye_thick, 8);
      cv::ellipse(moving_eye_face_rotate_1, cv::Point(right_eye_base.y+right_eye_msgs.y, right_eye_base.x+right_eye_msgs.x), cv::Size(right_eye_major_length, right_eye_minor_length), right_eye_rotate_angle, 0, 360, cv::Scalar(right_eye_b, right_eye_g, right_eye_r), right_eye_thick, 8);
      cv::imshow("face", moving_eye_face_rotate_1);
      cv::waitKey(10);
      break;

      case 1:
      GIFShow(face_path_1, &time_stack, 16, 0.1);
      break;

      case 2:
      GIFShow(face_path_2, &time_stack, 24, 0.1);
      break;

      case 3:
      GIFShow(face_path_3, &time_stack, 2, 0.4);
      break;

      case 4:
      GIFShow(face_path_4, &time_stack, 32, 0.15);
      break;

      case 5:
      GIFShow(face_path_5, &time_stack, 32, 0.07);
      break;

      case 6:
      GIFShow(face_path_6, &time_stack, 24, 0.1);
      break;

      case 7:
      GIFShow(face_path_7, &time_stack, 17, 0.15);
      break;

      case 12:
      GIFShow(gif_file_path_1, &time_stack, 12, 0.4);
      break;

      case 13:
      GIFShow(gif_file_path_2, &time_stack, 13, 0.2);
      break;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetLeftEyeCallback(const geometry_msgs::Pose2D& msg){
  left_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, eye_moving_x_min, eye_moving_x_max);
  left_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, eye_moving_y_min, eye_moving_y_max);
}
//-----------------------------------------------------------------------------------------
void GetRightEyeCallback(const geometry_msgs::Pose2D& msg){
  right_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, eye_moving_x_min, eye_moving_x_max);
  right_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, eye_moving_y_min, eye_moving_y_max);
}
///////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "navi_display_head_node");
  ros::NodeHandle nh;
  initialize();
  GetEyeParamYaml();

  cv::namedWindow("face", cv::WindowFlags::WINDOW_NORMAL);
  cv::setWindowProperty("face", cv::WindowPropertyFlags::WND_PROP_FULLSCREEN, cv::WindowFlags::WINDOW_FULLSCREEN);
  cv::startWindowThread();

  data_sub = nh.subscribe("/navi/display",1000,GetDataCallback);
  left_eye_sub = nh.subscribe("/navi/eye/left",1000,GetLeftEyeCallback);
  right_eye_sub = nh.subscribe("/navi/eye/right",1000,GetRightEyeCallback);

  ros::spin();
  cv::destroyWindow("face");
}

