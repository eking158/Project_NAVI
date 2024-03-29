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
//-----------------------------------------------------------------------------------------
void MovingEyeMinions(string path, int* now_time, float frame_time_blink, float frame_time_rest){
  float frame_time = 0;
  if(num_gif == 1) frame_time = frame_time_blink;
  else if(num_gif == 2) frame_time = frame_time_blink;
  else if(num_gif == 3) frame_time = frame_time_rest;
  if(*now_time > frame_time*PUB_HZ){
    if(num_gif>=3) num_gif=1;
    else num_gif++;
    //ROS_INFO("%d", num_gif);
  *now_time=0;
  }
  else{
    num_gif_char=to_string(num_gif);
    cv::Mat gif = cv::imread(path+num_gif_char+".png", cv::IMREAD_COLOR);
    cv::Mat gif_rotate;
    if(num_gif == 1 || num_gif == 3){
      cv::rotate(gif, gif_rotate, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::ellipse(gif_rotate, cv::Point(minions_eye_base_1.y+left_eye_msgs.y, minions_eye_base_1.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_1, minions_eye_minor_length_1), minions_eye_rotate_angle_1, 0, 360, cv::Scalar(minions_eye_b_1, minions_eye_g_1, minions_eye_r_1), minions_eye_thick_1, 8);
      cv::ellipse(gif_rotate, cv::Point(minions_eye_base_2.y+left_eye_msgs.y, minions_eye_base_2.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_2, minions_eye_minor_length_2), minions_eye_rotate_angle_2, 0, 360, cv::Scalar(minions_eye_b_2, minions_eye_g_2, minions_eye_r_2), minions_eye_thick_2, 8);
      cv::ellipse(gif_rotate, cv::Point(minions_eye_base_3.y+left_eye_msgs.y, minions_eye_base_3.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_3, minions_eye_minor_length_3), minions_eye_rotate_angle_3, 0, 360, cv::Scalar(minions_eye_b_3, minions_eye_g_3, minions_eye_r_3), minions_eye_thick_3, 8);
      cv::ellipse(gif_rotate, cv::Point(minions_eye_base_4.y+left_eye_msgs.y, minions_eye_base_4.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_4, minions_eye_minor_length_4), minions_eye_rotate_angle_4, 0, 360, cv::Scalar(minions_eye_b_4, minions_eye_g_4, minions_eye_r_4), minions_eye_thick_4, 8);
      cv::imshow("face", gif_rotate);
    }
    else if(num_gif == 2){
      cv::rotate(gif, gif_rotate, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::imshow("face", gif_rotate);
     }
    cv::waitKey(10);
  }
}
//-----------------------------------------------------------------------------------------
void MovingEyeNormal(string path, int* now_time, float frame_time_blink, float frame_time_rest){
  float frame_time = 0;
  if(num_gif == 1) frame_time = frame_time_blink;
  else if(num_gif == 2) frame_time = frame_time_blink;
  else if(num_gif == 3) frame_time = frame_time_rest;
  if(*now_time > frame_time*PUB_HZ){
    if(num_gif>=3) num_gif=1;
    else num_gif++;
    //ROS_INFO("%d", num_gif);
   *now_time=0;
  }
  else{
    cv::Mat gif = cv::imread(path, cv::IMREAD_COLOR);
    cv::Mat gif_rotate;
    if(num_gif == 1 || num_gif == 3){
      cv::rotate(gif, gif_rotate, cv::ROTATE_90_COUNTERCLOCKWISE);
      for(int i =0; i<18; i++){
        if(i>0){
          cv::ellipse(gif_rotate, cv::Point(left_eye_base.y+left_eye_msgs.y, left_eye_base.x+left_eye_msgs.x), cv::Size(left_eye_major_length+ (18 - i)*eye_shine_ratio, left_eye_minor_length+ (18 - i)*eye_shine_ratio), left_eye_rotate_angle, 0, 360, cv::Scalar(i*10, left_eye_g, left_eye_r), left_eye_thick, 8);
          cv::ellipse(gif_rotate, cv::Point(right_eye_base.y+left_eye_msgs.y, right_eye_base.x+left_eye_msgs.x), cv::Size(right_eye_major_length+ (18 - i)*eye_shine_ratio, right_eye_minor_length+ (18 - i)*eye_shine_ratio), right_eye_rotate_angle, 0, 360, cv::Scalar(i*10, right_eye_g, right_eye_r), right_eye_thick, 8);
        }
      }
      cv::imshow("face", gif_rotate);
     }
     else if(num_gif == 2){
      cv::rotate(gif, gif_rotate, cv::ROTATE_90_COUNTERCLOCKWISE);
      for(int i =0; i<18; i++){
        if(i>0){
          cv::ellipse(gif_rotate, cv::Point(left_eye_base.y+left_eye_msgs.y, left_eye_base.x+left_eye_msgs.x), cv::Size(0 ,0), left_eye_rotate_angle, 0, 360, cv::Scalar(i*10, left_eye_g, left_eye_r), left_eye_thick, 8);
          cv::ellipse(gif_rotate, cv::Point(right_eye_base.y+left_eye_msgs.y, right_eye_base.x+left_eye_msgs.x), cv::Size(0, 0), right_eye_rotate_angle, 0, 360, cv::Scalar(i*10, right_eye_g, right_eye_r), right_eye_thick, 8);
        }
      }
      cv::imshow("face", gif_rotate);
     }
    cv::waitKey(10);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetEyeParamYaml(){
  std::string GIF_path = ros::package::getPath("navi_display") + "/config/gif_data.yaml"; //AB param yaml
  YAML::Node GIF_doc = YAML::LoadFile(GIF_path);
  gif_frame_num_1           = GIF_doc["gif_frame_num_1"].as<int>();
  gif_frame_num_2           = GIF_doc["gif_frame_num_2"].as<int>();
  gif_frame_num_3           = GIF_doc["gif_frame_num_3"].as<int>();
  gif_frame_num_4           = GIF_doc["gif_frame_num_4"].as<int>();
  gif_frame_num_5           = GIF_doc["gif_frame_num_5"].as<int>();
  gif_frame_num_6           = GIF_doc["gif_frame_num_6"].as<int>();
  gif_frame_num_7           = GIF_doc["gif_frame_num_7"].as<int>();

  gif_frame_time_1           = GIF_doc["gif_frame_time_1"].as<float>();
  gif_frame_time_2           = GIF_doc["gif_frame_time_2"].as<float>();
  gif_frame_time_3           = GIF_doc["gif_frame_time_3"].as<float>();
  gif_frame_time_4           = GIF_doc["gif_frame_time_4"].as<float>();
  gif_frame_time_5           = GIF_doc["gif_frame_time_5"].as<float>();
  gif_frame_time_6           = GIF_doc["gif_frame_time_6"].as<float>();
  gif_frame_time_7           = GIF_doc["gif_frame_time_7"].as<float>();
  //-----------------------------------------------------------------------------------------
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

  eye_tracking_x_min         = Eye_doc["eye_tracking_x_min"].as<int>();
  eye_tracking_x_max         = Eye_doc["eye_tracking_x_max"].as<int>();
  eye_tracking_y_min         = Eye_doc["eye_tracking_y_min"].as<int>();
  eye_tracking_y_max         = Eye_doc["eye_tracking_y_max"].as<int>();

  eye_moving_x_min            = Eye_doc["eye_moving_x_min"].as<int>();
  eye_moving_x_max            = Eye_doc["eye_moving_x_max"].as<int>();
  eye_moving_y_min            = Eye_doc["eye_moving_y_min"].as<int>();
  eye_moving_y_max            = Eye_doc["eye_moving_y_max"].as<int>();

  eye_blink_time              = Eye_doc["eye_blink_time"].as<float>();
  eye_open_time               = Eye_doc["eye_open_time"].as<float>();

  eye_blink_major_ratio              = Eye_doc["eye_blink_major_ratio"].as<float>();
  eye_blink_minor_ratio               = Eye_doc["eye_blink_minor_ratio"].as<float>();

  eye_shine_ratio               = Eye_doc["eye_shine_ratio"].as<float>();
  //-----------------------------------------------------------------------------------------
  std::string Eye_path_2 = ros::package::getPath("navi_display") + "/config/eye_data_minions.yaml"; //AB param yaml
  YAML::Node Eye_doc_2 = YAML::LoadFile(Eye_path_2);
  minions_eye_base_1.x           = Eye_doc_2["minions_eye_base_1_x"].as<int>();
  minions_eye_base_1.y           = Eye_doc_2["minions_eye_base_1_y"].as<int>();
  minions_eye_major_length_1     = Eye_doc_2["minions_eye_major_length_1"].as<int>();
  minions_eye_minor_length_1     = Eye_doc_2["minions_eye_minor_length_1"].as<int>();
  minions_eye_rotate_angle_1     = Eye_doc_2["minions_eye_rotate_angle_1"].as<int>();
  minions_eye_r_1                = Eye_doc_2["minions_eye_r_1"].as<int>();
  minions_eye_g_1                = Eye_doc_2["minions_eye_g_1"].as<int>();
  minions_eye_b_1                = Eye_doc_2["minions_eye_b_1"].as<int>();
  minions_eye_thick_1            = Eye_doc_2["minions_eye_thick_1"].as<int>();

  minions_eye_base_2.x           = Eye_doc_2["minions_eye_base_2_x"].as<int>();
  minions_eye_base_2.y           = Eye_doc_2["minions_eye_base_2_y"].as<int>();
  minions_eye_major_length_2     = Eye_doc_2["minions_eye_major_length_2"].as<int>();
  minions_eye_minor_length_2     = Eye_doc_2["minions_eye_minor_length_2"].as<int>();
  minions_eye_rotate_angle_2     = Eye_doc_2["minions_eye_rotate_angle_2"].as<int>();
  minions_eye_r_2                = Eye_doc_2["minions_eye_r_2"].as<int>();
  minions_eye_g_2                = Eye_doc_2["minions_eye_g_2"].as<int>();
  minions_eye_b_2                = Eye_doc_2["minions_eye_b_2"].as<int>();
  minions_eye_thick_2            = Eye_doc_2["minions_eye_thick_2"].as<int>();

  minions_eye_base_3.x           = Eye_doc_2["minions_eye_base_3_x"].as<int>();
  minions_eye_base_3.y           = Eye_doc_2["minions_eye_base_3_y"].as<int>();
  minions_eye_major_length_3     = Eye_doc_2["minions_eye_major_length_3"].as<int>();
  minions_eye_minor_length_3     = Eye_doc_2["minions_eye_minor_length_3"].as<int>();
  minions_eye_rotate_angle_3     = Eye_doc_2["minions_eye_rotate_angle_3"].as<int>();
  minions_eye_r_3                = Eye_doc_2["minions_eye_r_3"].as<int>();
  minions_eye_g_3                = Eye_doc_2["minions_eye_g_3"].as<int>();
  minions_eye_b_3                = Eye_doc_2["minions_eye_b_3"].as<int>();
  minions_eye_thick_3            = Eye_doc_2["minions_eye_thick_3"].as<int>();

  minions_eye_base_4.x           = Eye_doc_2["minions_eye_base_4_x"].as<int>();
  minions_eye_base_4.y           = Eye_doc_2["minions_eye_base_4_y"].as<int>();
  minions_eye_major_length_4     = Eye_doc_2["minions_eye_major_length_4"].as<int>();
  minions_eye_minor_length_4     = Eye_doc_2["minions_eye_minor_length_4"].as<int>();
  minions_eye_rotate_angle_4     = Eye_doc_2["minions_eye_rotate_angle_4"].as<int>();
  minions_eye_r_4                = Eye_doc_2["minions_eye_r_4"].as<int>();
  minions_eye_g_4                = Eye_doc_2["minions_eye_g_4"].as<int>();
  minions_eye_b_4                = Eye_doc_2["minions_eye_b_4"].as<int>();
  minions_eye_thick_4            = Eye_doc_2["minions_eye_thick_4"].as<int>();

  minions_eye_moving_x_min       = Eye_doc_2["minions_eye_moving_x_min"].as<int>();
  minions_eye_moving_x_max       = Eye_doc_2["minions_eye_moving_x_max"].as<int>();
  minions_eye_moving_y_min       = Eye_doc_2["minions_eye_moving_y_min"].as<int>();
  minions_eye_moving_y_max       = Eye_doc_2["minions_eye_moving_y_max"].as<int>();

  minions_eye_blink_time              = Eye_doc_2["minions_eye_blink_time"].as<float>();
  minions_eye_open_time               = Eye_doc_2["minions_eye_open_time"].as<float>();
  //-----------------------------------------------------------------------------------------
  std::string Eye_path_3 = ros::package::getPath("navi_display") + "/config/eye_data_thomas.yaml"; //AB param yaml
  YAML::Node Eye_doc_3 = YAML::LoadFile(Eye_path_3);
  thomas_left_eye_base.x           = Eye_doc_3["thomas_left_eye_base_x"].as<int>();
  thomas_left_eye_base.y           = Eye_doc_3["thomas_left_eye_base_y"].as<int>();
  thomas_left_eye_major_length     = Eye_doc_3["thomas_left_eye_major_length"].as<int>();
  thomas_left_eye_minor_length     = Eye_doc_3["thomas_left_eye_minor_length"].as<int>();
  thomas_left_eye_rotate_angle     = Eye_doc_3["thomas_left_eye_rotate_angle"].as<int>();
  thomas_left_eye_r                = Eye_doc_3["thomas_left_eye_r"].as<int>();
  thomas_left_eye_g                = Eye_doc_3["thomas_left_eye_g"].as<int>();
  thomas_left_eye_b                = Eye_doc_3["thomas_left_eye_b"].as<int>();
  thomas_left_eye_thick            = Eye_doc_3["thomas_left_eye_thick"].as<int>();

  thomas_right_eye_base.x           = Eye_doc_3["thomas_right_eye_base_x"].as<int>();
  thomas_right_eye_base.y           = Eye_doc_3["thomas_right_eye_base_y"].as<int>();
  thomas_right_eye_major_length     = Eye_doc_3["thomas_right_eye_major_length"].as<int>();
  thomas_right_eye_minor_length     = Eye_doc_3["thomas_right_eye_minor_length"].as<int>();
  thomas_right_eye_rotate_angle     = Eye_doc_3["thomas_right_eye_rotate_angle"].as<int>();
  thomas_right_eye_r                = Eye_doc_3["thomas_right_eye_r"].as<int>();
  thomas_right_eye_g                = Eye_doc_3["thomas_right_eye_g"].as<int>();
  thomas_right_eye_b                = Eye_doc_3["thomas_right_eye_b"].as<int>();
  thomas_right_eye_thick            = Eye_doc_3["thomas_right_eye_thick"].as<int>();

  thomas_eye_tracking_x_min         = Eye_doc_3["thomas_eye_tracking_x_min"].as<int>();
  thomas_eye_tracking_x_max         = Eye_doc_3["thomas_eye_tracking_x_max"].as<int>();
  thomas_eye_tracking_y_min         = Eye_doc_3["thomas_eye_tracking_y_min"].as<int>();
  thomas_eye_tracking_y_max         = Eye_doc_3["thomas_eye_tracking_y_max"].as<int>();

  thomas_eye_moving_x_min            = Eye_doc_3["thomas_eye_moving_x_min"].as<int>();
  thomas_eye_moving_x_max            = Eye_doc_3["thomas_eye_moving_x_max"].as<int>();
  thomas_eye_moving_y_min            = Eye_doc_3["thomas_eye_moving_y_min"].as<int>();
  thomas_eye_moving_y_max            = Eye_doc_3["thomas_eye_moving_y_max"].as<int>();

  thomas_eye_blink_time              = Eye_doc_3["thomas_eye_blink_time"].as<float>();
  thomas_eye_open_time               = Eye_doc_3["thomas_eye_open_time"].as<float>();
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
  moving_eye_face_path_1 = file_path_origin+"images/moving_eye_face_1/1.png";
  moving_eye_face_path_2 = file_path_origin+"images/moving_eye_face_2/";
  moving_eye_face_path_3 = file_path_origin+"images/moving_eye_face_3/1.png";

  //CheckImage(moving_eye_face_path_1);
  normal_face = cv::imread(normal_face_path, cv::IMREAD_COLOR);
  moving_eye_face_1 = cv::imread(moving_eye_face_path_1, cv::IMREAD_COLOR);
  moving_eye_face_2 = cv::imread(moving_eye_face_path_2, cv::IMREAD_COLOR);
  moving_eye_face_3 = cv::imread(moving_eye_face_path_3, cv::IMREAD_COLOR);
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
      GIFShow(face_path_1, &time_stack, gif_frame_num_1, gif_frame_time_1);
      break;

      case 1:
      GIFShow(face_path_2, &time_stack, gif_frame_num_2, gif_frame_time_2);
      break;

      case 2:
      GIFShow(face_path_3, &time_stack, gif_frame_num_3, gif_frame_time_3);
      break;

      case 3:
      GIFShow(face_path_4, &time_stack, gif_frame_num_4, gif_frame_time_4);
      break;

      case 4:
      GIFShow(face_path_5, &time_stack, gif_frame_num_5, gif_frame_time_5);
      break;

      case 5:
      GIFShow(face_path_6, &time_stack, gif_frame_num_6, gif_frame_time_6);
      break;

      case 6:
      GIFShow(face_path_7, &time_stack, gif_frame_num_7, gif_frame_time_7);
      break;

      case 7:
      MovingEyeNormal(moving_eye_face_path_1, &time_stack, eye_blink_time, eye_open_time);
      //cv::rotate(moving_eye_face_1, moving_eye_face_rotate_1, cv::ROTATE_90_COUNTERCLOCKWISE);
      //for(int i =0; i<18; i++){
      //  if(i>0){
      //    cv::ellipse(moving_eye_face_rotate_1, cv::Point(left_eye_base.y+left_eye_msgs.y, left_eye_base.x+left_eye_msgs.x), cv::Size(left_eye_major_length+ (18 - i)*eye_shine_ratio, left_eye_minor_length+ (18 - i)*eye_shine_ratio), left_eye_rotate_angle, 0, 360, cv::Scalar(i*10, left_eye_g, left_eye_r), left_eye_thick, 8);
      //    cv::ellipse(moving_eye_face_rotate_1, cv::Point(right_eye_base.y+left_eye_msgs.y, right_eye_base.x+left_eye_msgs.x), cv::Size(right_eye_major_length+ (18 - i)*eye_shine_ratio, right_eye_minor_length+ (18 - i)*eye_shine_ratio), right_eye_rotate_angle, 0, 360, cv::Scalar(i*10, right_eye_g, right_eye_r), right_eye_thick, 8);
      //  }
      //}
      //cv::imshow("face", moving_eye_face_rotate_1);
      //cv::waitKey(10);
      break;

      case 8:
      MovingEyeMinions(moving_eye_face_path_2, &time_stack, minions_eye_blink_time, minions_eye_open_time);
      //cv::rotate(moving_eye_face_2, moving_eye_face_rotate_2, cv::ROTATE_90_COUNTERCLOCKWISE);
      //cv::ellipse(moving_eye_face_rotate_2, cv::Point(minions_eye_base_1.y+left_eye_msgs.y, minions_eye_base_1.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_1, minions_eye_minor_length_1), minions_eye_rotate_angle_1, 0, 360, cv::Scalar(minions_eye_b_1, minions_eye_g_1, minions_eye_r_1), minions_eye_thick_1, 8);
      //cv::ellipse(moving_eye_face_rotate_2, cv::Point(minions_eye_base_2.y+left_eye_msgs.y, minions_eye_base_2.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_2, minions_eye_minor_length_2), minions_eye_rotate_angle_2, 0, 360, cv::Scalar(minions_eye_b_2, minions_eye_g_2, minions_eye_r_2), minions_eye_thick_2, 8);
      //cv::ellipse(moving_eye_face_rotate_2, cv::Point(minions_eye_base_3.y+left_eye_msgs.y, minions_eye_base_3.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_3, minions_eye_minor_length_3), minions_eye_rotate_angle_3, 0, 360, cv::Scalar(minions_eye_b_3, minions_eye_g_3, minions_eye_r_3), minions_eye_thick_3, 8);
      //cv::ellipse(moving_eye_face_rotate_2, cv::Point(minions_eye_base_4.y+left_eye_msgs.y, minions_eye_base_4.x+left_eye_msgs.x), cv::Size(minions_eye_major_length_4, minions_eye_minor_length_4), minions_eye_rotate_angle_4, 0, 360, cv::Scalar(minions_eye_b_4, minions_eye_g_4, minions_eye_r_4), minions_eye_thick_4, 8);
      //cv::imshow("face", moving_eye_face_rotate_2);
      //cv::waitKey(10);
      break;

      case 9:
      cv::rotate(moving_eye_face_3, moving_eye_face_rotate_3, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::ellipse(moving_eye_face_rotate_3, cv::Point(thomas_left_eye_base.y+left_eye_msgs.y, thomas_left_eye_base.x+left_eye_msgs.x), cv::Size(thomas_left_eye_major_length, thomas_left_eye_minor_length), thomas_left_eye_rotate_angle, 0, 360, cv::Scalar(thomas_left_eye_b, thomas_left_eye_g, thomas_left_eye_r), thomas_left_eye_thick, 8);
      cv::ellipse(moving_eye_face_rotate_3, cv::Point(thomas_right_eye_base.y+left_eye_msgs.y, thomas_right_eye_base.x+left_eye_msgs.x), cv::Size(thomas_right_eye_major_length, thomas_right_eye_minor_length), thomas_right_eye_rotate_angle, 0, 360, cv::Scalar(thomas_right_eye_b, thomas_right_eye_g, thomas_right_eye_r), thomas_right_eye_thick, 8);
      cv::imshow("face", moving_eye_face_rotate_3);
      cv::waitKey(10);
      break;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetLeftEyeCallback(const geometry_msgs::Pose2D& msg){
  switch(display_head){
    case 7:
    left_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, eye_moving_x_min, eye_moving_x_max);
    left_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, eye_moving_y_min, eye_moving_y_max);
    break;
    case 8:
    left_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, minions_eye_moving_x_min, minions_eye_moving_x_max);
    left_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, minions_eye_moving_y_min, minions_eye_moving_y_max);
    break;
    case 9:
    left_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, thomas_eye_moving_x_min, thomas_eye_moving_x_max);
    left_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, thomas_eye_moving_y_min, thomas_eye_moving_y_max);
    break;
  }
}
//-----------------------------------------------------------------------------------------
void GetRightEyeCallback(const geometry_msgs::Pose2D& msg){
  switch(display_head){
    case 7:
    right_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, eye_moving_x_min, eye_moving_x_max);
    right_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, eye_moving_y_min, eye_moving_y_max);
    break;
    case 8:
    right_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, minions_eye_moving_x_min, minions_eye_moving_x_max);
    right_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, minions_eye_moving_y_min, minions_eye_moving_y_max);
    break;
    case 9:
    right_eye_msgs.x = -float_map(msg.x, eye_tracking_x_min, eye_tracking_x_max, thomas_eye_moving_x_min, thomas_eye_moving_x_max);
    right_eye_msgs.y = -float_map(msg.y, eye_tracking_y_min, eye_tracking_y_max, thomas_eye_moving_y_min, thomas_eye_moving_y_max);
    break;
  }
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

