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

  //CheckImage(moving_eye_face_path_1);
  normal_face = cv::imread(normal_face_path, cv::IMREAD_COLOR);
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

  ros::spin();
  cv::destroyWindow("face");
}

