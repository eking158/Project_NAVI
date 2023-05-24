#include <navi_display/navi_display_head_node.h>

void CheckImage(const char* path){
  int ret = access(path, F_OK);
  if (ret == 0) {
	  ROS_INFO("yes");
  } 
  else {
    ROS_INFO("No");
  }
}

void GIFShow(string path, int* now_time, int frame_num, float frame_time){
  if(*now_time > frame_time*PUB_HZ){
        //ROS_INFO("%d", *now_time);
        num_gif_char=to_string(num_gif);
        cv::Mat gif = cv::imread(path+num_gif_char+".png", cv::IMREAD_COLOR);
        cv::Mat gif_rotate;
        cv::rotate(gif, gif_rotate, cv::ROTATE_90_CLOCKWISE);
        cv::imshow("face", gif_rotate);
        cv::waitKey(10);
        if(num_gif==frame_num) num_gif=1;
        else num_gif++;
        *now_time=0;
      }
}

void initialize(){
  display_head=0;
  time_stack = 0;
  num_gif = 1;

  //file path setting for switching MCU
  //file_path_origin = "/home/ubuntu/catkin_ws/src/Project_NAVI/navi_display/src/";    //for rpi
  file_path_origin = "/home/eking/NAVI_ws/src/navi_main/navi_display/src/";      //for Notebook

  //image file path (for test)
  file_path_1 = file_path_origin+"images/test_1.jpg";
  file_path_2 = file_path_origin+"images/test_2.png";
  gif_file_path_1 = file_path_origin+"images/gif_test_1/";
  gif_file_path_2 = file_path_origin+"images/gif_test_2/";
  //image file path (for face)
  normal_face_path = file_path_origin+"images/normal_face.png";
  face_path_1 = file_path_origin+"images/face_1/";
  face_path_2 = file_path_origin+"images/face_2/";

  //CheckImage(gif_file_path_2);
  normal_face = cv::imread(normal_face_path, cv::IMREAD_COLOR);
  image_1 = cv::imread(file_path_1, cv::IMREAD_COLOR);
  image_2 = cv::imread(file_path_2, cv::IMREAD_COLOR);
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetDataCallback(const std_msgs::Int16& msg){
    display_head = msg.data;
    time_stack++;
    switch(display_head){
      case 0:
      cv::imshow("face", normal_face);
      cv::waitKey(10);
      break;

      case 1:
      GIFShow(face_path_1, &time_stack, 16, 0.1);
      break;

      case 2:
      GIFShow(face_path_2, &time_stack, 17, 0.15);
      break;

      case 10:
      cv::imshow("face", image_1);
      cv::waitKey(10);
      break;

      case 11:
      cv::imshow("face", image_2);
      cv::waitKey(10);
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "navi_display_head_node");
  ros::NodeHandle nh;
  initialize();

  cv::namedWindow("face", cv::WindowFlags::WINDOW_NORMAL);
  cv::setWindowProperty("face", cv::WindowPropertyFlags::WND_PROP_FULLSCREEN, cv::WindowFlags::WINDOW_FULLSCREEN);
  cv::startWindowThread();

  data_sub = nh.subscribe("/navi/display",1000,GetDataCallback);

  ros::spin();
  cv::destroyWindow("face");
}

