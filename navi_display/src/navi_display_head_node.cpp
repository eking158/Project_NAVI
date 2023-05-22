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
        ROS_INFO("%d", *now_time);
        num_gif_char=to_string(num_gif);
        cv::Mat gif = cv::imread(path+num_gif_char+".png", cv::IMREAD_COLOR);
        cv::imshow("view", gif);
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

  file_path_1 = "/home/eking/NAVI_ws/src/navi_main/navi_display/src/images/background2.jpg";
  file_path_2 = "/home/eking/NAVI_ws/src/navi_main/navi_display/src/images/test.png";
  gif_file_path_1 = "/home/eking/NAVI_ws/src/navi_main/navi_display/src/images/gif_test_1/";
  gif_file_path_2 = "/home/eking/NAVI_ws/src/navi_main/navi_display/src/images/gif_test_2/";

  //CheckImage(gif_file_path_2);

  image_1 = cv::imread(file_path_1, cv::IMREAD_COLOR);
  image_2 = cv::imread(file_path_2, cv::IMREAD_COLOR);
}
///////////////////////////////////////////////////////////////////////////////////////////
void GetDataCallback(const std_msgs::Int16& msg){
    display_head = msg.data;
    time_stack++;
    switch(msg.data){
      case 0:
      cv::imshow("view", image_1);
      cv::waitKey(10);
      break;

      case 1:
      cv::imshow("view", image_2);
      cv::waitKey(10);
      break;

      case 2:
      GIFShow(gif_file_path_1, &time_stack, 12, 0.4);
      break;

      case 3:
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

  cv::namedWindow("view", cv::WindowFlags::WINDOW_NORMAL);
  cv::setWindowProperty("view", cv::WindowPropertyFlags::WND_PROP_FULLSCREEN, cv::WindowFlags::WINDOW_FULLSCREEN);
  cv::startWindowThread();

  data_sub = nh.subscribe("/navi/display",1000,GetDataCallback);

  ros::spin();
  cv::destroyWindow("view");
}

