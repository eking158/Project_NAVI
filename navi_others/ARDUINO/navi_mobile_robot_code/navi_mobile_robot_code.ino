//#define USE_USBCON
#define BAUD 57600

#define MOTOR_FL_DIR 7
#define MOTOR_FL_PWM 6
#define MOTOR_FR_DIR 5
#define MOTOR_FR_PWM 4

#define MOTOR_BL_DIR 11
#define MOTOR_BL_PWM 10
#define MOTOR_BR_DIR 9
#define MOTOR_BR_PWM 8

#define WHEEL_RADIUS 0.075    //m  (7.5 cm) -> wheel radius
#define ROBOT_RADIUS_X 0.22  //m  (22 cm) -> robot middle to end(x axis)
#define ROBOT_RADIUS_Y 0.30  //m  (30 cm) -> robot middle to end(y axis)

#define RPM2PWM_RATIO 180/2.6   //2.6(rpm) : 180(pwm)
/////////////////////////////////////////////////////////////
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
/////////////////////////////////////////////////////////////
float linear_x = 0;
float linear_y = 0;
float angular_z = 0;

float wheel_rpm[4] = {0, 0, 0, 0};
/////////////////////////////////////////////////////////////

void navi_cmd_vel_callback(const geometry_msgs::Twist& msg){
  //get velocity datas
  linear_x = msg.linear.x;
  linear_y = msg.linear.y;
  angular_z = msg.angular.z;

  //front left wheel rpm
  wheel_rpm[0] = (linear_x - linear_y - (ROBOT_RADIUS_X+ROBOT_RADIUS_Y)*angular_z) / WHEEL_RADIUS;
  //front right wheel rpm
  wheel_rpm[1] = (linear_x + linear_y + (ROBOT_RADIUS_X+ROBOT_RADIUS_Y)*angular_z) / WHEEL_RADIUS;
  //back left wheel rpm
  wheel_rpm[2] = (linear_x + linear_y - (ROBOT_RADIUS_X+ROBOT_RADIUS_Y)*angular_z) / WHEEL_RADIUS;
  //back right wheel rpm
  wheel_rpm[3] = (linear_x - linear_y + (ROBOT_RADIUS_X+ROBOT_RADIUS_Y)*angular_z) / WHEEL_RADIUS;

  //control dc motors (change rpm to pwm)
  if(wheel_rpm[0] >= 0) Motor_Controller(0, true, RPM2PWM(wheel_rpm[0], RPM2PWM_RATIO));
  else if(wheel_rpm[0] < 0) Motor_Controller(0, false, RPM2PWM(-wheel_rpm[0], RPM2PWM_RATIO));

  if(wheel_rpm[1] >= 0) Motor_Controller(1, true, RPM2PWM(wheel_rpm[1], RPM2PWM_RATIO));
  else if(wheel_rpm[1] < 0) Motor_Controller(1, false, RPM2PWM(-wheel_rpm[1], RPM2PWM_RATIO));

  if(wheel_rpm[2] >= 0) Motor_Controller(2, true, RPM2PWM(wheel_rpm[2], RPM2PWM_RATIO));
  else if(wheel_rpm[2] < 0) Motor_Controller(2, false, RPM2PWM(-wheel_rpm[2], RPM2PWM_RATIO));

  if(wheel_rpm[3] >= 0) Motor_Controller(3, true, RPM2PWM(wheel_rpm[3], RPM2PWM_RATIO));
  else if(wheel_rpm[3] < 0) Motor_Controller(3, false, RPM2PWM(-wheel_rpm[3], RPM2PWM_RATIO));
}


ros::Subscriber<geometry_msgs::Twist> sub("/navi/cmd_vel", navi_cmd_vel_callback);

void setup(){
  nh.initNode();
  nh.subscribe(sub);

  pinMode(MOTOR_FL_DIR, OUTPUT);
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FR_DIR, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT);
  pinMode(MOTOR_BL_DIR, OUTPUT);
  pinMode(MOTOR_BL_PWM, OUTPUT);
  pinMode(MOTOR_BR_DIR, OUTPUT);
  pinMode(MOTOR_BR_PWM, OUTPUT);
}

void loop(){
  nh.spinOnce();
  delay(1);
}

//////////////////////////////////////////////////////////////////////////
void Motor_Controller(int motor_num, bool motor_direction, int pwm){  //motor control function
  switch(motor_num){
    case 0: //front left
      if(motor_direction == true){
        digitalWrite(MOTOR_FL_DIR, false);
        analogWrite(MOTOR_FL_PWM, pwm);
      }
      else if(motor_direction == false){
        digitalWrite(MOTOR_FL_DIR, true);
        analogWrite(MOTOR_FL_PWM, pwm);
      }
    break;

    case 1: //front right
      if(motor_direction == true){
        digitalWrite(MOTOR_FR_DIR, true);
        analogWrite(MOTOR_FR_PWM, pwm);
      }
      else if(motor_direction == false){
        digitalWrite(MOTOR_FR_DIR, false);
        analogWrite(MOTOR_FR_PWM, pwm);
      }
    break;

    case 2: //back left
      if(motor_direction == true){
        digitalWrite(MOTOR_BL_DIR, false);
        analogWrite(MOTOR_BL_PWM, pwm);
      }
      else if(motor_direction == false){
        digitalWrite(MOTOR_BL_DIR, true);
        analogWrite(MOTOR_BL_PWM, pwm);
      }
    break;

    case 3: //back right
      if(motor_direction == true){
        digitalWrite(MOTOR_BR_DIR, true);
        analogWrite(MOTOR_BR_PWM, pwm);
      }
      else if(motor_direction == false){
        digitalWrite(MOTOR_BR_DIR, false);
        analogWrite(MOTOR_BR_PWM, pwm);
      }
    break;
  }
}
//////////////////////////////////////////////////////////////////////////
float RPM2PWM(float motor_rpm, float ratio){
  return constrain(motor_rpm * ratio, 0, 250);
}
