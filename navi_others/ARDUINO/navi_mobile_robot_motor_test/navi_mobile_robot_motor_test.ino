//#define USE_USBCON

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

#define RPM2PWM_RATIO 2.25   //100(rpm) : 225(pwm)
/////////////////////////////////////////////////////////////
float linear_x = 0;
float linear_y = 0;
float angular_z = 0;

float wheel_rpm[4] = {0, 0, 0, 0};
/////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(5600);
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
  Motor_Controller(0, true, 0);
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  Motor_Controller(3, true, 0);
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
