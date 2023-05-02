#define USE_USBCON
////////////////////////////////////////////////////////////////////////
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN        6
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 100

//////////////////////////////////////////////////////////////////////

#include <ros.h>
#include <navi_proto_humanoid_msgs/Humanoid.h>
#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

ros::NodeHandle  nh;

//////////////////////////////////////////////////////////////

const int LASER_1=7;
const int LED_LEFT=8;
const int LED_RIGHT=9;

//////////////////////////////////////////////////////////////

float ROBOT_WIDTH = 0.185; //mm   (18.5 cm)
float WHEEL_WIDTH = 0.065; //mm   (18.5 cm)

/////////////////////////////////////////////////////////////

const uint8_t DXL_ID_10 = 10;
const uint8_t DXL_ID_9 = 9;
const uint8_t DXL_ID_8 = 8;
const uint8_t DXL_ID_7 = 7;
const uint8_t DXL_ID_6 = 6;
const uint8_t DXL_ID_5 = 5;
const uint8_t DXL_ID_4 = 4;
const uint8_t DXL_ID_3 = 3;
const uint8_t DXL_ID_2 = 2;
const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_0 = 0;

////////////////////////////////////////////////////////////

int DXL_POSITION_9= 2040;
int DXL_POSITION_8= 2040;
int DXL_POSITION_7= 1246;
int DXL_POSITION_6= 1246;
int DXL_POSITION_5= 2040;
int DXL_POSITION_4= 2040;
int DXL_POSITION_3= 2040;
int DXL_POSITION_2= 2040;

int TANK=0;
bool tank_fire=0;
bool fire_1=0;
bool fire_2=0;

int DXL_VELOCITY_1= 0;
int DXL_VELOCITY_0= 0;

int time_count=0;
int pixel=0;
int color_num=0;
bool color_on[3]={1,1,1};

////////////////////////////////////////////////////////////
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUD_RATE = 1000000;
const uint8_t DXL_CONTROL_SPEED = 0;
////////////////////////////////////////////////////////////

Dynamixel2Arduino dxl_10(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_9(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_8(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_7(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_6(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_5(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_4(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_3(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_2(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_1(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_0(DXL_SERIAL, DXL_DIR_PIN);

////////////////////////////////////////////////////////////
//This namespace is required to use Control table item names
using namespace ControlTableItem;


void navi_control_callback( const navi_proto_humanoid_msgs::Humanoid& msg){
  DXL_POSITION_8=msg.servo_head[0];
  DXL_POSITION_9=msg.servo_head[1];

  DXL_POSITION_3=msg.servo_left[0];
  DXL_POSITION_5=msg.servo_left[1];
  DXL_POSITION_7=msg.servo_left[2];

  DXL_POSITION_2=msg.servo_right[0];
  DXL_POSITION_4=msg.servo_right[1];
  DXL_POSITION_6=msg.servo_right[2];

  TANK=msg.extra[0];
  tank_fire=msg.extra[1];
  fire_1=msg.extra[2];
  fire_2=msg.extra[3];
  

  //float linear=msg.linear;
  //float angular=msg.angular;

  //DXL_VELOCITY_1=(int)((linear-angular)*(ROBOT_WIDTH/2)/(PI*WHEEL_WIDTH));
  //DXL_VELOCITY_0=(int)((linear+angular)*(ROBOT_WIDTH/2)/(PI*WHEEL_WIDTH));

  DXL_VELOCITY_0=msg.linear;
  DXL_VELOCITY_1=msg.angular;


  dxl_9.setGoalPosition(DXL_ID_9, DXL_POSITION_9);
  dxl_8.setGoalPosition(DXL_ID_8, DXL_POSITION_8);

  dxl_3.setGoalPosition(DXL_ID_3, DXL_POSITION_3);
  dxl_5.setGoalPosition(DXL_ID_5, DXL_POSITION_5);
  dxl_7.setGoalPosition(DXL_ID_7, DXL_POSITION_7);

  dxl_2.setGoalPosition(DXL_ID_2, DXL_POSITION_2);
  dxl_4.setGoalPosition(DXL_ID_4, DXL_POSITION_4);
  dxl_6.setGoalPosition(DXL_ID_6, DXL_POSITION_6);

  dxl_1.setGoalVelocity(DXL_ID_1, DXL_VELOCITY_1);
  dxl_0.setGoalVelocity(DXL_ID_0, DXL_VELOCITY_0);

  dxl_10.setGoalPosition(DXL_ID_10, TANK);

  digitalWrite(LASER_1, tank_fire);
  digitalWrite(LED_LEFT, fire_1);
  digitalWrite(LED_RIGHT, fire_2);

  time_count++;
}


ros::Subscriber<navi_proto_humanoid_msgs::Humanoid> sub("/navi/openrb", navi_control_callback);

void setup(){
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(LASER_1,OUTPUT);
  pinMode(LED_LEFT,OUTPUT);
  pinMode(LED_RIGHT,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  DEBUG_SERIAL.begin(115200);
  DXL_setup();

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
 
  pixels.begin();
}

void loop(){
  nh.spinOnce();
  delay(1);
  if(time_count>5){
    pixels.setPixelColor(pixel, pixels.Color(30*pixel*color_on[0],30*pixel*color_on[1],30*pixel*color_on[2]));
    pixels.show();
    time_count=0;
    if(pixel==7){
      pixel=0;
      color_num++;
      pixels.clear();
    }
    else{
      pixel++;
    }
    switch(color_num){
    case 0: color_on[0]=1; color_on[1]=1; color_on[2]=1;  break;
    case 1: color_on[0]=0; color_on[1]=1; color_on[2]=1;  break;
    case 2: color_on[0]=1; color_on[1]=0; color_on[2]=1;  break;
    case 3: color_on[0]=1; color_on[1]=1; color_on[2]=0;  break;
    case 4: color_on[0]=1; color_on[1]=0; color_on[2]=0;  break;
    case 5: color_on[0]=0; color_on[1]=0; color_on[2]=1;  break;
    case 6: color_on[0]=0; color_on[1]=1; color_on[2]=0;  break;
    case 7: color_num=0;  break;
  }
  }
}

//////////////////////////////////////////////////////////////////////////

void DXL_setup(){
  dxl_10.begin(DXL_BAUD_RATE);
  dxl_9.begin(DXL_BAUD_RATE);
  dxl_8.begin(DXL_BAUD_RATE);
  dxl_7.begin(DXL_BAUD_RATE);
  dxl_6.begin(DXL_BAUD_RATE);
  dxl_5.begin(DXL_BAUD_RATE);
  dxl_4.begin(DXL_BAUD_RATE);
  dxl_3.begin(DXL_BAUD_RATE);
  dxl_2.begin(DXL_BAUD_RATE);
  dxl_1.begin(DXL_BAUD_RATE);
  dxl_0.begin(DXL_BAUD_RATE);

  dxl_10.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_9.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_8.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_7.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_6.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_5.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_4.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_3.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_0.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  

  dxl_10.ping(DXL_ID_10);
  dxl_9.ping(DXL_ID_9);
  dxl_8.ping(DXL_ID_8);
  dxl_7.ping(DXL_ID_7);
  dxl_6.ping(DXL_ID_6);
  dxl_5.ping(DXL_ID_5);
  dxl_4.ping(DXL_ID_4);
  dxl_3.ping(DXL_ID_3);
  dxl_2.ping(DXL_ID_2);
  dxl_1.ping(DXL_ID_1);
  dxl_0.ping(DXL_ID_0);

  dxl_10.torqueOff(DXL_ID_10);
  dxl_9.torqueOff(DXL_ID_9);
  dxl_8.torqueOff(DXL_ID_8);
  dxl_7.torqueOff(DXL_ID_7);
  dxl_6.torqueOff(DXL_ID_6);
  dxl_5.torqueOff(DXL_ID_5);
  dxl_4.torqueOff(DXL_ID_4);
  dxl_3.torqueOff(DXL_ID_3);
  dxl_2.torqueOff(DXL_ID_2);
  dxl_1.torqueOff(DXL_ID_1);
  dxl_0.torqueOff(DXL_ID_0);

  dxl_10.setOperatingMode(DXL_ID_10, OP_POSITION);
  dxl_9.setOperatingMode(DXL_ID_9, OP_POSITION);
  dxl_8.setOperatingMode(DXL_ID_8, OP_POSITION);
  dxl_7.setOperatingMode(DXL_ID_7, OP_POSITION);
  dxl_6.setOperatingMode(DXL_ID_6, OP_POSITION);
  dxl_5.setOperatingMode(DXL_ID_5, OP_POSITION);
  dxl_4.setOperatingMode(DXL_ID_4, OP_POSITION);
  dxl_3.setOperatingMode(DXL_ID_3, OP_POSITION);
  dxl_2.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl_1.setOperatingMode(DXL_ID_1, OP_VELOCITY);
  dxl_0.setOperatingMode(DXL_ID_0, OP_VELOCITY);
  delay(100);
  dxl_10.torqueOn(DXL_ID_10);
  delay(100);
  dxl_9.torqueOn(DXL_ID_9);
  delay(100);
  dxl_8.torqueOn(DXL_ID_8);
  delay(100);
  dxl_7.torqueOn(DXL_ID_7);
  delay(100);
  dxl_6.torqueOn(DXL_ID_6);
  delay(100);
  dxl_5.torqueOn(DXL_ID_5);
  delay(100);
  dxl_4.torqueOn(DXL_ID_4);
  delay(100);
  dxl_3.torqueOn(DXL_ID_3);
  delay(100);
  dxl_2.torqueOn(DXL_ID_2);
  delay(100);
  dxl_1.torqueOn(DXL_ID_1);
  delay(100);
  dxl_0.torqueOn(DXL_ID_0);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);

  dxl_10.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_10, DXL_CONTROL_SPEED);
  dxl_9.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_9, DXL_CONTROL_SPEED);
  dxl_8.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_8, DXL_CONTROL_SPEED);
  dxl_7.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_7, DXL_CONTROL_SPEED);
  dxl_6.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_6, DXL_CONTROL_SPEED);
  dxl_5.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_5, DXL_CONTROL_SPEED);
  dxl_4.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_4, DXL_CONTROL_SPEED);
  dxl_3.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_3, DXL_CONTROL_SPEED);
  dxl_2.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, DXL_CONTROL_SPEED);
 
}
