// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples dynamicxel_test
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /navi/dynamicxel_set_position_up dynamixel_sdk_examples/SyncSetPosition "{id1: 5, id2: 7, id3: 6, id4: 4, position1: 0, position2: 1000, position3: 1000, position4: 1000}"
 * $ rostopic pub -1 /navi/dynamicxel_set_position_up dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 0, position2: 1000}"
 * $ rostopic pub -1 /sync_set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 1000, position2: 0}"
 * $ rosservice call /sync_get_position "{id1: 1, id2: 2}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

/*
Control NAVI upper Dynamixels

Left_shoulder_pitch -> Left_shoulder_roll --> Left_elbow_pitch
5 -> 7 --> 11

Right_shoulder_pitch -> Right_shoulder_roll  --> Right_elbow_pitch
4 -> 6 --> 10
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h> 
#include <ros/package.h>
#include <string>

#include "std_msgs/String.h"
#include "navi_control_dynamixel/SyncGetPosition.h"
#include "navi_control_dynamixel/SyncSetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_POSITION    116
#define PROFILE_VELOCITY    112
#define PROFILE_ACCEL    108
uint32_t profile_velocity1 = 100;
uint32_t profile_velocity2 = 100;
uint32_t profile_velocity3 = 100;
uint32_t profile_velocity4 = 100;
uint32_t profile_velocity5 = 100;
uint32_t profile_velocity6 = 100;
uint32_t profile_accel1 = 2;
uint32_t profile_accel2 = 2;
uint32_t profile_accel3 = 2;
uint32_t profile_accel4 = 2;
uint32_t profile_accel5 = 2;
uint32_t profile_accel6 = 2;

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               5              // DXL1 ID
#define DXL2_ID               7              // DXL2 ID
#define DXL3_ID               11              // DXL4 ID
#define DXL4_ID               4              // DXL3 ID
#define DXL5_ID               6              // DXL4 ID
#define DXL6_ID               10              // DXL4 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/navi_540"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
GroupSyncWrite groupSyncWrite_velocity(portHandler, packetHandler, PROFILE_VELOCITY, 4);
GroupSyncWrite groupSyncWrite_accel(portHandler, packetHandler, PROFILE_ACCEL, 4);

/*
bool syncGetPresentPositionCallback(
  dynamixel_sdk_examples::SyncGetPosition::Request & req,
  dynamixel_sdk_examples::SyncGetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position1 = 0;
  int32_t position2 = 0;
  int32_t position3 = 0;
  int32_t position4 = 0;
  int32_t position5 = 0;
  int32_t position6 = 0;
  int32_t position7 = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id1);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id1);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id2);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id2);
    return 0;
  }

  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    position1 = groupSyncRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
    position2 = groupSyncRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
    ROS_INFO("getPosition : [POSITION:%d]", position1);
    ROS_INFO("getPosition : [POSITION:%d]", position2);
    res.position1 = position1;
    res.position2 = position2;
    groupSyncRead.clearParam();
    return true;
  } else {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    groupSyncRead.clearParam();
    return false;
  }
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void syncSetPositionCallback(const navi_control_dynamixel::SyncSetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  
  uint8_t param_goal_position1[4];
  uint8_t param_goal_position2[4];
  uint8_t param_goal_position3[4];
  uint8_t param_goal_position4[4];
  uint8_t param_goal_position5[4];
  uint8_t param_goal_position6[4];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position1 = (unsigned int)msg->position1; // Convert int32 -> uint32
  param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(position1));
  param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(position1));
  param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(position1));
  param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(position1));
  uint32_t position2 = (unsigned int)msg->position2; // Convert int32 -> uint32
  param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(position2));
  param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(position2));
  param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(position2));
  param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(position2));
  uint32_t position3 = (unsigned int)msg->position3; // Convert int32 -> uint32
  param_goal_position3[0] = DXL_LOBYTE(DXL_LOWORD(position3));
  param_goal_position3[1] = DXL_HIBYTE(DXL_LOWORD(position3));
  param_goal_position3[2] = DXL_LOBYTE(DXL_HIWORD(position3));
  param_goal_position3[3] = DXL_HIBYTE(DXL_HIWORD(position3));
  uint32_t position4 = (unsigned int)msg->position4; // Convert int32 -> uint32
  param_goal_position4[0] = DXL_LOBYTE(DXL_LOWORD(position4));
  param_goal_position4[1] = DXL_HIBYTE(DXL_LOWORD(position4));
  param_goal_position4[2] = DXL_LOBYTE(DXL_HIWORD(position4));
  param_goal_position4[3] = DXL_HIBYTE(DXL_HIWORD(position4));
  uint32_t position5 = (unsigned int)msg->position5; // Convert int32 -> uint32
  param_goal_position5[0] = DXL_LOBYTE(DXL_LOWORD(position5));
  param_goal_position5[1] = DXL_HIBYTE(DXL_LOWORD(position5));
  param_goal_position5[2] = DXL_LOBYTE(DXL_HIWORD(position5));
  param_goal_position5[3] = DXL_HIBYTE(DXL_HIWORD(position5));
  uint32_t position6 = (unsigned int)msg->position6; // Convert int32 -> uint32
  param_goal_position6[0] = DXL_LOBYTE(DXL_LOWORD(position6));
  param_goal_position6[1] = DXL_HIBYTE(DXL_LOWORD(position6));
  param_goal_position6[2] = DXL_LOBYTE(DXL_HIWORD(position6));
  param_goal_position6[3] = DXL_HIBYTE(DXL_HIWORD(position6));
  

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id1, param_goal_position1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id1);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id2, param_goal_position2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id2);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id3, param_goal_position3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id3);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id4, param_goal_position4);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id4);
  }
  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id5, param_goal_position5);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id5);
  }
  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id6, param_goal_position6);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id6);
  }

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id4, msg->position4);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id5, msg->position5);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id6, msg->position6);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupSyncWrite.clearParam();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "navi_dynamixel_up_node");
  ros::NodeHandle nh;
  //ros::ServiceServer sync_get_position_srv = nh.advertiseService("/sync_get_position", syncGetPresentPositionCallback);
  ros::Subscriber sync_set_position_sub = nh.subscribe("/navi/dynamicxel_set_position_up", 10, syncSetPositionCallback);
  //-------------------------------------------------------------------------------------------------------------------------------------------
  std::string VelAcc_path = ros::package::getPath("navi_control_dynamixel") + "/config/vel_and_acc.yaml"; //AB param yaml
  YAML::Node VelAcc_doc = YAML::LoadFile(VelAcc_path);
  profile_velocity1     = VelAcc_doc["velocity_5"].as<int>();
  profile_velocity2     = VelAcc_doc["velocity_7"].as<int>();
  profile_velocity3     = VelAcc_doc["velocity_11"].as<int>();
  profile_velocity4     = VelAcc_doc["velocity_4"].as<int>();
  profile_velocity5     = VelAcc_doc["velocity_6"].as<int>();
  profile_velocity6     = VelAcc_doc["velocity_10"].as<int>();

  profile_accel1     = VelAcc_doc["accel_5"].as<int>();
  profile_accel2     = VelAcc_doc["accel_7"].as<int>();
  profile_accel3     = VelAcc_doc["accel_11"].as<int>();
  profile_accel4     = VelAcc_doc["accel_4"].as<int>();
  profile_accel5     = VelAcc_doc["accel_6"].as<int>();
  profile_accel6     = VelAcc_doc["accel_10"].as<int>();

  ROS_INFO("[id: 5]profile_velocity: %d   profile_accel: %d", profile_velocity1, profile_accel1);
  ROS_INFO("[id: 7]profile_velocity: %d   profile_accel: %d", profile_velocity2, profile_accel2);
  ROS_INFO("[id: 11]profile_velocity: %d   profile_accel: %d", profile_velocity3, profile_accel3);
  ROS_INFO("[id: 4]profile_velocity: %d   profile_accel: %d", profile_velocity4, profile_accel4);
  ROS_INFO("[id: 6]profile_velocity: %d   profile_accel: %d", profile_velocity5, profile_accel5);
  ROS_INFO("[id: 10]profile_velocity: %d   profile_accel: %d", profile_velocity6, profile_accel6);
  //--------------------------------------------------------------------------------------------------------------------------------
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL4_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL5_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL6_ID);
    return -1;
  }

  int dxl_addparam_result = false;

  uint8_t param_profile_velocity1[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity2[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity3[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity4[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity5[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity6[4]; // Convert int32 -> uint32

  param_profile_velocity1[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity1));
  param_profile_velocity1[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity1));
  param_profile_velocity1[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity1));
  param_profile_velocity1[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity1));

  param_profile_velocity2[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity2));
  param_profile_velocity2[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity2));
  param_profile_velocity2[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity2));
  param_profile_velocity2[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity2));

  param_profile_velocity3[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity3));
  param_profile_velocity3[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity3));
  param_profile_velocity3[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity3));
  param_profile_velocity3[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity3));

  param_profile_velocity4[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity4));
  param_profile_velocity4[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity4));
  param_profile_velocity4[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity4));
  param_profile_velocity4[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity4));

  param_profile_velocity5[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity5));
  param_profile_velocity5[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity5));
  param_profile_velocity5[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity5));
  param_profile_velocity5[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity5));

  param_profile_velocity6[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity6));
  param_profile_velocity6[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity6));
  param_profile_velocity6[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity6));
  param_profile_velocity6[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity6));

  uint8_t param_profile_accel1[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel2[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel3[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel4[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel5[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel6[4]; // Convert int32 -> uint32

  param_profile_accel1[0] = DXL_LOBYTE(DXL_LOWORD(profile_accel1));
  param_profile_accel1[1] = DXL_HIBYTE(DXL_LOWORD(profile_accel1));
  param_profile_accel1[2] = DXL_LOBYTE(DXL_HIWORD(profile_accel1));
  param_profile_accel1[3] = DXL_HIBYTE(DXL_HIWORD(profile_accel1));

  param_profile_accel2[0] = DXL_LOBYTE(DXL_LOWORD(profile_accel2));
  param_profile_accel2[1] = DXL_HIBYTE(DXL_LOWORD(profile_accel2));
  param_profile_accel2[2] = DXL_LOBYTE(DXL_HIWORD(profile_accel2));
  param_profile_accel2[3] = DXL_HIBYTE(DXL_HIWORD(profile_accel2));

  param_profile_accel3[0] = DXL_LOBYTE(DXL_LOWORD(profile_accel3));
  param_profile_accel3[1] = DXL_HIBYTE(DXL_LOWORD(profile_accel3));
  param_profile_accel3[2] = DXL_LOBYTE(DXL_HIWORD(profile_accel3));
  param_profile_accel3[3] = DXL_HIBYTE(DXL_HIWORD(profile_accel3));

  param_profile_accel4[0] = DXL_LOBYTE(DXL_LOWORD(profile_accel4));
  param_profile_accel4[1] = DXL_HIBYTE(DXL_LOWORD(profile_accel4));
  param_profile_accel4[2] = DXL_LOBYTE(DXL_HIWORD(profile_accel4));
  param_profile_accel4[3] = DXL_HIBYTE(DXL_HIWORD(profile_accel4));

  param_profile_accel5[0] = DXL_LOBYTE(DXL_LOWORD(profile_accel5));
  param_profile_accel5[1] = DXL_HIBYTE(DXL_LOWORD(profile_accel5));
  param_profile_accel5[2] = DXL_LOBYTE(DXL_HIWORD(profile_accel5));
  param_profile_accel5[3] = DXL_HIBYTE(DXL_HIWORD(profile_accel5));

  param_profile_accel6[0] = DXL_LOBYTE(DXL_LOWORD(profile_accel6));
  param_profile_accel6[1] = DXL_HIBYTE(DXL_LOWORD(profile_accel6));
  param_profile_accel6[2] = DXL_LOBYTE(DXL_HIWORD(profile_accel6));
  param_profile_accel6[3] = DXL_HIBYTE(DXL_HIWORD(profile_accel6));


  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL1_ID, param_profile_velocity1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID 5");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL2_ID, param_profile_velocity2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID 7");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL3_ID, param_profile_velocity3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID 11");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL4_ID, param_profile_velocity4);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID 4");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL5_ID, param_profile_velocity5);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID 6");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL6_ID, param_profile_velocity6);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID 10");
  }


  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL1_ID, param_profile_accel1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID 5");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL2_ID, param_profile_accel2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID 7");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL3_ID, param_profile_accel3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID 11");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL4_ID, param_profile_accel4);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID 4");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL5_ID, param_profile_accel5);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID 6");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL6_ID, param_profile_accel6);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID 10");
  }

  dxl_comm_result = groupSyncWrite_velocity.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setVelocity : [ID:%d] [Velocity:%d]", DXL1_ID, profile_velocity1);
    ROS_INFO("setVelocity : [ID:%d] [Velocity:%d]", DXL2_ID, profile_velocity2);
    ROS_INFO("setVelocity : [ID:%d] [Velocity:%d]", DXL3_ID, profile_velocity3);
    ROS_INFO("setVelocity : [ID:%d] [Velocity:%d]", DXL4_ID, profile_velocity4);
    ROS_INFO("setVelocity : [ID:%d] [Velocity:%d]", DXL5_ID, profile_velocity5);
    ROS_INFO("setVelocity : [ID:%d] [Velocity:%d]", DXL6_ID, profile_velocity6);
  } else {
    ROS_ERROR("Failed to set profile velocity! Result: %d", dxl_comm_result);
  }

  dxl_comm_result = groupSyncWrite_accel.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setAccel : [ID:%d] [Accel:%d]", DXL1_ID, profile_accel1);
    ROS_INFO("setAccel : [ID:%d] [Accel:%d]", DXL2_ID, profile_accel2);
    ROS_INFO("setAccel : [ID:%d] [Accel:%d]", DXL3_ID, profile_accel3);
    ROS_INFO("setAccel : [ID:%d] [Accel:%d]", DXL4_ID, profile_accel4);
    ROS_INFO("setAccel : [ID:%d] [Accel:%d]", DXL4_ID, profile_accel5);
    ROS_INFO("setAccel : [ID:%d] [Accel:%d]", DXL4_ID, profile_accel6);
  } else {
    ROS_ERROR("Failed to set profile accel! Result: %d", dxl_comm_result);
  }

  groupSyncWrite_velocity.clearParam();
  groupSyncWrite_accel.clearParam();
  //--------------------------------------------------------------------------------------------------------------------------------
  
  //나중에 모터 없으면 error 출력하는 부분 추가해
  ros::spin();

  portHandler->closePort();
  return 0;
}
