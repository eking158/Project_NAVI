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
 * $ rostopic pub -1 /dynamicxel_set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id3: 3, id5: 5, id7: 7, position1: 0, position3: 1000, position5: 1000, position7: 1000}"
 * $ rostopic pub -1 /dynamicxel_set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 0, position2: 1000}"
 * $ rostopic pub -1 /sync_set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 1000, position2: 0}"
 * $ rosservice call /sync_get_position "{id1: 1, id2: 2}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

/*
Control NAVI middle Dynamixels

Left_shoulder_yaw -> Left_elbow_pitch
9 -> 11

Right_shoulder_yaw -> Right_elbow_pitch
8 -> 10
*/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/SyncGetPosition.h"
#include "dynamixel_sdk_examples/SyncSetPosition.h"
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
uint32_t profile_accel1 = 2;
uint32_t profile_accel2 = 2;
uint32_t profile_accel3 = 2;
uint32_t profile_accel4 = 2;

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               9              // DXL1 ID
#define DXL2_ID               11              // DXL2 ID
#define DXL3_ID               8              // DXL3 ID
#define DXL4_ID               10              // DXL4 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/navi_430"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

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

void syncSetPositionCallback(const dynamixel_sdk_examples::SyncSetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position1[4];
  uint8_t param_goal_position2[4];
  uint8_t param_goal_position3[4];
  uint8_t param_goal_position4[4];

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

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id4, msg->position4);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupSyncWrite.clearParam();
}

int main(int argc, char ** argv)
{
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
    ROS_INFO("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    //ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    //return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_INFO("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    //ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    //return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_INFO("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    //ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    //return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_INFO("Failed to enable torque for Dynamixel ID %d", DXL4_ID);
    //ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    //return -1;
  }

  int dxl_addparam_result = false;

  uint8_t param_profile_velocity1[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity2[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity3[4]; // Convert int32 -> uint32
  uint8_t param_profile_velocity4[4]; // Convert int32 -> uint32

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

  uint8_t param_profile_accel1[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel2[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel3[4]; // Convert int32 -> uint32
  uint8_t param_profile_accel4[4]; // Convert int32 -> uint32

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


  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL1_ID, param_profile_velocity1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL2_ID, param_profile_velocity2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL3_ID, param_profile_velocity3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID");
  }
  dxl_addparam_result = groupSyncWrite_velocity.addParam((uint8_t)DXL4_ID, param_profile_velocity4);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam velocity to groupSyncWrite for Dynamixel ID");
  }


  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL1_ID, param_profile_accel1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL2_ID, param_profile_accel2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL3_ID, param_profile_accel3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID");
  }
  dxl_addparam_result = groupSyncWrite_accel.addParam((uint8_t)DXL4_ID, param_profile_accel4);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam accel to groupSyncWrite for Dynamixel ID");
  }

  dxl_comm_result = groupSyncWrite_velocity.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL1_ID, profile_velocity1);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL2_ID, profile_velocity2);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL3_ID, profile_velocity3);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL4_ID, profile_velocity4);
  } else {
    ROS_ERROR("Failed to set profile velocity! Result: %d", dxl_comm_result);
  }

  dxl_comm_result = groupSyncWrite_accel.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL1_ID, profile_accel1);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL2_ID, profile_accel2);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL3_ID, profile_accel3);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL4_ID, profile_accel4);
  } else {
    ROS_ERROR("Failed to set profile accel! Result: %d", dxl_comm_result);
  }

  groupSyncWrite_velocity.clearParam();
  groupSyncWrite_accel.clearParam();

  
  //나중에 모터 없으면 error 출력하는 부분 추가해

  ros::init(argc, argv, "navi_dynamixel_middle_node");
  ros::NodeHandle nh;
  //ros::ServiceServer sync_get_position_srv = nh.advertiseService("/sync_get_position", syncGetPresentPositionCallback);
  ros::Subscriber sync_set_position_sub = nh.subscribe("/navi/dynamicxel_set_position_middle", 10, syncSetPositionCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
