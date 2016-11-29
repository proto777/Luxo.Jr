/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 1.0
// This example is tested with two Dynamixel MX-28, and an USB2DYNAMIXEL
// Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 1000000)
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#1 ID: 3
#define DXL4_ID                         4                   // Dynamixel#2 ID: 4
#define BAUDRATE                        1000000
#define DEVICENAME                      "COM6"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
float DXL_MINIMUM_POSITION_VALUE1 = 0;	                // Dynamixel will rotate between this value
float DXL_MAXIMUM_POSITION_VALUE1 = 1023;                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
float DXL_MINIMUM_POSITION_VALUE2 = 0;	                // Dynamixel will rotate between this value
float DXL_MAXIMUM_POSITION_VALUE2 = 1023;
float DXL_MINIMUM_POSITION_VALUE3 = 0;	                // Dynamixel will rotate between this value
float DXL_MAXIMUM_POSITION_VALUE3 = 1023;
float DXL_MINIMUM_POSITION_VALUE4 = 0;	                // Dynamixel will rotate between this value
float DXL_MAXIMUM_POSITION_VALUE4 = 1023;

#define DXL_MOVING_STATUS_THRESHOLD     2                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  float dxl_goal_position1[2] = { DXL_MINIMUM_POSITION_VALUE1, DXL_MAXIMUM_POSITION_VALUE1 };         // Goal position
  float dxl_goal_position2[2] = { DXL_MINIMUM_POSITION_VALUE2, DXL_MAXIMUM_POSITION_VALUE2 };         // Goal position
  float dxl_goal_position3[2] = { DXL_MINIMUM_POSITION_VALUE3, DXL_MAXIMUM_POSITION_VALUE3 };         // Goal position
  float dxl_goal_position4[2] = { DXL_MINIMUM_POSITION_VALUE4, DXL_MAXIMUM_POSITION_VALUE4 };         // Goal position

  uint8_t dxl_error1 = 0;                          // Dynamixel error1
  uint8_t dxl_error2 = 0;                          // Dynamixel error2
  uint8_t dxl_error3 = 0;                          // Dynamixel error3
  uint8_t dxl_error4 = 0;                          // Dynamixel error4
  uint8_t param_goal_position1[2];
  uint8_t param_goal_position2[2];
  uint8_t param_goal_position3[2];
  uint8_t param_goal_position4[2];
  uint16_t dxl1_present_position = 0, dxl2_present_position = 0, dxl3_present_position = 0, dxl4_present_position = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error1);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error1 != 0)
  {
    packetHandler->printRxPacketError(dxl_error1);
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error2);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error2 != 0)
  {
    packetHandler->printRxPacketError(dxl_error2);
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
  }

  // Enable Dynamixel#3 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error3);
  if (dxl_comm_result != COMM_SUCCESS)
  {
	  packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error3 != 0)
  {
	  packetHandler->printRxPacketError(dxl_error3);
  }
  else
  {
	  printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
  }

  // Enable Dynamixel#4 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error4);
  if (dxl_comm_result != COMM_SUCCESS)
  {
	  packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error4 != 0)
  {
	  packetHandler->printRxPacketError(dxl_error4);
  }
  else
  {
	  printf("Dynamixel#%d has been successfully connected \n", DXL4_ID);
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Allocate goal position value into byte array
    param_goal_position1[0] = DXL_LOBYTE(dxl_goal_position1[index]);
    param_goal_position1[1] = DXL_HIBYTE(dxl_goal_position1[index]);
	param_goal_position2[0] = DXL_LOBYTE(dxl_goal_position2[index]);
	param_goal_position2[1] = DXL_HIBYTE(dxl_goal_position2[index]);
	param_goal_position3[0] = DXL_LOBYTE(dxl_goal_position3[index]);
	param_goal_position3[1] = DXL_HIBYTE(dxl_goal_position3[index]);
	param_goal_position4[0] = DXL_LOBYTE(dxl_goal_position4[index]);
	param_goal_position4[1] = DXL_HIBYTE(dxl_goal_position4[index]);

    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position1);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
      return 0;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position2);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
      return 0;
    }

	// Add Dynamixel#3 goal position value to the Syncwrite storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position3);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
		return 0;
	}

	// Add Dynamixel#4 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position4);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
		return 0;
	}

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Read Dynamixel#1 present position
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error1);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error1 != 0)
      {
        packetHandler->printRxPacketError(dxl_error1);
      }

      // Read Dynamixel#2 present position
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error2);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error2 != 0)
      {
        packetHandler->printRxPacketError(dxl_error2);
      }

	  // Read Dynamixel#3 present position
	  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION, &dxl3_present_position, &dxl_error3);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error3 != 0)
	  {
		  packetHandler->printRxPacketError(dxl_error3);
	  }

	  // Read Dynamixel#4 present position
	  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION, &dxl4_present_position, &dxl_error4);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error4 != 0)
	  {
		  packetHandler->printRxPacketError(dxl_error4);
	  }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_position1[index], dxl1_present_position, DXL2_ID, dxl_goal_position2[index], dxl2_present_position, DXL3_ID, dxl_goal_position3[index], dxl3_present_position, DXL4_ID, dxl_goal_position4[index], dxl4_present_position);

	} while ((abs(dxl_goal_position1[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position2[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position3[index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position4[index] - dxl4_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error1);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error1 != 0)
  {
    packetHandler->printRxPacketError(dxl_error1);
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error2);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error2 != 0)
  {
    packetHandler->printRxPacketError(dxl_error2);
  }

  // Disable Dynamixel#3 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error3);
  if (dxl_comm_result != COMM_SUCCESS)
  {
	  packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error3 != 0)
  {
	  packetHandler->printRxPacketError(dxl_error3);
  }

  // Disable Dynamixel#4 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error4);
  if (dxl_comm_result != COMM_SUCCESS)
  {
	  packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error4 != 0)
  {
	  packetHandler->printRxPacketError(dxl_error4);
  }

  // Close port
  portHandler->closePort();

  return 0;
}
