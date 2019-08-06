/*
  ax12.h - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// This version hacked up by Kurt Eckhardt...


#ifndef _DXL_SERIAL_H_
#define _DXL_SERIAL_H_
#include <Arduino.h> 

#define AX12_MAX_SERVOS             30
#define AX12_BUFFER_SIZE            32
#define DXL_SUPPORT_PROTOCOL 2

#if defined(__ARDUINO_X86__)
    // Currently assume using USB2AX or the like
#define PAX12Serial (void*)0   // no default yet... 
#else    
#define PAX12Serial &Serial1
#endif

#ifndef PAX12Serial
#if defined(UBRR3H)
#define PAX12Serial &Serial3
#else
#define PAX12Serial &Serial1
#endif
#endif

//=============================================================================
// AX Servo Registers
//=============================================================================
/** EEPROM AREA **/
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23
/** RAM AREA **/
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

/** Status Return Levels **/
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

/** Instruction Set **/
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

/** Error Levels **/
#define ERR_NONE                    0
#define ERR_VOLTAGE                 1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64

/** AX-S1 **/
#define AX_LEFT_IR_DATA             26
#define AX_CENTER_IR_DATA           27
#define AX_RIGHT_IR_DATA            28
#define AX_LEFT_LUMINOSITY          29
#define AX_CENTER_LUMINOSITY        30
#define AX_RIGHT_LUMINOSITY         31
#define AX_OBSTACLE_DETECTION       32
#define AX_BUZZER_INDEX             40

//=============================================================
// Defines for X series (XL430)
//=============================================================================
#define DXL_PING                    1
#define DXL_READ_DATA               2
#define DXL_WRITE_DATA              3
#define DXL_REG_WRITE               4
#define DXL_ACTION                  5
#define DXL_RESET                   6
#define DXL_SYNC_READ               0x82
#define DXL_SYNC_WRITE              0x83


/** EEPROM AREA **/
#define DXL_X_MODEL_NUMBER         0 //2 1060(xl430-250)
#define DXL_X_MODEL_INFORMATION    2 //4
#define DXL_X_VERSION              6
#define DXL_X_ID                   7   // 1 1
#define DXL_X_BAUD_RATE            8   // 1 1
#define DXL_X_RETURN_DELAY_TIME    9   // 1 250
#define DXL_X_DRIVE_MODE           10  // 1 0
#define DXL_X_OPERATING_MODE       11  // 1 3
#define DXL_X_SECONDARY_ID         12  // 1 255
#define DXL_X_PROTOCOL_VERSION     13  // 1 - 2
#define DXL_X_HOMING_OFFSET        20  // 4 - 0
#define DXL_X_MOVING_THRESHOLD     24  // 4 - 10
#define DXL_X_TEMPERATURE_LIMIT    31  //1 - 72
#define DXL_X_MAX_VOLTAGE_LIMIT    32  // 2 140
#define DXL_X_MIN_VOLTAGE_LIMIT    34  // 2 60
#define DXL_X_PWM_LIMIT            36  // 2 885
#define DXL_X_ACCELERATION_LIMIT   40  // 4 32767
#define DXL_X_VElOCITY_LIMIT       44  // 4 415
#define DXL_X_MAX_POSITION_LIMIT   48  // 4 4095
#define DXL_X_MIN_POSITION_LIMIT   52  // 4 0
#define DXL_X_SHUTDOWN             63  // 1 52

/** RAM AREA **/
#define DXL_X_TORQUE_ENABLE        64  // 1 0
#define DXL_X_LED                  65  // 1 0
#define DXL_X_STATUS_RETURN_LEVEL  68  // 1 2
#define DXL_X_REGISTERED_INSTRUCTION 69 //1 (R)
#define DXL_X_HARDWARE_ERROR_STATUS  70 // 1 (R)
#define DXL_X_VELOCITY_I_GAIN      76  // 2 1000 
#define DXL_X_VELOCITY_P_GAIN      78  // 2 100 
#define DXL_X_POSITION_D_GAIN      80  // 2 4000 
#define DXL_X_POSITION_I_GAIN      82  // 2 0
#define DXL_X_POSITION_P_GAIN      84  // 2 640
#define DXL_X_FEEDFORWARD_2_GAIN   88  // 2 0
#define DXL_X_FEEDFORWARD_1_GAIN   90  // 2 0
#define DXL_X_BUS_WATCHDOG         98  // 1 -
#define DXL_X_GOAL_PWM             100 // 2 
#define DXL_X_GOAL_VELOCITY        104 // 4
#define DXL_X_PROFILE_ACCELERATION 108 // 4
#define DXL_X_PROFILE_VELOCITY     112 // 4
#define DXL_X_GOAL_POSITION        116 // 4
#define DXL_X_REALTIME_TICK        120 // 2 (R)
#define DXL_X_MOVING               122 // 1 (R)
#define DXL_X_MOVING_STATUS        123 // 1 (R)
#define DXL_X_PRESENT_PWN          124 // 2 (R)
#define DXL_X_PRESENT_LOAD         126 // 2 (R)
#define DXL_X_PRESENT_VELOCITY     128 // 4 (R)
#define DXL_X_PRESENT_POSITION     132 // 4 (R)
#define DXL_X_VELOCITY_TRAJECTORY  136 // 4 (R)
#define DXL_X_POSITION_TRAJECTORY  140 // 4 (R)
#define DXL_X_PRESENT_INPUT_VOLTAGE  144 // 2 (R)
#define DXL_X_PRESENT_TEMPERATURE  146 // 1 (R)
// (Indirects maybe added later?)


//=============================================================
// Helper functions
#if defined(KINETISK) || defined(KINETISL) || defined(__IMXRT1052__) || defined(__IMXRT1062__)
void dxlInit(long baud, HardwareSerial* pserial, int direction_pin = -1, int tx_pin = -1, int rx_pin = -1);
#endif
void dxlInit(long baud, Stream* pStream, int direction_pin = -1, int tx_pin = -1, int rx_pin = -1);
void dxlEnd();

void setTXall();     // for sync write
void setTX(int id);
void setRX(int id);

void dxlWrite(unsigned char data);
void dxlWrite(unsigned char *pdata, int length);


int ax12ReadPacket(int length);
int ax12GetRegister(int id, int regstart, int length);
void ax12SetRegister(int id, int regstart, int data);
void ax12SetRegister2(int id, int regstart, int data);
int dxlGetLastError();

bool dxlP1SyncWrite(uint8_t servo_count, uint8_t regstart, uint8_t regcount, uint8_t *buffer);

// protocol 2

void dxlP2SetRegisters(int id, int regstart, uint32_t data, uint8_t data_size=1);
int dxlP2GetRegisters(int id, int regstart, int length);
int dxlP2ReadPacket();
uint32_t dxlP2Ping(int id);
bool dxlP2SyncWrite(uint8_t servo_count, uint16_t regstart, uint16_t regcount, uint8_t *buffer);
bool dxlP2SyncRead(uint8_t servo_count, uint16_t regstart, uint16_t regcount, uint8_t *buffer);

extern unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
extern uint8_t ax_tx_buffer[AX12_BUFFER_SIZE];


//==============================================
// Helper functions only currently valid for Protocol 1
//=============================================================
void inline ax12Init(long baud, Stream* pStream, int direction_pin = -1) 
    {dxlInit(baud, pStream, direction_pin);}
void dxlEnd();

void setTXall();     // for sync write
void setTX(int id);
void setRX(int id);

void inline ax12write(unsigned char data) {dxlWrite(data);}
void inline ax12write(unsigned char *pdata, int length) {dxlWrite(pdata, length);}
void inline ax12writeB(unsigned char data) {dxlWrite(data);}

bool dxlP1Ping(int id);
int ax12ReadPacket(int length);
int ax12GetRegister(int id, int regstart, int length);
void ax12SetRegister(int id, int regstart, int data);
void ax12SetRegister2(int id, int regstart, int data);
int inline ax12GetLastError() {return dxlGetLastError();}


#define SetPosition(id, pos) (ax12SetRegister2(id, AX_GOAL_POSITION_L, pos))
#define GetPosition(id) (ax12GetRegister(id, AX_PRESENT_POSITION_L, 2))
#define TorqueOn(id) (ax12SetRegister(id, AX_TORQUE_ENABLE, 1))
#define Relax(id) (ax12SetRegister(id, AX_TORQUE_ENABLE, 0))

#define GetLeftIRData(id) (ax12GetRegister(id, AX_LEFT_IR_DATA))
#define GetCenterIRData(id) (ax12GetRegister(id, AX_CENTER_IR_DATA))
#define GetRightIRData(id) (ax12GetRegister(id, AX_RIGHT_IR_DATA))
#define GetObstacles(id) (ax12GetRegister(id, OBSTACLE_DETECTION))

#define PlayTone(id, note) (ax12SetRegister(id, AX_BUZZER_INDEX, note))

#endif
