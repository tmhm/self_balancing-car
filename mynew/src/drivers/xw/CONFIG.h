
#ifndef __CONFIG__
#define __CONFIG__
//------------------------------------------------------------------------------
#ifdef CONFIG_GLOBALS
    #define CONFIG_EXT
#else
    #define CONFIG_EXT extern
#endif // CONFIG_GLOBALS

#include  "common.h"
//--------------------------发动静止控制------------------------
#define start_time_count              150     

#define TIME1MS_INT_STOP 			g_nTime1MSStopFlag = 1
#define TIME1MS_INT_START			g_nTime1MSStopFlag = 0;
#define TIME1MS_INT_FLAG			g_nTime1MSStopFlag







//------------------------------------------------------------------------------
//==============================================================================
//              全局变量定义区			CONFIGRATION 
//------------------------------------------------------------------------------

int g_nGravityOffset;
int g_nGyroscopeOffset;
int g_nGravityMax;
int g_nGravityMin;

float g_fAngleControlP, g_fAngleControlD;
float g_fSpeedControlP, g_fSpeedControlI;
float g_fDirectionControlP;

float g_fGyroscopeAngleRatio;
float g_fGravityTimeConstant;
float g_fDeadVoltage;
float g_fCarSpeedSet;

int g_nDirLeftOffset;
int g_nDirRightOffset;

unsigned int g_nPad;
unsigned int g_nInitFlag;
u8  g_fStarttimeCount;



//------------------------------------------------------------------------------
#define GRAVITY_OFFSET_DEFAULT			4000
#define GYROSCOPE_OFFSET_DEFAULT		2000
#define GRAVITY_MAX_DEFAULT				3000
#define GRAVITY_MIN_DEFAULT				1000
#define ANGLE_CONTROL_P_DEFAULT			50
#define ANGLE_CONTROL_D_DEFAULT			1.5
#define SPEED_CONTROL_P_DEFAULT			2
#define SPEED_CONTROL_I_DEFAULT			0.2
#define DIRECTION_CONTROL_P_DEFAULT		1
#define DIR_LEFT_OFFSET_DEFAULT			0
#define DIR_RIGHT_OFFSET_DEFAULT		0
#define DEAD_VOLTAGE_DEFAULT			0x40
#define INIT_FLAG_DEFAULT				0x55aa
#define GYROSCOPE_ANGLE_RATIO_DEFAULT	2.0
#define GRAVITY_TIME_CONSTANT_DEFAULT	2

#define CAR_SPEED_SET_DEFAULT			0
#define CAR_PAD_DEFAULT					0

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//#define ARGUMENT_ADDRESS				0x1A00

void ArgumentInit(void);
//void ArgumentLoad(void);
//void ArgumentSave(void);

//------------------------------------------------------------------------------


//==============================================================================
//             END OF THE FILE : CONFIG.H
//------------------------------------------------------------------------------
#endif // __CONFIG__
