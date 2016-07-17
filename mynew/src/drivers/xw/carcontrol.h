/*
**==============================================================================
** CARSUB.H:             -- by Dr. ZhuoQing, 2012-2-3
**
**  Description:
**
**==============================================================================
*/
#ifndef __carcontrol__
#define __carcontrol__
//------------------------------------------------------------------------------
#ifdef CARSUB_GLOBALS
    #define CARSUB_EXT
#else
    #define CARSUB_EXT  extern
#endif // CARSUB_GLOBALS
//------------------------------------------------------------------------------

#include "common.h"
#include "include.h"

//--------------------------������ֹ����------------------------
#define start_time_count              10     

#define TIME1MS_INT_STOP 			g_nTime1MSStopFlag = 1
#define TIME1MS_INT_START			g_nTime1MSStopFlag = 0;
#define TIME1MS_INT_FLAG			g_nTime1MSStopFlag

//------------------------------------------------------------------------------
//==============================================================================
//              ȫ�ֱ���������			CONFIGRATION 
//------------------------------------------------------------------------------
u8  CCD_NRF_flag;        //���߷��ͱ�־λ
u8  ImageCCD_flag;      //ȡadֵ��־λ
s32  RightPulse,LeftPulse;
u16 leftn,rightn;

s16  Acc_Xvalue;   //���ٶ�x��ֵ���������
u8  slope_flag;
u8  slope_key;

u8  gu8_StarttimeCount;      //������ֹ��ʱ�䡣��λ��ms
u8  g_n1MSEventCount;        //�жϺ��������ʱ����

//------------------------------------------------------------------------------




//void ArgumentInit(void);

void Discuss_Init(void);
void CarSubInit(void);                          // Total Car Control Initialize functions
void Speed_add_init(void);


u8  Get_Barrier(void);
void  Barrier_init(void);
//------------------------------------------------------------------------------

#define TIMETEST_EN						1
#if TIMETEST_EN
	#define TIMETEST_ON					LED_ON
	#define TIMETEST_OFF			        	LED_OFF
#else
	#define TIMETEST_ON
	#define TIMETEST_OFF
#endif // TIMETEST_EN

CARSUB_EXT int g_nTimeTestFlag;
#define TIME_TEST_ENA					g_nTimeTestFlag = 1
#define TIME_TEST_DIS					g_nTimeTestFlag = 0

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#define CONTROL_PERIOD					12		// Every 5 ms adjust all control

//------------------------------------------------------------------------------
//						MOTOR SPEED VOLTAGE OUTPUT FUNCTIONS
#define OPTICAL_ENCODE_CONSTANT			200     //
#define SPEED_CONTROL_COUNT				17  	// 17  * 4.8 ms

CARSUB_EXT s16 g_nLeftMotorPulseSigma, g_nRightMotorPulseSigma;
void GetMotorPulse(void);

//------------------------------------------------------------------------------
void SetMotorVoltage(float fLeftVoltage, float fRightVoltage);
                                                // Voltage : > 0 : Move forward;
                                                //           < 0 : Move backward
#define MOTOR_STOP						SetMotorVoltage(1000, 1000)


//------------------------------------------------------------------------------
//						MOTOR SPEED CONTROL OUTPUT
// 
CARSUB_EXT float g_fLeftMotorOut, g_fRightMotorOut;
#define MOTOR_OUT_MAX					1.0
#define MOTOR_OUT_MIN					-1.0


float Car_DeadVoltage;
#define MOTOR_OUT_DEAD_VAL			Car_DeadVoltage     //   0.00053	  //g_fDeadVoltage	//Unit : Full output
void MotorSpeedOut(void);						// 


CARSUB_EXT float g_fAngleControlOut, g_fSpeedControlOut, g_fDirectionControlOut;

void MotorOutput(void);

//------------------------------------------------------------------------------
//						MOTOR SPEED CONTROL
//
// Input : g_nCarSpeedSet: Speed setting value
//       : g_nCarSpeed: Motor Speed feedback value.
//
// Output :  g_nSpeedControlOut;
//
// Algrithm : PI adjust
#define SPEED_CONTROL_PERIOD			(SPEED_CONTROL_COUNT * CONTROL_PERIOD) //unit: ms
#define CAR_SPEED_CONSTANT				1000.0 / (SPEED_CONTROL_COUNT * CONTROL_PERIOD) / (float)OPTICAL_ENCODE_CONSTANT

CARSUB_EXT float g_fCarSpeed;			// Car Speed = (LeftMotorSpeedCount + nRightMotorSpeedCount) / 2;


//#define CAR_SPEED_SET_DEFAULT			0

float g_fCarSpeed_defult;     
CARSUB_EXT float g_fCarSpeedSet;
#define CAR_SPEED_SET					g_fCarSpeed_defult; //g_Argument.fCarSpeedSet;
#define CarSpeedMax                                    ( g_fCarSpeedSet+2.0)

float  Speed_Add_Value;
u8  start_time_flag;
s16 CarSpeed_data;

//------------------------------------------------------------------------------
#define CAR_SPEED_SET_MAX				50.0 		//
CARSUB_EXT float g_fSpeedControlIntegral;			// Keep the Speed Control Integral value
#define SPEED_CONTROL_OUT_MAX			MOTOR_OUT_MAX * 5
#define SPEED_CONTROL_OUT_MIN			MOTOR_OUT_MIN * 5

CARSUB_EXT float g_fSpeedControlOutOld, g_fSpeedControlOutNew;
float g_fSpeed_OutValue;

//------------------------------------------------------------------------------
//					SPEED CONTROL
#define SPEED_CONTROL_STOP				g_nSpeedControlFlag = 0
#define SPEED_CONTROL_START				g_nSpeedControlFlag = 1
CARSUB_EXT int g_nSpeedControlFlag;


//------------------------------------------------------------------------------

CARSUB_EXT u16 g_nSpeedControlCount, g_nSpeedControlPeriod;


#define SPEED_CONTROL_P				SpeedControlP 
#define SPEED_CONTROL_I				SpeedControlI 
#define SPEED_CONTROL_D				SpeedControlD 

float SpeedControlP,SpeedControlI,SpeedControlD;  
#define Speed_default_P         0.115     //0.115    //0.15 // 0.13 // 0.15   // 0.15  
#define Speed_default_I         0.008 //0.01       //0.025   // 0.025 // 0.03   //0.05   
#define Speed_default_D         -0.012

void SpeedControl(void);
void SpeedControlOutput(void);

u8    Speed_Out_flag;

s16 g_Gravity_offset; 

float Speederr_new,Speederr_pre;
float SpeederrD;

u8  Roadb_DownSpe;      //�ϰ��ڶ��μ���
//float  Car_Accspeed;    //���ٶ�
//==============================================================================
//			��ȡ�Ƕȼ��Ǽ��ٶ� 
//------------------------------------------------------------------------------
#define GRAVITY_OFFSET				g_Gravity_offset  //-1150  //-1654    //-1700	-1654ͦ�ã�����ǰ����	
#define GRAVITY_MAX				4118	//g_nGravityMax		// 3030
#define GRAVITY_MIN				-4147	//g_nGravityMin		// 1030

//#define GYROSCOPE1_OFFSET			2076	//g_nGyroscopeOffset	
#define GYROSCOPE_ANGLE_RATIO            angle_ratio      //0.6      // 0.295 ,5����      //0.173   //0.165  
                                                        //�Ŵ���10�������ǣ�0.173���ĸ���Ч���ܺ�
float angle_ratio;

#define GRAVITY_ADJUST_TIME_CONSTANT            4.0   //2 //4


s16 VOLTAGE_GRAVITY;		
s16 VOLTAGE_GYRO;

s16   g_s16GYROSCOPE1_OFFSET;     //����ֱ����������ƫ��
s16   g_s16GYRO_DirOFFSET;     //����ת���������ƫ��

float g_fCarAngle;
float g_fGravityAngle, g_fGyroscopeAngleSpeed;
float g_fGyroscopeAngleIntegral;

float angle_m;    //Kalman_Filter����ĽǶ�

//float Q_angle, Q_gyro;  
float R_angle;

float gyro_m;
float q_bias;

//float g_fCarAngle1;
//-------------------------------------------------------
//�����˲�
//-------------------------------------------------------
//void complement_filter(float angle_m_cf,float gyro_m_cf);

//float angle,angle_dot; 		//�ⲿ��Ҫ���õı���
//-------------------------------------------------------
// static float bias_cf;
//static const float dt=0.01;


#define CAR_ANGLE_RANGE					180.0
#define GRAVITY_ANGLE_RATIO				0.02178    //((float)CAR_ANGLE_RANGE / (float)(GRAVITY_MAX - GRAVITY_MIN)) 

//#define GYROSCOPE_ANGLE_RATIO			g_fGyroscopeAngleRatio		// 2.40    //
#define GYROSCOPE_ANGLE_SIGMA_FREQUENCY	    (1000.0 / CONTROL_PERIOD)

//#define GRAVITY_ADJUST_TIME_CONSTANT	g_fGravityTimeConstant  //2.0     // unit : second

//------------------------------------------------------------------------------
//u8  AngleCalculate_flag;
//void AngleCalculate(void);
//void  AngleCalculate(void); 
                      
void  AngleCalculate(float VOLTAGE_GRAVITY,float VOLTAGE_GYRO);
void  Kalman_Filter(float VOLTAGE_GRAVITY,float VOLTAGE_GYRO);	
//static float angle,angle_dot; 		//�ⲿ��Ҫ���õı���
//static float acceler,gyro;	
//void complement_filter(float angle_m_cf,float gyro_m_cf);
//void AD_calculate(void);


//------------------------------------------------------------------------------
//  CONTROL CAR ANGLE 
//	Input : CAR_ANGLE_SET 0
//          g_fCarAngle, g_fGyroscopeAngleSpeed
//  Output : g_fAngleControlOut;
#define CAR_ANGLE_SET		  		        0    //-1.757 //0     //-1.4335	      //0

float AngleControlP,AngleControlD;	
//float Angle_err_Integral,AngleControl_I;
//------------------------------------------------------------------------------
#define CAR_ANGLE_SPEED_SET				0
#define ANGLE_CONTROL_P				         AngleControlP         // 0.38600    �е��ͷ
#define ANGLE_CONTROL_D					 AngleControlD		//0.00480  

#define Angle_default_P              0.23 //     0.23   // 0.27       // 0.3  
#define Angle_default_D              0.0033     // 0.0033     // 0.0026      

//#define AngleControl_I_defult        0.0   //  0.001//  0.00004  //     0.0001    

#define ANGLE_CONTROL_STOP				g_nAngleControlFlag = 0
#define ANGLE_CONTROL_START				g_nAngleControlFlag = 1

u8 g_nAngleControlFlag;

void AngleControl(void);
#define ANGLE_CONTROL_OUT_MAX			MOTOR_OUT_MAX * 10
#define ANGLE_CONTROL_OUT_MIN			MOTOR_OUT_MIN * 10

//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// 					DIRECTION CONTROL
#define DIRECTION_CONTROL_STOP			g_nDirectionControlFlag = 0
#define DIRECTION_CONTROL_START			g_nDirectionControlFlag = 1
CARSUB_EXT int g_nDirectionControlFlag;

//void DirectionControl(void);
//void DirectionControlOutput(void);

#define DIRECTION_CONTROL_OUT_MAX		MOTOR_OUT_MAX
#define DIRECTION_CONTROL_OUT_MIN		MOTOR_OUT_MIN

#define DIRECTION_CONTROL_DEADVALUE		0.0


#define DIR_CONTROL_P					Dir_mControl_P
#define DIR_CONTROL_I                                   Dir_mControl_I
#define DIR_CONTROL_D                                   Dir_mControl_D

float Dir_mControl_P,Dir_mControl_I,Dir_mControl_D;

#define    DirectionControl_default_P   0.017     //0.021    // 0.018    // 0.02
#define    DirectionControl_default_I   0.0004    // 0.0005   // 0.0
#define    DirectionControl_default_D   0.004    // 0.0045    //0.0035

#define LEFT_RIGHT_MINIMUM				2

CARSUB_EXT float g_fDirectionControlOutOld,g_fDirectionControlOutNew;
CARSUB_EXT float g_fDir_OutValue;

CARSUB_EXT int g_nDirectionControlCount, g_nDirectionControlPeriod;


#define DIRECTION_CONTROL_COUNT			4
#define DIRECTION_CONTROL_PERIOD		(DIRECTION_CONTROL_COUNT * CONTROL_PERIOD)

//-----------------------------------------------------------------------------

s16	CCD_GateValve,CCD_GateValve_pre,CCD_Gate_Vir;		//CCD��̬��ֵ
//s16     CCDGate_PN_Diff;
//u8      CCDgate_update;
//u8	Left_Distance,Right_Distance;
//u8	Left_Distance_flag,Right_Distance_flag;

u8      Left_site,Right_site;       //���Һ��ߵ��λ��
u8      Left_site_old,Right_site_old;     //�ϴ����Һ��ߵ��λ��

float   Line_Middle_site;         //����ܵ�������λ��
float   Car_Middle_site;          //������е�λ��

u8  can_calculate_flag;

//u8    CCD_first_flag;   //CCD��һ�δ����־����1��ʾ�Ѵ����һ��


float CCD_Err;		//�����Ϊ��׼
                    //�����ߵ����
float CCD_Err_pre;
float CCD_Err_Dif;
float   CCD_err_Integral;

u16   Dir_AD;    //ת�������ǵ�ADֵ

float    LeftRight_distance;

u8      Dir_Not_flag;   //û���ҵ�����,����ʮ���䣬��λ,  ���򲻿���
u8      Corss_deal_Flag;    //ʮ���䴦���־λ���Ѵ�����λ
u16     Cross_Count;    //ʮ���������

float fleft_Correction; //ʮ�������������
u8    CCD_Out_flag;     //CCD�������������־λ
u8    CCD_Cstart_flag;  //CCD���Ƽ��㺯�������־������main����ķ���������

u8  Stop_flag;
u8  Roadblock_flag;   //�ϰ���־λ
u8  Roadblock_count; 
u8  Roadblock_key;
u8  Roadblock_Intime_flag;   //·���ع��־λ
u8  way_flag;  //ֱ�� 2,  ���� 1��  ���� 3
u8  LittleS_flag;

u8  Can_Stop_flag;      //����ͣ����־λ
s16  Started_encoder;       //��������������
s16  Stop_encoder;       //ͣ����ʱ����


u8  CCD_c_Ga_flag;  //�������ʮ����CCD����ֵ
u8  CCD_Add_count;  //�ӵĴ���
u32 CCD_shi;
u8  CCD_count_start_flag;
u8  Corss_last_flag;    //��һ��Ϊʮ�����־λ

void Get_Car_Middle(void);
void CCD_Control(void);
void CCD_ControlOutput(void);



//------------------------------------------------------------------------------
CARSUB_EXT	int g_nCarControlFlag;
#define CAR_CONTROL_SET					g_nCarControlFlag = 1
#define CAR_CONTROL_CLEAR				g_nCarControlFlag = 0
#define IF_CAR_CONTROL					(g_nCarControlFlag)
//void WaitCarStand(void);
//void CheckCarStand(void);

//CARSUB_EXT int g_nWaitCarStandCount;
//#define WAIT_CAR_STAND_COUNT			1


#define CAR_FAILURE_ANGLE_MAX			60.0
#define CAR_FAILURE_ANGLE_MIN			-60.0

#define CAR_STAND_ANGLE_MAX				10
#define CAR_STAND_ANGLE_MIN				-10.0


//u8  CarFall_flag1;
//u8  CarFall_flag2;
//u8  CarFall_flag3;
//------------------------------------------------------------------------------
void Stand_Check(void);

void CarControlStart(void);
void Car_Stop(void);


//------------------------------------------------------------------------------
// 		TEST SPEED VALUE
// Notes: This global variable is only used for test purpos
//CARSUB_EXT float g_fTestSpeedValue;

//==============================================================================
//             END OF THE FILE : CARSUB.H
//------------------------------------------------------------------------------
#endif //
