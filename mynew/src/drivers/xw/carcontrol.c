
#define CARSUB_GLOBALS        1                       // Define the global variables
#include "carcontrol.h"
#include "common.h"
#include "include.h"


        
//float  g_fCarSpeedSet;
        
//u8 gu8_StarttimeCount;   
/*
void ArgumentInit(void)       //还未包含方向控制的pid参数
{
	AngleControlP	  = ANGLE_CONTROL_P_DEFAULT;
	AngleControlD	  = ANGLE_CONTROL_D_DEFAULT;
	g_fSpeedControlP	  = SPEED_CONTROL_P_DEFAULT;
	g_fSpeedControlI          = SPEED_CONTROL_I_DEFAULT;

	g_fDeadVoltage		  = DEAD_VOLTAGE_DEFAULT;
	
	g_fCarSpeedSet		  = CAR_SPEED_SET_DEFAULT;
        
	//g_nPad					= 0;
	//g_nInitFlag			= INIT_FLAG_DEFAULT;
        
   //     gu8_fStarttimeCount         = start_time_count;
}

*/

void  Discuss_Init(void)
{
    AngleControlP = Angle_default_P;
    AngleControlD = Angle_default_D;
    
    
    if(GPIO_GET_1bit(PORTD,6)==1)  g_Gravity_offset  = -1200;//第二电池   1370    位置在1300左右用1200
    else      g_Gravity_offset  = -1150;  //第一个电池    //1200  1150    位置在 1250左右
    
    SpeedControlP = Speed_default_P;
    SpeedControlI = Speed_default_I;
    SpeedControlD = Speed_default_D;
    
    Dir_mControl_P  = DirectionControl_default_P;
    Dir_mControl_I  = DirectionControl_default_I;
    Dir_mControl_D  = DirectionControl_default_D;
}

//==============================================================================
//				CAR CONTROL SUBROUTINES
//------------------------------------------------------------------------------


void CarSubInit(void) 
{
	//--------------------------------------------------------------------------
	// Initialize the Global Variable
	g_n1MSEventCount = 0;     //中断内的分时片段用
            
        gu8_StarttimeCount  = 40;     //静止 时间的计数量，以100ms为单位
        start_time_flag = 0;   
        
        g_fCarSpeedSet		  = 0.0;    //初始化速度为0，拨码开关设定的速度，等静止时间到时，装载到g_fCarSpeed_defult

	g_fCarSpeed_defult  = 0.0;        //跑时设定的速度
	
	//--------------------------------------------------------------------------
	g_nLeftMotorPulseSigma = g_nRightMotorPulseSigma = 0;
	g_fCarSpeed = 0.0;
	
	g_fLeftMotorOut = g_fRightMotorOut = 0;
	g_fAngleControlOut = g_fSpeedControlOut = g_fDirectionControlOut = 0.0;
        
        g_fSpeed_OutValue = g_fDir_OutValue = 0.0;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew = 0;
	g_nSpeedControlCount = g_nSpeedControlPeriod = 0;
	g_fSpeedControlIntegral = 0;

        VOLTAGE_GRAVITY  = 0;       //直立控制参数初始化
        VOLTAGE_GYRO = 0;
        g_s16GYROSCOPE1_OFFSET  = 0;
	g_fCarAngle = 0;
	g_fGyroscopeAngleSpeed = 0;
	g_fGravityAngle = 0;
        g_fGyroscopeAngleIntegral = 0;
        //Angle_err_Integral  = 0;
	
	//--------------------------------------------------------------------------
	g_fDirectionControlOutNew = g_fDirectionControlOutOld = 0;
	g_nDirectionControlCount = g_nDirectionControlPeriod = 0;
        
        Speed_Out_flag  = CCD_Out_flag  = 0;  //拿到标志位，才开始在直立函数里面输出其控制的计算量
	
        fleft_Correction  = 0.0025;      //十字弯左边修正量
        
        CCD_err_Integral  = 0;
        CCD_Err = 0;   
        
        CCD_Cstart_flag = 0;
	//--------------------------------------------------------------------------
        
        /* 128个像素点的平均AD值 */
        PixelAverageValue = 0;
        
        /* 设定目标平均AD值 */          //17，20, 24, 28
     //   TargetPixelAverageValue = 142;   //87,102,123,142

        /* 设定目标平均电压值与实际值的偏差，实际电压的10倍 */
     //   PixelAverageVoltageError = 0;

        /* 设定目标平均AD值允许的偏差 */
    //    TargetPixelAverageVoltageAllowError = 10;
        
        /* 曝光时间，单位ms */
        IntegrationTime = 15;   //6ms  15
       
        integration_piont = 21;
        TimerCnt5ms = 0;
        TimerCnt36ms  =0;
        send_data_cnt = 0;
        
     //   CarFall_flag1 = 0;
    //    CarFall_flag2 = 0;
     //   CarFall_flag3 = 0;
        Stop_flag = 0;    //检测到起跑线，停车标志位
        Roadblock_flag  =0;
        way_flag = 0;    //初始化为直道
        Can_Stop_flag=0;        //可以停车标志，用于第二次检测黑线
        Started_encoder=0;         //发车编码器计数
        
        Stop_encoder  = 0;
        CarSpeed_data = 0;      //两个编码器的平均值
        
        //CCDgate_update=0;
        CCD_Gate_Vir=110;
        
        CCD_C_Can_flag  = 0;
        CCD_c_Ga_flag = 0;
        CCD_Add_count=0;
        CCD50total  = CCDtotal  = 0;
        CCD_count_start = 0;
        CCD_count_start_flag  =0;
        Corss_last_flag  = 0;
       // Speednew  = Speedpre  =0;
        slope_flag  = 0;
        slope_key=0;
        CCD_NRF_flag=0;
        ImageCCD_flag=0;
        Roadb_DownSpe = 0;
        Roadblock_key = 0;
        Roadblock_count=0;
        Roadblock_Intime_flag = 0;
        LittleS_flag = 0;
}

void Speed_add_init(void)
{
  //u8 D4=0,D6=0;
  gpio_init(PORTD,4,GPI,0);
  gpio_init(PORTD,7,GPI,0);
  
  if((GPIO_GET_1bit(PORTD,7)==0)&&(GPIO_GET_1bit(PORTD,4)==0))         Speed_Add_Value = 1.5;
  else if((GPIO_GET_1bit(PORTD,4)==0)&&(GPIO_GET_1bit(PORTD,7)==1))    Speed_Add_Value = 2.0;
  else if((GPIO_GET_1bit(PORTD,4)==1)&&(GPIO_GET_1bit(PORTD,7)==0))    Speed_Add_Value = 4.0;
  else if((GPIO_GET_1bit(PORTD,7)==1)&&(GPIO_GET_1bit(PORTD,4)==1))    Speed_Add_Value = 6.0;
  
}

 
//------------------------------------------------------------------------------
//						MOTOR SPEED CONTROL
//
// 
void  Barrier_init(void)
{
  gpio_init(PORTE,24,GPI,0);
  gpio_init(PORTE,25,GPI,0);
  gpio_init(PORTE,26,GPI,0);
}

u8  Get_Barrier(void)
{
    u8  B24,B25,B26;
     B24  = GPIO_Get(PORTE,24);
     B25  = GPIO_Get(PORTE,25);
     B26  = GPIO_Get(PORTE,25);
     if(B24==1&&B26==1&&B25==0) return  1;
     else return  0;
}

void GetMotorPulse(void) 
{      
        u16 LeftMotorPulse ,RightMotorPulse;	

	RightMotorPulse = (u16)DMA_count_get(DMA_CH4);     //C0,右边
        DMA_count_reset(DMA_CH4);
	LeftMotorPulse  = (u16)DMA_count_get(DMA_CH5);     //D0，左边
        DMA_count_reset(DMA_CH5);
        
	if(g_fLeftMotorOut  < 0)		LeftMotorPulse  = -LeftMotorPulse;
	if(g_fRightMotorOut < 0)		RightMotorPulse = -RightMotorPulse;
		
	g_nLeftMotorPulseSigma  += LeftMotorPulse;
	g_nRightMotorPulseSigma += RightMotorPulse;
}
//void  Get_AccSpeed(void)
//{

//}


//------------------------------------------------------------------------------
void SetMotorVoltage(float fLeftVoltage, float fRightVoltage) {
                                                // Voltage : > 0 : Move forward;
                                                //           < 0 : Move backward
	s16 nOutL,nOutR;
	float fre;
        fre = 1000.0;
	//--------------------------------------------------------------------------
	if(fLeftVoltage > 1.0) 			fLeftVoltage = 1.0;
	else if(fLeftVoltage < -1.0) 	        fLeftVoltage = -1.0;
	
	if(fRightVoltage > 1.0) 		fRightVoltage = 1.0;
	else if(fRightVoltage < -1.0)	        fRightVoltage = -1.0;
                                              
	//--------------------------------------------------------------------------	
        
        if(fRightVoltage > 0) 
        {
                FTM_PWM_Duty(FTM0, CH1,  1000);
		nOutR = (u16)(fRightVoltage *fre);     //1000     右    0_2前   0_1 后
                
		FTM_PWM_Duty(FTM0, CH2, 1000  -  nOutR);
	} 
        else 
        {
                FTM_PWM_Duty(FTM0, CH2, 1000);
		fRightVoltage = -fRightVoltage;
		nOutR = (u16)(fRightVoltage *fre);
               
		FTM_PWM_Duty(FTM0, CH1,  1000 - nOutR);
	}   
        //---------------------------------------------------------------------------
        
	if(fLeftVoltage > 0)        //左    2_0 前      2_1后
        {                  
                FTM_PWM_Duty(FTM2, CH1,  1000);
                nOutL = (u16)(fLeftVoltage*fre);
              
		FTM_PWM_Duty(FTM2, CH0, 1000  -  nOutL);
	} 
        else 
        {
                FTM_PWM_Duty(FTM2, CH0, 1000);
		fLeftVoltage = -fLeftVoltage;
		nOutL = (u16)(fLeftVoltage * fre);
             
		FTM_PWM_Duty(FTM2, CH1,  1000- nOutL);
	}                                     

	
}                                            


//------------------------------------------------------------------------------
//						MOTOR SPEED CONTROL OUTPUT
// 
void MotorSpeedOut(void) {
	float fLeftVal, fRightVal;
	
	fLeftVal  = g_fLeftMotorOut;
	fRightVal = g_fRightMotorOut;
        
	if(fLeftVal > 0) 	        fLeftVal += MOTOR_OUT_DEAD_VAL;
	else if(fLeftVal < 0) 		fLeftVal -= MOTOR_OUT_DEAD_VAL;
	
	if(fRightVal > 0)		fRightVal += MOTOR_OUT_DEAD_VAL;
	else if(fRightVal < 0)		fRightVal -= MOTOR_OUT_DEAD_VAL;
		
	if(fLeftVal > MOTOR_OUT_MAX)	fLeftVal = MOTOR_OUT_MAX;
	if(fLeftVal < MOTOR_OUT_MIN)	fLeftVal = MOTOR_OUT_MIN;
	if(fRightVal > MOTOR_OUT_MAX)	fRightVal = MOTOR_OUT_MAX;
	if(fRightVal < MOTOR_OUT_MIN)	fRightVal = MOTOR_OUT_MIN;
			
	SetMotorVoltage(fLeftVal, fRightVal);
}
		
//------------------------------------------------------------------------------
void MotorOutput(void) {
	float fLeft, fRight;

	fLeft  = g_fAngleControlOut - g_fSpeedControlOut - g_fDirectionControlOut;
	fRight = g_fAngleControlOut - g_fSpeedControlOut + g_fDirectionControlOut;
	
      /*  if(Corss_deal_Flag  == 1)   //十字弯左边加修正量
        {
            if(fLeft>0){
            fLeft += fleft_Correction; 
            }
            else{
            fLeft -= fleft_Correction; 
            }
        }*/
	if(fLeft > MOTOR_OUT_MAX)		fLeft = MOTOR_OUT_MAX;
	if(fLeft < MOTOR_OUT_MIN)		fLeft = MOTOR_OUT_MIN;
	if(fRight > MOTOR_OUT_MAX)		fRight = MOTOR_OUT_MAX;
	if(fRight < MOTOR_OUT_MIN)		fRight = MOTOR_OUT_MIN;
		
	g_fLeftMotorOut  = fLeft;
	g_fRightMotorOut = fRight;
	MotorSpeedOut();
}

//------------------------------------------------------------------------------
void SpeedControl(void) {
	float fP, fData,fDelta;
	float fI,fD;
        
	//--------------------------------------------------------------------
        Speederr_pre  = Speederr_new;
        
	//--------------------------------------------------------------------------
	CarSpeed_data = (g_nLeftMotorPulseSigma + g_nRightMotorPulseSigma) / 2;
        
        if(g_nLeftMotorPulseSigma >(g_nRightMotorPulseSigma  +50))   way_flag = 3;   //往右走   //45
        else if(g_nLeftMotorPulseSigma< (g_nRightMotorPulseSigma -50))  way_flag = 1;    //往左走
        else if (g_nLeftMotorPulseSigma< (g_nRightMotorPulseSigma +30) && g_nRightMotorPulseSigma < (g_nLeftMotorPulseSigma +30) )  way_flag = 2; 
        else way_flag = 4; 
        
	g_nLeftMotorPulseSigma = g_nRightMotorPulseSigma = 0;
	g_fCarSpeed = CarSpeed_data * 0.061;    // CAR_SPEED_CONSTANT;     1000/(5*16)/200

     //   if(start_time_flag )
      //  {
           if(Roadb_DownSpe)   {
            //  g_fCarSpeed_defult-=0.5;
              Roadb_DownSpe = 0;
           }
           else if(way_flag == 2) 
            {
                    //way_flag = 0 ;
                    if(Roadblock_flag)  {
                      Roadblock_flag = 0;
                      Roadb_DownSpe = 1;
                      g_fCarSpeed_defult=1.0;   GPIO_SET(PORTA,15,0);
                    //  g_fCarSpeed_defult-=5.0;   GPIO_SET(PORTA,15,0);
                    //  if(g_fCarSpeed_defult < g_fCarSpeedSet-3.0)   g_fCarSpeed_defult = g_fCarSpeedSet-3.0;
                      
                    }
                    else if(!Stop_flag && start_time_flag){
                        g_fCarSpeed_defult  +=0.5;     //0.4
                        if(g_fCarSpeed_defult > g_fCarSpeedSet + Speed_Add_Value )  g_fCarSpeed_defult = g_fCarSpeedSet + Speed_Add_Value;  
                    }
            }
            else if( !Stop_flag && start_time_flag  && way_flag == 1 || way_flag == 3) {
              g_fCarSpeed_defult-=0.5;    // 0.5
              if(g_fCarSpeed_defult < g_fCarSpeedSet) g_fCarSpeed_defult = g_fCarSpeedSet;
            }
      //  }
	//--------------------------------------------------------------------------	

	//--------------------------------------------------------------------------
	fData  = CAR_SPEED_SET; 
	fDelta = fData - g_fCarSpeed;
        Speederr_new  = fDelta;
        SpeederrD  = Speederr_new  - Speederr_pre;
        
        fD = SpeederrD * SPEED_CONTROL_D;
	fP = fDelta * SPEED_CONTROL_P;
	fI = fDelta * SPEED_CONTROL_I;
	g_fSpeedControlIntegral += fI;
	
        
	if(g_fSpeedControlIntegral > SPEED_CONTROL_OUT_MAX)	    //积分饱和
		g_fSpeedControlIntegral = SPEED_CONTROL_OUT_MAX;
	if(g_fSpeedControlIntegral < SPEED_CONTROL_OUT_MIN)  	
		g_fSpeedControlIntegral = SPEED_CONTROL_OUT_MIN;
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;

      //  if(Roadblock_flag)   g_fSpeedControlOutNew = g_fSpeedControlOutOld;
      //  else   
          g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral  + fD;
        g_fSpeed_OutValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
       // Speed_Out_flag  = 1;

}

//------------------------------------------------------------------------------
//   
void SpeedControlOutput(void) 
{
	g_fSpeedControlOut = g_fSpeed_OutValue * ((float)g_nSpeedControlPeriod)  / 204.0 + g_fSpeedControlOutOld;
}

//==============================================================================
//				INPUT GRAVITY ACCELERATION AND GYROSCOPE 
//------------------------------------------------------------------------------
void  AngleCalculate(float VOLTAGE_GRAVITY,float VOLTAGE_GYRO) //用浮点数计算要5.5us一次，两次11us
{
	float DeltaValue;
      // float DeltaAngle;
	//--------------------------------------------------------------------------

	g_fGravityAngle = (float)(VOLTAGE_GRAVITY - GRAVITY_OFFSET)* GRAVITY_ANGLE_RATIO;
      //  DeltaAngle  = asin(VOLTAGE_GRAVITY/4098.0);
      //  g_fGravityAngle = 57.296*DeltaAngle ;
          
	g_fGyroscopeAngleSpeed = (float)(VOLTAGE_GYRO - g_s16GYROSCOPE1_OFFSET) * GYROSCOPE_ANGLE_RATIO;
	
        /////////////////////////////////////////////////
	g_fCarAngle = g_fGyroscopeAngleIntegral;
	DeltaValue = (g_fGravityAngle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;
	
	g_fGyroscopeAngleIntegral += (g_fGyroscopeAngleSpeed + DeltaValue ) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
     
       /////////////////////2选一/////////////////////////////// 
        
     //   bias_cf *=  0.0001;			//陀螺仪零飘低通滤波；500次均值；0.998 
	//bias_cf += g_fGyroscopeAngleSpeed*0.009;		   //0.002
	//angle_dot = g_fGyroscopeAngleSpeed  - bias_cf;		   
	//g_fCarAngle=(angle+angle_dot*dt)*0.975  + g_fGravityAngle*0.025;	
}


//-------------------------------------------------------
/*void complement_filter(float angle_m_cf,float gyro_m_cf)
{
	bias_cf*=0.0001;			//陀螺仪零飘低通滤波；500次均值；0.998 
	bias_cf+=gyro_m_cf*0.009;		   //0.002
	angle_dot=gyro_m_cf-bias_cf;		   
	angle=(angle+angle_dot*dt)*0.975+angle_m_cf*0.025;	
	//加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；0.90 0.05
}	
*/

void Kalman_Filter(float VOLTAGE_GRAVITY,float VOLTAGE_GYRO)			
{
	 //-------------------------------------------------------
	//float Q_angle=0.001, Q_gyro=0.003,  dt=0.005; //R_angle越大跟踪速度越快
				                      //注意：dt的取值为kalman滤波器采样时间;
        float dt=0.0048;  //0.005
        
        float Q_angle=0.003, Q_gyro=0.0003; 
       // float R_angle=250;
        
	float P[2][2] = {	{ 1, 0 },
				{ 0, 1 }
				};
		
	float Pdot[4] ={0,0,0,0};
	
	const char C_0 = 1;
	
	float  angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	//-------------------------------------------------------
        angle_m = (float)(VOLTAGE_GRAVITY - GRAVITY_OFFSET)* GRAVITY_ANGLE_RATIO;
          
	gyro_m = (float)(VOLTAGE_GYRO - g_s16GYROSCOPE1_OFFSET) * GYROSCOPE_ANGLE_RATIO;
        
        
	g_fCarAngle+=(gyro_m-q_bias) * dt;//先验估计
	
	Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' 先验估计误差协方差的微分
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;
	
	P[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	
	
	angle_err = angle_m - g_fCarAngle;//zk-先验估计
	
	
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//后验估计误差协方差
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	
	
	g_fCarAngle	+= K_0 * angle_err;//后验估计
	q_bias	+= K_1 * angle_err;//后验估计
	g_fGyroscopeAngleSpeed = gyro_m-q_bias;//输出值（后验估计）的微分 = 角速度
}

/*
static float angle,angle_dot; 		//外部需要引用的变量
//-------------------------------------------------------
 static float bias_cf;
static const float dt=0.01;
//-------------------------------------------------------
void complement_filter(float angle_m_cf,float gyro_m_cf)
{
	bias_cf*=0.0001;			//陀螺仪零飘低通滤波；500次均值；0.998 
	bias_cf+=gyro_m_cf*0.009;		   //0.002
	angle_dot=gyro_m_cf-bias_cf;		   
	angle=(angle+angle_dot*dt)*0.95+angle_m_cf*0.04;	
	//加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；0.90 0.05
}	
*/
/*	
//-----------------------------
//互补滤波
//-------------------------------------------------------
//static float angle,angle_dot; 		//外部需要引用的变量
//-------------------------------------------------------
 static float bias_cf;
static const float dt=0.01;
//-------------------------------------------------------
void complement_filter(float angle_m_cf,float gyro_m_cf)
{
	bias_cf*=0.0001;			//陀螺仪零飘低通滤波；500次均值；0.998 
	bias_cf+=gyro_m_cf*0.009;		   //0.002
	angle_dot=gyro_m_cf-bias_cf;		   
	angle=(angle+angle_dot*dt)*0.95+angle_m_cf*0.04;	
	//加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；0.90 0.05
}										
//static float acceler,gyro;	
void AD_calculate(void)
{


gyro=0.011557*VOLTAGE_GYRO-3.035+105*3.14/180;	//后面105*3.14/180是对角速度零点的修正
 
acceler=0.003052*VOLTAGE_GRAVITY-1.4375;		//计算角度时，只用了加速度传感器的Z轴就够了
									
acceler=acceler*1.5+14*3.14/180;  //没有使用arcsin（），因为sin在0度附近变化时可以用sin值代替角度值，这样可以减轻单片机的计算负担。
									//1.5是对角度的适当放大，14*3.14/180是对角度零点的修正
					  
   complement_filter(acceler,gyro);

}
*/


//------------------------------------------------------------------------------
void AngleControl(void) 
{
	float fValue; //,err;
        
        //err=  CAR_ANGLE_SET - g_fCarAngle;
	//Angle_err_Integral  +=  err;
        //--------------------------------------------------------------------------
        /*
        if(g_fCarAngle > 45.0 || g_fCarAngle < -45.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) *  0.09 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0012; // 
        
        else if(g_fCarAngle > 38.0 || g_fCarAngle < -38.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) *  0.1 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0015; //  0.315 0.0045
                                                        
        else if( g_fCarAngle > 30.0 || g_fCarAngle < -30.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) *0.13 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.002; //  0.295  0.00395
                                                  //0.28
        else if(g_fCarAngle > 20.0 || g_fCarAngle < -20.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.155 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00225;  // 0.275  0.00365
                                                      //0.265
        else if(g_fCarAngle > 15.0 || g_fCarAngle < -15.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.17 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0025;  // 0.26  0.0035
                                                        //0.255
        else if(g_fCarAngle > 10.0 || g_fCarAngle < -10.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.2  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0028  ;  //0.25  0,0034    
        else if(g_fCarAngle > 6.0 || g_fCarAngle < -6.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.21  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.003  ;  //0.24  0,0033
        
        else
           fValue = (CAR_ANGLE_SET - g_fCarAngle) * ANGLE_CONTROL_P + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * ANGLE_CONTROL_D;
           */
        /*
        if(g_fCarAngle > 45.0)
             fValue = (CAR_ANGLE_SET - g_fCarAngle) *  -0.17 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * -0.0023; // 
        else if (g_fCarAngle > 38.0)
	     fValue = (CAR_ANGLE_SET - g_fCarAngle) *  -0.205 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * -0.0030; //  0.315 0.0045
        else if (g_fCarAngle > 30.0)  
             fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.20 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0038; //  0.295  0.00395
        else if(g_fCarAngle > 20.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.15 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00355;  // 0.275  0.00365
                                                      //0.265
        else if(g_fCarAngle > 15.0 )
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.14 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0035;  // 0.26  0.0035
                                                        //0.255
        else if(g_fCarAngle > 10.0 )
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.17  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00345  ;  //0.25  0,0034    
        else if(g_fCarAngle > 6.0 )
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.215  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0034  ;  //0.24  0,0033
     //   else if(g_fCarAngle > 0.0 )//-----------------------------------------------------------------------------------------------------
        else if(g_fCarAngle < -45.0)
          fValue = (CAR_ANGLE_SET - g_fCarAngle) *  0.36 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0048; // 
        else if (g_fCarAngle < -38.0)
	  fValue = (CAR_ANGLE_SET - g_fCarAngle) *  0.325 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0043; //  0.315 0.0045
        else if (g_fCarAngle < -30.0)  
          fValue = (CAR_ANGLE_SET - g_fCarAngle) *0.30 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0041; //  0.295  0.00395
        else if(g_fCarAngle < -20.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.285 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.000395;  // 0.275  0.00365
                                                      //0.265
        else if(g_fCarAngle < -15.0 )
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.265 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.000355;  // 0.26  0.0035
                                                        //0.255
        else if(g_fCarAngle < -10.0 )
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.25  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00034  ;  //0.25  0,0034    
        else if(g_fCarAngle < -6.0 )
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.235  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00335  ;  //0.24  0,0033
        else       
               fValue = (CAR_ANGLE_SET - g_fCarAngle) * ANGLE_CONTROL_P + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * ANGLE_CONTROL_D;
        */
        
         //--------------------------------------------------------------------------
        if(g_fCarAngle > 45.0 || g_fCarAngle < -45.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) *  0.33 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.005; // 
        
        else if(g_fCarAngle > 38.0 || g_fCarAngle < -38.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) *  0.315 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0045; //  0.315 0.0045
        else if( g_fCarAngle > 30.0 || g_fCarAngle < -30.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) *0.295 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00395; //  0.295  0.00395
        
        else if(g_fCarAngle > 20.0 || g_fCarAngle < -20.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.27 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.00365;  // 0.275  0.00365
        
        else if(g_fCarAngle > 15.0 || g_fCarAngle < -15.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.265 + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0035;  // 0.26  0.0035
        
        else if(g_fCarAngle > 10.0 || g_fCarAngle < -10.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.255  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0034  ;  //0.25  0,0034    
        else if(g_fCarAngle > 6.0 || g_fCarAngle < -6.0)
              fValue = (CAR_ANGLE_SET - g_fCarAngle) * 0.245  + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * 0.0033  ;  //0.24  0,0033
        
        else
           fValue = (CAR_ANGLE_SET - g_fCarAngle) * ANGLE_CONTROL_P + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * ANGLE_CONTROL_D;
           
        
        
	if(fValue > ANGLE_CONTROL_OUT_MAX)			fValue = ANGLE_CONTROL_OUT_MAX;
	else if(fValue < ANGLE_CONTROL_OUT_MIN)          	fValue = ANGLE_CONTROL_OUT_MIN;
	g_fAngleControlOut = fValue;
}

/*
//------------------------------------------------------------------------------
void DirectionControl(void) {
	float fLeftRightAdd, fLeftRightSub, fValue;
	int nLeft, nRight;

	//--------------------------------------------------------------------------
	nLeft = (int)(g_fLeftVoltageSigma /= DIRECTION_CONTROL_COUNT);
	nRight = (int)(g_fRightVoltageSigma /= DIRECTION_CONTROL_COUNT);
	g_fLeftVoltageSigma = 0;
	g_fRightVoltageSigma = 0;

	fLeftRightAdd = nLeft + nRight;
	fLeftRightSub = nLeft - nRight;
	
	g_fDirectionControlOutOld = g_fDirectionControlOutNew;
	
	if(fLeftRightAdd < LEFT_RIGHT_MINIMUM) {
		g_fDirectionControlOutNew = 0;
	} else {
		fValue = fLeftRightSub * DIR_CONTROL_P / fLeftRightAdd;
		
		if(fValue > 0) fValue += DIRECTION_CONTROL_DEADVALUE;
		if(fValue < 0) fValue -= DIRECTION_CONTROL_DEADVALUE;
		
		if(fValue > DIRECTION_CONTROL_OUT_MAX) fValue = DIRECTION_CONTROL_OUT_MAX;
		if(fValue < DIRECTION_CONTROL_OUT_MIN) fValue = DIRECTION_CONTROL_OUT_MIN;
	        g_fDirectionControlOutNew = fValue;
	}
}

//------------------------------------------------------------------------------
void DirectionControlOutput(void) {
	float fValue;
	fValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld;
	g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / DIRECTION_CONTROL_PERIOD + g_fDirectionControlOutOld;
	
}
*/

//------------------------------------------------------------------------------

extern  u8  Pixel[128];
/*******************************************************************************
去掉了转向陀螺仪，感觉时间太长，直接用后向差分算偏向的变换率    2013.6.2，待调
，加了硬件均值，时间允许
问题：
      开机时会进入两边都没有黑线的情况？？


可以改进的地方：

      方向控制里面可以加一个方向误差判断，当小于某一个值时方向不控制。


*******************************************************************************/
//------------------------------------------------------------------------------
void CCD_Control(void)  
{
	u8	i,j;
	u8    Left_flag=0,Right_flag=0;
	float	fValue;
        s16   line_distance;
        s16     Dir_realAD;
        
        if(Corss_deal_Flag ) {    //清掉上次十字标志
           GPIO_SET(PORTA,15,1);
           Corss_deal_Flag  = 0;
           Corss_last_flag  = 1;
        }
        
        Dir_AD  = ad_xwonce(ADC1, SE4a, ADC_8bit); //采集模拟量的AD值,加八次硬件平均滤波    八次硬件均值滤波28.5us

        Dir_realAD  = Dir_AD  - g_s16GYRO_DirOFFSET;
        
       
        
        //--------------------------起跑检测------------------------------
       /* if  ((Pixel[63]-Pixel[38])>30)
        {
              if((Pixel[64]-Pixel[84])>30  )  {
                  if(Can_Stop_flag && way_flag == 2 )   Stop_flag  = 1;
              }
        }
        */
        
        //-----------方向识别部分--------
        for(j=63;j<126;j+=2)        //74号点坏了，不会检测到
        {
                if(Pixel[j] < CCD_GateValve)  
                {
                    if( ((j-1)==74) || ((j+1)==74) ) {
                        if(Pixel[72]<CCD_GateValve || Pixel[75]<CCD_GateValve )
                        {
                           Right_site = j;
                           Right_flag  = 1;  break;
                        }
                    }
                    else if(Pixel[j-1]<CCD_GateValve || Pixel[j+1]<CCD_GateValve )
                    {
                        Right_site = j;
                        Right_flag  = 1;  break;
                    }
                }
        }      
        for(i=63;i>2;i-=2)      //i不能是 i>=1,会卡死在这里
        {  
                if(Pixel[i] < CCD_GateValve)  
                {
                    if(Pixel[i-1]<CCD_GateValve || Pixel[i+1]<CCD_GateValve )
                    {
                        Left_site  = i; 
                        Left_flag  = 1; break;
                    }
                }
        }
//-----------路障检测段---------------------------------------------------------        
    /*    if(Roadblock_key){
            if( (Left_site  ==63) &&(Right_site  ==63 ) && (Pixel[43]<115) && (Pixel[83]<115) )  // ||
             //  (Pixel[35]>(Pixel[7]-20) && Pixel[65] > (Pixel[35]-20) && Pixel[80]>(Pixel[65]-20) &&Pixel[120]>(Pixel[80]-20)))       //全黑
            {
                Roadblock_count++;
                if(Roadblock_count>=2){
                   Roadblock_count=0;
                   Roadblock_flag = 1; 
                }
            }
            else  Roadblock_count=0;
            
        }*/
//-----------方向处理部分-------------------------------------------------------
	if(Right_flag && Left_flag)     //  Right_flag  = 1)      //是否看到两条线，四种情况
	{
              line_distance = Right_site - Left_site; 
              if(!line_distance && Roadblock_key){                             //全黑 ，路障
                    if( (Pixel[43]<115) && (Pixel[83]<115) && (Pixel[23]<115) && (Pixel[103]<115)  ){
                      Roadblock_count++;
                      Roadblock_Intime_flag = 1;
                      if(Roadblock_count>=2){
                         Roadblock_count=0;
                         Roadblock_flag = 1; 
                         GPIO_SET(PORTA,15,0);
                        }}
                     else  {
                      Roadblock_Intime_flag = 0;
                      Roadblock_count=0;
                    }
              }
              else if(line_distance<40){
                    if ( ((Pixel[63]-Pixel[40])>30) && ((Pixel[64]-Pixel[85])>30) )
                    {
                        Line_Middle_site  =63.5;
                        if(Can_Stop_flag && way_flag == 2)   {
                          Stop_flag  = 1;     //停车 检测
                          
                        }
                    }
                    else if(Can_Stop_flag && way_flag == 2 )    Stop_flag  = 1;
              }
              else if((line_distance> 60 ) && (line_distance< 125 ))       //坡道上看到80个点左右，平时更多点
              {
                    LeftRight_distance  = (float)line_distance/2.0;  //两条黑线的一半
                    Right_flag = Left_flag  =0;
                    Line_Middle_site = LeftRight_distance +Left_site;      //算的黑线中线位置
              }
              else{
                   Line_Middle_site  =63.5;
              }       
        }
        
        else if(!Right_flag && !Left_flag)       //未看到线，假设十字弯
        {
             if(CCDtotal  > CCD_shi)   
             {     if( way_flag == 1 || way_flag == 3)
                   {
                     LittleS_flag = 1;
                   }
                   else
                   {
                      Corss_deal_Flag  = 1;
                      GPIO_SET(PORTA,15,0);
                   }
             }
          
        }
        else if(Right_flag && !Left_flag)  //仅看到右线
        {
              Right_flag =0;
              Line_Middle_site = Right_site - LeftRight_distance +1;  //+1
        }
        else if(!Right_flag && Left_flag)   //仅看到左线
        {
              Left_flag =0;
              Line_Middle_site = Left_site + LeftRight_distance -1; //-1
        }
        else  Line_Middle_site  =63.5;
    
        //-------------------------------------------------方向处理部分
        CCD_Err_pre = CCD_Err;
        //--误差计算部分--------------------------------------------------------
        if( LittleS_flag  || Corss_deal_Flag  || Corss_last_flag  ||  Roadblock_flag  || Roadblock_Intime_flag )   //处理十字弯，障碍 及障碍预判断
        {
          //方向输出不更新，即输出上一次的方向值
             CCD_Err = 0;     
        }
        else
        {
             CCD_Err =  Car_Middle_site  - Line_Middle_site;   //      63.5;         //
        }
        //--------------------------------------------------------
	g_fDirectionControlOutOld = g_fDirectionControlOutNew;
        
        CCD_Err_Dif= CCD_Err - CCD_Err_pre ;
      //  if(Corss_deal_Flag    ||  Roadblock_flag  || Roadblock_Intime_flag){}
      //  else if(CCD_Err_Dif>40 || CCD_Err_Dif<-40)  CCD_Err = 0;
        
        //---------CCD误差处理部分----------------------------------------------
        if(CCD_Err  == 0)       //以下两种都当成直道
        {
                fValue  = 0.0;
                //straightaway_flag = 1;
        } 
        else if(CCD_Err < 2.0 && CCD_Err > -2.0)
        {
                CCD_err_Integral  += CCD_Err;
                
                if(CCD_err_Integral>5500)  CCD_err_Integral  = 5500;
                else if(CCD_err_Integral<-5500)  CCD_err_Integral= -5500;
                
                CCD_Err = 0;
                fValue = CCD_err_Integral * DIR_CONTROL_I  - Dir_realAD *DIR_CONTROL_D;   
                //straightaway_flag = 1;
        }
        /**/
        
         else  if(CCD_Err > 50.0  ||  CCD_Err < -50.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.002  -  Dir_realAD *DIR_CONTROL_D; 
                   }
                else  fValue = CCD_Err * 0.008  -  Dir_realAD *0.0033;    //0.009,0.0038
        }
         else  if(CCD_Err > 45.0  ||  CCD_Err < -45.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.002  -  Dir_realAD *DIR_CONTROL_D; 
                   }
                else  fValue = CCD_Err * 0.01  -  Dir_realAD *0.0035;    //0.009,0.0038
        }
        
         else  if(CCD_Err > 40.0  ||  CCD_Err < -40.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.002  -  Dir_realAD *DIR_CONTROL_D; 
                   }
                else  fValue = CCD_Err * 0.01  -  Dir_realAD *0.0032;    //0.0115,0.004
        }
        
        else  if(CCD_Err > 35.0  ||  CCD_Err < -35.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.0022  -  Dir_realAD *DIR_CONTROL_D; 
                   }
                else  fValue = CCD_Err * 0.0115  -  Dir_realAD *0.0034;    //0.013,0.0043
        }
        
        else  if(CCD_Err > 30.0  ||  CCD_Err < -30.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.0025  -  Dir_realAD *DIR_CONTROL_D; //0.15
                   }
                else  fValue = CCD_Err * 0.0125  -  Dir_realAD *0.0036;    //0.016 ,0.0050
        }
        
         else  if(CCD_Err > 25.0  ||  CCD_Err < -25.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.003  -  Dir_realAD *DIR_CONTROL_D; //0.15
                   }
                else  fValue = CCD_Err * 0.0145  -  Dir_realAD *0.0041;    //0.0135,0.0038
        }
         else  if(CCD_Err > 20.0  ||  CCD_Err < -20.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.004  -  Dir_realAD *DIR_CONTROL_D; //0.15
                   }
                else  fValue = CCD_Err * 0.0155  -  Dir_realAD *0.0038; // 0.0155,0.0038
        }
        
        else  if(CCD_Err > 15.0  ||  CCD_Err < -15.0) 
        {
                if( Corss_last_flag  == 1)   { //十字出弯
                    Corss_last_flag  = 0;
                    fValue = CCD_Err * 0.005  -  Dir_realAD *DIR_CONTROL_D; //0.15
                   }
                else  fValue = CCD_Err * 0.016  -  Dir_realAD *0.00385;   //0.015,0.0039
        }
        else  if(CCD_Err > 10.0  ||  CCD_Err < -10.0) 
        {       
              if(Corss_last_flag  == 1)   { //十字出弯
                Corss_last_flag  = 0;
                fValue = CCD_Err * 0.005  -  Dir_realAD *DIR_CONTROL_D; //0.15
              }
                 else   fValue = CCD_Err * 0.0165  -  Dir_realAD *0.0039;     //0.016,0.0038
        }
        else
        {
                CCD_err_Integral  += CCD_Err;
                
                if(CCD_err_Integral>5500)  CCD_err_Integral  = 5500;
                else if(CCD_err_Integral<-5500)  CCD_err_Integral= -5500;
                
                fValue = CCD_Err * DIR_CONTROL_P  + CCD_err_Integral * DIR_CONTROL_I    //丢线用积分找回位置
                       - Dir_realAD *DIR_CONTROL_D;
        }
	  
	if(fValue > DIRECTION_CONTROL_OUT_MAX) fValue = DIRECTION_CONTROL_OUT_MAX;
	if(fValue < DIRECTION_CONTROL_OUT_MIN) fValue = DIRECTION_CONTROL_OUT_MIN;
		
	g_fDirectionControlOutNew = fValue;
        g_fDir_OutValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld;
        
        
         //-----------------------------------------------------------------------------------------
        CCD_GateValve_pre = CCD_GateValve;
        
        if( Corss_deal_Flag || Roadblock_Intime_flag || Roadblock_flag){    //路障不更新阀值
        }
        else{
            
            CCD_Gate_Vir = (s16)PixelAverageValue - 25;
            if(CCD_Gate_Vir>140) CCD_Gate_Vir=140;          //110,75
            else if(CCD_Gate_Vir<85) CCD_Gate_Vir=85;       //115,75
                                                             //135,85
                                                              //140,85  ok
            CCD_GateValve = CCD_GateValve_pre + ((CCD_Gate_Vir - CCD_GateValve_pre)/3);
        //  CCDGate_PN_Diff = (CCD_Gate_Vir - CCD_GateValve_pre)/3;
        //  CCD_GateValve = (s16)PixelAverageValue - 25;    //还是要每次根据CCD的AD值更新阀值   //30   //25
        }
        //--------------------阀值限幅---------------------
        if(CCD_GateValve>140) CCD_GateValve=140;          //110,75
        else if(CCD_GateValve<85) CCD_GateValve=85;       //115,75
                                                          //120,85
        
        
        if( Corss_last_flag )       Corss_last_flag  = 0;    //清除上一次十字弯标志位

}

void CCD_ControlOutput(void) 
{
	g_fDirectionControlOut = g_fDir_OutValue * ((float) g_nDirectionControlPeriod )  / 36.0 + g_fDirectionControlOutOld;
}




void Get_Car_Middle(void) 
{
	u8	i,j;
        u8      Right_line,Left_Line;
        float   fvalue;
        for(j=64;j<126;j+=2)
	{
		if(Pixel[j] < CCD_GateValve)  
                {
                  
                    if(Pixel[j-1]<CCD_GateValve || Pixel[j+1]<CCD_GateValve )
                    {
                        Right_line = j;
                        break;
                    }
                }
	}


	for(i=63;i>2;i-=2)
	{
          
          if(Pixel[i] < CCD_GateValve)  
                {
                  
                    if(Pixel[i-1]<CCD_GateValve || Pixel[i+1]<CCD_GateValve )
                    {
                        Left_Line = i;
                        break;
                    }
                }
	}
        
        fvalue = (float)(Right_line  - Left_Line)/2.0 + (float)Left_Line;
        
        if(fvalue>55  &&  fvalue<72)
        {
            Car_Middle_site = fvalue;
        }
        else
        {
            Car_Middle_site = 63.5;
        }
        
}
        


//------------------------------------------------------------------------------

void CarControlStart(void) {
	CAR_CONTROL_SET;
	ANGLE_CONTROL_START;
	SPEED_CONTROL_START;
	DIRECTION_CONTROL_START;
	TIME_TEST_ENA;
}

//------------------------------------------------------------------------------
void Car_Stop(void) 
{
         DisableInterrupts;     //禁止总中断
        
         FTM_PWM_Duty(FTM0, CH1,  1000);
         FTM_PWM_Duty(FTM0, CH2,  1000);
         FTM_PWM_Duty(FTM2, CH0,  1000);
         FTM_PWM_Duty(FTM2, CH1,  1000);
            
   /*     FTM_PWM_init(FTM0,CH1,5000,1000);        //A4     右  前行  ok  
        FTM_PWM_init(FTM0,CH2,5000,1000);        //A5     右  后行  ok
         
        FTM_PWM_init(FTM2,CH0,5000,1000);        //B18    左  前行  ok    
        FTM_PWM_init(FTM2,CH1,5000,1000);        //B19    左  后行  ok
	*/
	g_fLeftMotorOut = g_fRightMotorOut = 0;
	g_fAngleControlOut = 0;
	g_fSpeedControlOut = 0;
	g_fDirectionControlOut = 0;
        
}

//------------------------------------------------------------------------------
/*
void WaitCarStand(void) 
{
	if(g_nCarControlFlag == 1) return;
	if(g_nWaitCarStandCount < WAIT_CAR_STAND_COUNT) 
        {
		g_nWaitCarStandCount ++;
		return;
	}
	
	
	if(g_fCarAngle > CAR_STAND_ANGLE_MIN &&
	   g_fCarAngle < CAR_STAND_ANGLE_MAX &&
	   g_fGravityAngle > CAR_STAND_ANGLE_MIN &&
	   g_fGravityAngle < CAR_STAND_ANGLE_MAX) 
        {
	   	CarControlStart();
	} 	
}
*/
//------------------------------------------------------------------------------	
void Stand_Check(void) 
{
  
      
      //单纯的算出角度判断
	if(g_fCarAngle >= CAR_FAILURE_ANGLE_MAX || g_fCarAngle <= CAR_FAILURE_ANGLE_MIN ) 
          while(1)
          {
		Car_Stop();
              //  Led_Indicate();
          }
	
}


//==============================================================================
//                END OF THE FILE : CARSUB.C
//------------------------------------------------------------------------------
