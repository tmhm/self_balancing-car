#include "common.h"
#include "include.h"
#include "arm_math.h"
#include "isr.h"
//#include  "NRF24L0.h"     //无线模块


extern s8 IntegrationTime ;
extern void StartIntegration(void);
extern  u8 Pixel[128]; 

void PIT0_IRQHandler(void) 
{  

        PIT_Flag_Clear(PIT0); 
  
      //  gpio_set (PORTE, 25, 0);
        g_n1MSEventCount ++;
        
	Stand_Check();      //倒车保护

        //--------------------------------------------------------------------------
        TimerCnt36ms++;     
        if(TimerCnt36ms  >=36)  TimerCnt36ms  =0;     //留4ms出来给取AD
        //------------------------------------------------------------------------------------------------------------------------------      
        //放在计算曝光时间函数里面了 integration_piont = 10 - IntegrationTime;     //!!曝光时间的函数永远不会和取AD的函数同时运行
       // if(integration_piont >= 2)                                                //因为曝光一定会在10ms前的某一个时刻
       // {      /* 曝光时间小于2则不进行再曝光 */
        if(integration_piont == TimerCnt36ms)    
        {
          StartIntegration();    //曝光段50us     
        }
        //}  
         //------------------------------------------------------------------------------------------------------------------------------     
        
        //下面为中断内分时控制，每段最多只能用750us。
 	//--------------------------------------------------------------------------
  	if(g_n1MSEventCount >= CONTROL_PERIOD)      //与取AD值共用一个1ms周期     //此处60us，可以看成进出中断压栈时间
        {  //=====CCD采样数据======================          //此段共840us左右
            g_n1MSEventCount = 0;  
            TimerCnt5ms ++;
            if(TimerCnt5ms>=3)      //3.7ms
            {// gpio_set (PORTE, 25, 0);   
              TimerCnt5ms=0;
              ImageCCD_flag = 1;
              
         //     ImageCapture(Pixel);      //3.7ms的时间取128个CCD的AD值,
         //     CCD_Cstart_flag  = 1;     //采样CCD的AD之后，在mian计算控制量
         //     CalculateIntegrationTime();    //30us   
         //     send_data_cnt++;    //用来发送CCD上位机的数据，计数   
             //  gpio_set (PORTE, 25, 1);   
            }
  	}  
   //     else  if(g_n1MSEventCount == 1)    // CCD control,有可能是算的上次的值，跟main里面的取值不同步，已解决
   //     {                                 //方向控制只在这1ms内       ,放到main里面！2013.6.16      
        
  	//}
        else if(g_n1MSEventCount == 3)     //=====速度控制计算======================
        {      
              g_nSpeedControlCount ++;
              if(g_nSpeedControlCount >= SPEED_CONTROL_COUNT) 
              {
                 //--启动分段加速------------------------------------------------
                      gu8_StarttimeCount--;
                      if(start_time_flag == 0 && gu8_StarttimeCount==0)     
                      {                  
                            gu8_StarttimeCount  = 1;
                            if(g_fCarSpeed_defult <  g_fCarSpeedSet){
                                  g_fCarSpeed_defult  +=  0.3;    //0.5
                                  if(g_fCarSpeed_defult >=  g_fCarSpeedSet){
                                   start_time_flag = 1;     //速度已经加到给定的速度
                        }}}
                      
                      if(start_time_flag )  Started_encoder += CarSpeed_data;
                      if(Started_encoder>3000){        //发车6s后 开始检测起跑线，置位允许停车标志
                        Can_Stop_flag=1;
                        Started_encoder = 0;
                      }
              
                 //--停车处理---------------------------------------------------
                      if(Stop_flag )   //暂先不分段降速处理
                      {
                            Stop_encoder += CarSpeed_data;       //延时0.5s，冲过起跑线，再停车
                            if(Stop_encoder>950)    {g_fCarSpeed_defult  = 0.0;
                                 while(1)
                                  {
                                        Car_Stop();
                                      //  Led_Indicate();
                                  }
                            }
                      }
                      //else
                    //  {       //--路障及直道、弯道速度处理-----------------------------------
                     //         if((Roadblock_flag )) {
                        //               Roadblock_flag =0;
                                     //  if(g_fCarSpeed_defult>2.0)  g_fCarSpeed_defult  = 2.0;       //路障速度，貌似现在只能以10.0过路障！！！！！ 
                       
                    //  }
                
                   //   if((g_fCarSpeed_defult <  g_fCarSpeedSet-0.7)  &&(Stop_flag  == 0) &&(Can_Stop_flag == 1))  g_fCarSpeed_defult++;
                  //    if(g_fCarSpeed_defult <  g_fCarSpeedSet-0.7 && start_time_flag == 1)   g_fCarSpeed_defult+=0.5;    //障碍降速后加速
                      
                     
                            
                  //------------------------------------------------------------
                      SpeedControl();
                      g_nSpeedControlCount  = 0;
                      g_nSpeedControlPeriod = 0;   
                  //------------------------------------------------------
                      if(way_flag == 1){GPIO_SET(PORTC,16,1);GPIO_SET(PORTC,17,1);GPIO_SET(PORTC,18,0);}
                      else if(way_flag == 2){GPIO_SET(PORTC,16,0);GPIO_SET(PORTC,17,1);GPIO_SET(PORTC,18,1);}
                       else if(way_flag == 3){GPIO_SET(PORTC,16,1);GPIO_SET(PORTC,17,0);GPIO_SET(PORTC,18,1);}
                      else {GPIO_SET(PORTC,16,1);GPIO_SET(PORTC,17,1);GPIO_SET(PORTC,18,1);}
                       if(CCD_c_Ga_flag)
                      {
                          CCD_Add_count++;
                          CCD50total  +=CCDtotal;
                          if(CCD_Add_count>=8)   {
                            CCD_c_Ga_flag = 0;    //开始累加8次赛道CCD总量,100msyici
                            CCD_C_Can_flag  = 1;
                          }                       
                      }
                      
                      CCD_count_start++;
                      if((CCD_count_start>=30)&&(CCD_count_start_flag  == 0))  {
                        CCD_c_Ga_flag=1;
                        CCD_count_start_flag  = 1;
                      }   
                  }
  	} 
        else if(g_n1MSEventCount == 6)            //取得直立控制需要的角度及加速度    总共650us 左右    改完后250us左右
        {    //=====直立控制计算======================
             Get_mma8451_z();                         //8次平均，耗时451us    4次     2ci    
             ad1_XW_ave(ADC0, SE8, ADC_12bit,1);   //2次平均，每次8次硬件滤波   
             Kalman_Filter((float)VOLTAGE_GRAVITY , (float)VOLTAGE_GYRO);	//40us
  	} 
        else if(g_n1MSEventCount == 9)    //========直立控制及电机输出==========     
        {                 
              GetMotorPulse();
              AngleControl();		//18us
              //-------------------------------
          //    if(Speed_Out_flag == 1)
          //    {
                  g_nSpeedControlPeriod += 12;
                  SpeedControlOutput();
          //    }
              
            //  if(CCD_Out_flag == 1)
            //  {
                  g_nDirectionControlPeriod += 12;
                  CCD_ControlOutput(); 
            //  }
              //-------------------------------
              MotorOutput();                     
  	} 
  	    
    
        
      //  gpio_set (PORTE, 25, 1);
        
}



/*************************************************************************
*  函数名称：PIT1_IRQHandler
*  功能说明：PIT1定时中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-9-18    已测试
*  备    注：
*************************************************************************/

void  PIT1_IRQHandler(void) 
{  
    LED_TURN(LED0); //LED0反转 
    PIT_Flag_Clear(PIT1); //清中断标志位 
}


/**********************卡尔曼滤波，待用*****************************
  volatile float QingJiao = 0;
 volatile float Gyro_Data = 0;
 void Kalman(void)
 {
    float Q =1,R = 300;
    static float RealData = 0,RealData_P = 0;
    float NowData = 0,NowData_P = 0;
    float Kg = 0,gyroscope_rate = 0,accelerometer_angle=0;
    float Acc_x = 0,Acc_z = 0, Gyro = 0;
    static float his_acc = 0.0,his_accx = 0.0,his_accz = 0.0;
    
    while(!ATD0STAT0_SCF);   //等待转换完成
         
     Acc_x = (float)ATD0DR1;
     Acc_z = (float)ATD0DR2;
     Gyro  = (float)ATD0DR0;
     
   if(Gyro > 4090) Gyro += 1000;
    else
    if(Gyro > 4084) Gyro += 500;
    
    if(Gyro < 55)   Gyro -= 1000;
    else
    if(Gyro < 60)   Gyro -= 500;
        
    Acc_x = Acc_x - 2342.0;
     Acc_z = Acc_z - 2076.0;
     Gyro  = Gyro  - 2048.0;
     Gyro_Data = Gyro;
     
    OutData[0] = Gyro_Data;
     accelerometer_angle = atan2f(-Acc_x,Acc_z);
     OutData[1] = accelerometer_angle*1000;

     gyroscope_rate = Gyro*0.0023;         //参考电压3.3v 12位ADC 放大9.1倍 enc-03 0.67mv/deg./sec.
     
                                          //(3300/4096)/(0.67*9.1)*(3.14/180) =  0.0023
     
    NowData = RealData + gyroscope_rate*0.005;                 
     //1.预估计 X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
     NowData_P = sqrt(Q*Q+RealData_P*RealData_P);             
    //2.计算预估计协方差矩阵   P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k)
     Kg = sqrt(NowData_P*NowData_P/(NowData_P*NowData_P+R*R));
     //3.计算卡尔曼增益矩阵 K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k))
     RealData = NowData + Kg*(accelerometer_angle - NowData); 
    //4.更新估计 X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
     RealData_P = sqrt((1-Kg)*NowData_P*NowData_P);           
     //5.计算更新后估计协防差矩阵 P(k|k) =（I-K(k)*H(k)）*P(k|k-1)
     
    QingJiao =  RealData;  
     
    
    OutData[2] = QingJiao*1000; 
    OutPut_Data();
 }

********************以上为卡尔曼滤波*******************************/

/********************以下为互补滤波*******************************/

/*
static float acceler,gyro;	
static  float  AD_ENC180_result , IIC_MMA8451_result;
void AD_calculate(void)
{
    AD_ENC180_result    =   ad_once(ADC1, SE4a, ADC_12bit);         //读取 ADC1_SE4a ，16位精度,E0
    IIC_MMA8451_result  = Get_mma8451_z;
    gyro=0.011557*AD_ENC180_result-3.035+105*3.14/180;	//后面105*3.14/180是对角速度零点的修正
     
    acceler=0.003052*IIC_MMA8451_result-1.4375;		//计算角度时，只用了加速度传感器的Z轴就够了
                                                                            
    acceler=acceler*1.5+14*3.14/180;  //没有使用arcsin（），因为sin在0度附近变化时可以用sin值代替角度值，这样可以减轻单片机的计算负担。
				      //  式中*1.5是对角度的适当放大，14*3.14/180是对角度零点的修正				  
    complement_filter(acceler,gyro);
}
********************以上为互补滤波*******************************/

void PIT2_IRQHandler(void) 
{  
     PIT_Flag_Clear(PIT2); //清中断标志位 
     
     //float32_t  sin0_f32, sin1_f32, sin2_f32, sin3_f32,common_sin_factor = (float)1000;
     u8 temp = 0;
     for(temp = 64; temp >  0; temp--)
        {
         /*  sin0_f32 =  arm_sin_f32(temp);
           sin1_f32 =  arm_sin_f32(temp-3.14/4);
           sin2_f32 =  arm_sin_f32(temp+3.14/4);
           sin3_f32 =  arm_sin_f32(temp+3.14/2);
           arm_mult_f32(&sin0_f32, &common_sin_factor, &OutData[0],1);
           arm_mult_f32(&sin1_f32, &common_sin_factor, &OutData[1],1);
           arm_mult_f32(&sin2_f32, &common_sin_factor, &OutData[2],1);
           arm_mult_f32(&sin3_f32, &common_sin_factor, &OutData[3],1);
         */  
      //     AD_calculate();
           OutData[0] =   ad_mid(ADC1, SE4a, ADC_12bit); 
//           OutData[1] =   g_s16InputVoltage[2];  
       //    OutData[2] =   angle;  
       //    OutData[3] =   angle_dot; 
        //   VOLTAGE_GRAVITY  = OutData[1];
         //  VOLTAGE_GYRO     = OutData[0];
       //    AngleCalculate();
           OutData[2] =   0;
           
           OutPut_Data();
        }
}

//无线模块用的中断服务函数
//void SPI1_IRQHandler(void)
//{
//  NRF_Handler();
//}
/*
 void NRF_Handler(void)
{
    if( nrf_mode == TX_MODE)
    {
        NRF_ISR_Tx_Handler();
    }
    else if( nrf_mode == RX_MODE )
    {
        NRF_ISR_Rx_Handler();
    }
}
*/

void PORTE_IRQHandler()
{
 //   u8  n = 0;    //引脚号

    /*  根据自己的引脚号，自己编写，这里给出 n= 0 的模版，即 PTAn 产生外部中断
     *  也给出 n=26 例子 ，自行修改 n 即可  ，添加用户任务就行
     */

 //   n = 6;
   // Temp2=PORTB_ISFR;
    if(PORTE_ISFR & (1 << 5))           //PTA26触发中断
    {
        PORTE_ISFR  |= (1 << 5);        //写1清中断标志位
        /*  以下为用户任务  */

        NRF_Handler();

        /*  以上为用户任务  */
    }
}