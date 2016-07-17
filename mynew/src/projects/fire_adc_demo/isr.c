#include "common.h"
#include "include.h"
#include "arm_math.h"
#include "isr.h"
//#include  "NRF24L0.h"     //����ģ��


extern s8 IntegrationTime ;
extern void StartIntegration(void);
extern  u8 Pixel[128]; 

void PIT0_IRQHandler(void) 
{  

        PIT_Flag_Clear(PIT0); 
  
      //  gpio_set (PORTE, 25, 0);
        g_n1MSEventCount ++;
        
	Stand_Check();      //��������

        //--------------------------------------------------------------------------
        TimerCnt36ms++;     
        if(TimerCnt36ms  >=36)  TimerCnt36ms  =0;     //��4ms������ȡAD
        //------------------------------------------------------------------------------------------------------------------------------      
        //���ڼ����ع�ʱ�亯�������� integration_piont = 10 - IntegrationTime;     //!!�ع�ʱ��ĺ�����Զ�����ȡAD�ĺ���ͬʱ����
       // if(integration_piont >= 2)                                                //��Ϊ�ع�һ������10msǰ��ĳһ��ʱ��
       // {      /* �ع�ʱ��С��2�򲻽������ع� */
        if(integration_piont == TimerCnt36ms)    
        {
          StartIntegration();    //�ع��50us     
        }
        //}  
         //------------------------------------------------------------------------------------------------------------------------------     
        
        //����Ϊ�ж��ڷ�ʱ���ƣ�ÿ�����ֻ����750us��
 	//--------------------------------------------------------------------------
  	if(g_n1MSEventCount >= CONTROL_PERIOD)      //��ȡADֵ����һ��1ms����     //�˴�60us�����Կ��ɽ����ж�ѹջʱ��
        {  //=====CCD��������======================          //�˶ι�840us����
            g_n1MSEventCount = 0;  
            TimerCnt5ms ++;
            if(TimerCnt5ms>=3)      //3.7ms
            {// gpio_set (PORTE, 25, 0);   
              TimerCnt5ms=0;
              ImageCCD_flag = 1;
              
         //     ImageCapture(Pixel);      //3.7ms��ʱ��ȡ128��CCD��ADֵ,
         //     CCD_Cstart_flag  = 1;     //����CCD��AD֮����mian���������
         //     CalculateIntegrationTime();    //30us   
         //     send_data_cnt++;    //��������CCD��λ�������ݣ�����   
             //  gpio_set (PORTE, 25, 1);   
            }
  	}  
   //     else  if(g_n1MSEventCount == 1)    // CCD control,�п���������ϴε�ֵ����main�����ȡֵ��ͬ�����ѽ��
   //     {                                 //�������ֻ����1ms��       ,�ŵ�main���棡2013.6.16      
        
  	//}
        else if(g_n1MSEventCount == 3)     //=====�ٶȿ��Ƽ���======================
        {      
              g_nSpeedControlCount ++;
              if(g_nSpeedControlCount >= SPEED_CONTROL_COUNT) 
              {
                 //--�����ֶμ���------------------------------------------------
                      gu8_StarttimeCount--;
                      if(start_time_flag == 0 && gu8_StarttimeCount==0)     
                      {                  
                            gu8_StarttimeCount  = 1;
                            if(g_fCarSpeed_defult <  g_fCarSpeedSet){
                                  g_fCarSpeed_defult  +=  0.3;    //0.5
                                  if(g_fCarSpeed_defult >=  g_fCarSpeedSet){
                                   start_time_flag = 1;     //�ٶ��Ѿ��ӵ��������ٶ�
                        }}}
                      
                      if(start_time_flag )  Started_encoder += CarSpeed_data;
                      if(Started_encoder>3000){        //����6s�� ��ʼ��������ߣ���λ����ͣ����־
                        Can_Stop_flag=1;
                        Started_encoder = 0;
                      }
              
                 //--ͣ������---------------------------------------------------
                      if(Stop_flag )   //���Ȳ��ֶν��ٴ���
                      {
                            Stop_encoder += CarSpeed_data;       //��ʱ0.5s����������ߣ���ͣ��
                            if(Stop_encoder>950)    {g_fCarSpeed_defult  = 0.0;
                                 while(1)
                                  {
                                        Car_Stop();
                                      //  Led_Indicate();
                                  }
                            }
                      }
                      //else
                    //  {       //--·�ϼ�ֱ��������ٶȴ���-----------------------------------
                     //         if((Roadblock_flag )) {
                        //               Roadblock_flag =0;
                                     //  if(g_fCarSpeed_defult>2.0)  g_fCarSpeed_defult  = 2.0;       //·���ٶȣ�ò������ֻ����10.0��·�ϣ��������� 
                       
                    //  }
                
                   //   if((g_fCarSpeed_defult <  g_fCarSpeedSet-0.7)  &&(Stop_flag  == 0) &&(Can_Stop_flag == 1))  g_fCarSpeed_defult++;
                  //    if(g_fCarSpeed_defult <  g_fCarSpeedSet-0.7 && start_time_flag == 1)   g_fCarSpeed_defult+=0.5;    //�ϰ����ٺ����
                      
                     
                            
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
                            CCD_c_Ga_flag = 0;    //��ʼ�ۼ�8������CCD����,100msyici
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
        else if(g_n1MSEventCount == 6)            //ȡ��ֱ��������Ҫ�ĽǶȼ����ٶ�    �ܹ�650us ����    �����250us����
        {    //=====ֱ�����Ƽ���======================
             Get_mma8451_z();                         //8��ƽ������ʱ451us    4��     2ci    
             ad1_XW_ave(ADC0, SE8, ADC_12bit,1);   //2��ƽ����ÿ��8��Ӳ���˲�   
             Kalman_Filter((float)VOLTAGE_GRAVITY , (float)VOLTAGE_GYRO);	//40us
  	} 
        else if(g_n1MSEventCount == 9)    //========ֱ�����Ƽ�������==========     
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
*  �������ƣ�PIT1_IRQHandler
*  ����˵����PIT1��ʱ�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-9-18    �Ѳ���
*  ��    ע��
*************************************************************************/

void  PIT1_IRQHandler(void) 
{  
    LED_TURN(LED0); //LED0��ת 
    PIT_Flag_Clear(PIT1); //���жϱ�־λ 
}


/**********************�������˲�������*****************************
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
    
    while(!ATD0STAT0_SCF);   //�ȴ�ת�����
         
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

     gyroscope_rate = Gyro*0.0023;         //�ο���ѹ3.3v 12λADC �Ŵ�9.1�� enc-03 0.67mv/deg./sec.
     
                                          //(3300/4096)/(0.67*9.1)*(3.14/180) =  0.0023
     
    NowData = RealData + gyroscope_rate*0.005;                 
     //1.Ԥ���� X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
     NowData_P = sqrt(Q*Q+RealData_P*RealData_P);             
    //2.����Ԥ����Э�������   P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k)
     Kg = sqrt(NowData_P*NowData_P/(NowData_P*NowData_P+R*R));
     //3.���㿨����������� K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k))
     RealData = NowData + Kg*(accelerometer_angle - NowData); 
    //4.���¹��� X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
     RealData_P = sqrt((1-Kg)*NowData_P*NowData_P);           
     //5.������º����Э������� P(k|k) =��I-K(k)*H(k)��*P(k|k-1)
     
    QingJiao =  RealData;  
     
    
    OutData[2] = QingJiao*1000; 
    OutPut_Data();
 }

********************����Ϊ�������˲�*******************************/

/********************����Ϊ�����˲�*******************************/

/*
static float acceler,gyro;	
static  float  AD_ENC180_result , IIC_MMA8451_result;
void AD_calculate(void)
{
    AD_ENC180_result    =   ad_once(ADC1, SE4a, ADC_12bit);         //��ȡ ADC1_SE4a ��16λ����,E0
    IIC_MMA8451_result  = Get_mma8451_z;
    gyro=0.011557*AD_ENC180_result-3.035+105*3.14/180;	//����105*3.14/180�ǶԽ��ٶ���������
     
    acceler=0.003052*IIC_MMA8451_result-1.4375;		//����Ƕ�ʱ��ֻ���˼��ٶȴ�������Z��͹���
                                                                            
    acceler=acceler*1.5+14*3.14/180;  //û��ʹ��arcsin��������Ϊsin��0�ȸ����仯ʱ������sinֵ����Ƕ�ֵ���������Լ��ᵥƬ���ļ��㸺����
				      //  ʽ��*1.5�ǶԽǶȵ��ʵ��Ŵ�14*3.14/180�ǶԽǶ���������				  
    complement_filter(acceler,gyro);
}
********************����Ϊ�����˲�*******************************/

void PIT2_IRQHandler(void) 
{  
     PIT_Flag_Clear(PIT2); //���жϱ�־λ 
     
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

//����ģ���õ��жϷ�����
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
 //   u8  n = 0;    //���ź�

    /*  �����Լ������źţ��Լ���д��������� n= 0 ��ģ�棬�� PTAn �����ⲿ�ж�
     *  Ҳ���� n=26 ���� �������޸� n ����  ������û��������
     */

 //   n = 6;
   // Temp2=PORTB_ISFR;
    if(PORTE_ISFR & (1 << 5))           //PTA26�����ж�
    {
        PORTE_ISFR  |= (1 << 5);        //д1���жϱ�־λ
        /*  ����Ϊ�û�����  */

        NRF_Handler();

        /*  ����Ϊ�û�����  */
    }
}