#include "common.h"
#include "include.h"
//#include "arm_math.h"

/********************************ȫ�ֱ�������*************************/
                     
u8  txbuf[32]={1,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32};	 //���ͻ���
s16 NRFtx[5]={0,0,0,0,0};
//CCDerr,Angle,speed,�ع�ʱ�䣬CCDֵ
u8  Rxbuf[33]={0};

void  main()
{
     
     DisableInterrupts;     //��ֹ���ж�
     
     u8 *pixel_pt;         
  //   u8  my_key1=0,my_key2=0;
//**************************��·pwm�����ʼ��**************************//
  
     FTM_PWM_init(FTM0,CH1,5000,1000);        //A4     ��  ����  ok  
     FTM_PWM_init(FTM0,CH2,5000,1000);        //A5     ��  ǰ��  ok
       
     FTM_PWM_init(FTM2,CH0,5000,1000);        //B18    ��  ǰ��  ok    
     FTM_PWM_init(FTM2,CH1,5000,1000);        //B19    ��  ����  ok
  
      
//***********************ȫ�ֱ�����ʼ��********************//      
     gpio_init(PORTD,5,GPI,0);   //ƫ��ѡ�񿪹�
     gpio_init(PORTD,6,GPI,0);   //·�Ͽ���
     
     CarSubInit();                       //��Ҫ�õĹ�������ʼ��
     Discuss_Init();                     //����Ҫ���ԵĲ�����ʼ��
     Speed_add_init();  
     
     angle_ratio= 0.35;     //5����0.255    //11����0.12;
   //  Q_angle=0.003, Q_gyro=0.0003; 
     R_angle=250;
     Car_DeadVoltage  = 0.00;

//***********************led�Ƽ����뿪�س�ʼ��********************//      
     LED_init();                //A15,C16,C17,C18,��ʼ��IO�����1.            2013.5.6����ok��ͨ�ӵ�
                                //C16����MMA8451_OK   �������ȡʧ��
                                //C17��������������ƫ��OK
                                //C18������ʼ�������Ǿ�̬ƫ��
  //   LCD_Init();
  //   gpio_init(PORTE,25,GPI,0);   //
     
     boma_key_init();           // C12����Чλ��C15 C14 C13 C12 : 0000-1111 ��16��ѡ��    2013.5.6����ok
     
     debug_boma_key();
   //  NRF_Init();       //2013.5.5���� ����ʼ���ɹ�������ok
    
/**************************MMA���ٶȼ�ģ���ʼ��**********************/      
     MMA845_init();      //ģ��IIc����mma8451�������������õ�����������B23
                                
//**************************��·�����ǳ�ʼ��**************************//    
// ADC_xwstart��xw�����8��Ӳ����ֵ����
    // GPIO_SET(PORTC,18,0);                //��ߵ�����ʼ����̬ƫ��
     adc_init(ADC1, SE4a);                          //������1����PTE0       ת����    8λAD
    
     adc_init(ADC0, SE8);                         //��ʼ��ADC0_SE8 ,--PTB0    ֱ����    12λAD

     time_delay_ms(900);    //��ʱȥ��
     /*
     my_key1=GPIO_GET_1bit(PORTE,25);
     my_key2=GPIO_GET_1bit(PORTE,26);
     if((my_key1==0)&&(my_key2==0)) TargetPixelAverageValue=87;
     else if((my_key1==0)&&(my_key2==0))TargetPixelAverageValue=102;
     else if((my_key1==1)&&(my_key2==0))TargetPixelAverageValue=123;
     else if((my_key1==1)&&(my_key2==1))TargetPixelAverageValue=142;
     */
     g_s16GYRO_DirOFFSET    = Get_ad1_Offset(ADC1, SE4a, ADC_8bit,64);  
     g_s16GYROSCOPE1_OFFSET = Get_ad1_Offset(ADC0, SE8, ADC_12bit,64);  

     GYROSCOPE1_OFFSET_Check();     //�м������ֱ����ת��OFFSet ,ok   ,��ˮ��ָʾƫ�û�ô���  
     
                                 

//**************************��·���������ʼ��****************************//    
     DMA_count_Init(DMA_CH4, PTC0 ,0x7FFF,DMA_falling);     //C0    �ұ߼���  ok
     
     DMA_count_Init(DMA_CH5, PTD0 ,0x7FFF,DMA_falling);     //Do    ��߼���  ok
    
//*****************************CCD��ʼ��**********************************// 
     CCD1_init();         //��ʼ��AD14��B10�����õ�������IO��B16��B17
     u8 k;   
     pixel_pt = Pixel;
     for(k=0; k<128; k++) 
     {
        *pixel_pt++ = 0;
     }
     

     //920us��ʱ��ȡ128��CCD��ADֵ
     CCD_GateValve = 110;
     Car_Middle_site = 63.5;
 
//**************************PIT��ʱ����ʼ��*******************************//     
    //pit_init_ms(PIT0,1); 
     pit_init(PIT0, 20000); 
    
    //  LCD_P8x16Str(0,0,"angot:");          //Ӣ���ַ�����ʾ 
    //  LCD_P8x16Str(0,2,"speot: ");          //Ӣ���ַ�����ʾ
    
    EnableInterrupts;       //�����ж�
    
  //  StartIntegration();     //�жϿ�ʼ����ǰ��CCD��λһ��
     
    
    
    while(1)
    {    
        //  NRF_Rx_Dat(Rxbuf);
        //   if(Rxbuf[0]==5) Car_Stop();
      
    
    /*  PTE25_OUT = 1;    //����40ns����
      SamplingDelay();
      SamplingDelay();
      PTE25_OUT = 0;      //������������200ns
      SamplingDelay();    
      SamplingDelay();
    */  
      if(GPIO_GET_1bit(PORTD,5)==1) Roadblock_key = 1;
      
      if(ImageCCD_flag)       //4.5ms
      {//gpio_set (PORTE, 25, 0);  
          ImageCCD_flag = 0;
          ImageCapture(Pixel);      //3.7ms��ʱ��ȡ128��CCD��ADֵ,
          CCD_NRF_flag  = 1;     
          CalculateIntegrationTime();    //30us   
          CCD_Control();           //100us
          
          send_data_cnt++;    //��������CCD��λ�������ݣ�����           
          g_nDirectionControlCount = 0;
          g_nDirectionControlPeriod = 0;
 //   gpio_set (PORTE, 25, 1);  
      }
      
      
      
      if(CCD_C_Can_flag )  {
          CCD_shi = CCD50total/8;
         // if(CCD_shi>21400) CCD_shi=21400;
       //   else if(CCD_shi<19000) CCD_shi=19000;
          CCD_C_Can_flag  = 0;
      }
          Stand_Check();      //��������

      
      //   LED_TURN(LED0);  
          
      //   sum11 = ad_xwonce(ADC0, SE8, ADC_8bit); //�ɼ�ĳ·ģ������ADֵ,�Ӱ˴�Ӳ��ƽ���˲�    �˴�Ӳ����ֵ�˲�28.5us
      //   sum11 = ad_once(ADC0, SE8, ADC_8bit);     //8λADһ��6.5us
      //   time_delay_ms(500); 


//*********************����ʾ�������****************// 
    //    OutData[0] = Acc_Xvalue;
      /*  
         OutData[0] =    angle_m;
         
     
         OutData[1] =   gyro_m;
               
         OutData[2] =  g_fCarAngle; //angle; // 
         OutData[3] = g_fGyroscopeAngleSpeed;  //  angle_dot;// 
           
         OutPut_Data();
        */
        // gpio_turn(PORTE,25);

//*********************CCD����****************//        

        /*    if(CCD_Cstart_flag  == 1)
            {// gpio_set (PORTE, 25, 0);      
               CCD_Cstart_flag  =0;
               CCD_Control();           //100us
               CCD_Out_flag  = 1;
               g_nDirectionControlCount = 0;
               g_nDirectionControlPeriod = 0;
               //gpio_set (PORTE, 25, 1);
            }  */
          
            
           if(send_data_cnt >=10) 
            {
                send_data_cnt = 0;
               // SendImageData(Pixel);   //24ms��128����   37ms 
            }
            
           if(CCD_NRF_flag)   //10ms����һ��
           {
              CCD_NRF_flag=0;
            //  NRFdata_Tidy();  
            //  NRF_ISR_Tx_Dat(txbuf,MAX_ONCE_TX_NUM);
            //  NRF_RX_Mode();
           } 
          
           
           /*     
           if((RX_ISR_FIFO[0][0])==5)
           {
                Car_Stop();
                Led_Indicate();
           }*/
         
//*********************����ΪCCD����****************//       
                 
            
    // LCD_Fill(0xff);//���� 
    // time_delay_ms(100); 
    // LCD_Fill(0x00);//����
    // time_delay_ms(1000);      
    // ANG = (s16)(g_fAngleControlOut  *1000);
      
    // LCD_P8x16Str(64,0,"ANG");          //Ӣ���ַ�����ʾ
      
           
   }

}
