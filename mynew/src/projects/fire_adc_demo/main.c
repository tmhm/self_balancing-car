#include "common.h"
#include "include.h"
//#include "arm_math.h"

/********************************全局变量定义*************************/
                     
u8  txbuf[32]={1,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32};	 //发送缓冲
s16 NRFtx[5]={0,0,0,0,0};
//CCDerr,Angle,speed,曝光时间，CCD值
u8  Rxbuf[33]={0};

void  main()
{
     
     DisableInterrupts;     //禁止总中断
     
     u8 *pixel_pt;         
  //   u8  my_key1=0,my_key2=0;
//**************************四路pwm输出初始化**************************//
  
     FTM_PWM_init(FTM0,CH1,5000,1000);        //A4     右  后行  ok  
     FTM_PWM_init(FTM0,CH2,5000,1000);        //A5     右  前行  ok
       
     FTM_PWM_init(FTM2,CH0,5000,1000);        //B18    左  前行  ok    
     FTM_PWM_init(FTM2,CH1,5000,1000);        //B19    左  后行  ok
  
      
//***********************全局变量初始化********************//      
     gpio_init(PORTD,5,GPI,0);   //偏置选择开关
     gpio_init(PORTD,6,GPI,0);   //路障开关
     
     CarSubInit();                       //需要用的过程量初始化
     Discuss_Init();                     //各种要调试的参数初始化
     Speed_add_init();  
     
     angle_ratio= 0.35;     //5倍的0.255    //11倍的0.12;
   //  Q_angle=0.003, Q_gyro=0.0003; 
     R_angle=250;
     Car_DeadVoltage  = 0.00;

//***********************led灯及拨码开关初始化********************//      
     LED_init();                //A15,C16,C17,C18,初始化IO口输出1.            2013.5.6测试ok，通接地
                                //C16亮，MMA8451_OK   ，否则读取失败
                                //C17亮，两个陀螺仪偏置OK
                                //C18亮，开始读陀螺仪静态偏置
  //   LCD_Init();
  //   gpio_init(PORTE,25,GPI,0);   //
     
     boma_key_init();           // C12低有效位，C15 C14 C13 C12 : 0000-1111 共16种选择    2013.5.6测试ok
     
     debug_boma_key();
   //  NRF_Init();       //2013.5.5测试 ，初始化成功，测试ok
    
/**************************MMA加速度计模拟初始化**********************/      
     MMA845_init();      //模拟IIc启动mma8451，包括启动其用到的两个引脚B23
                                
//**************************两路陀螺仪初始化**************************//    
// ADC_xwstart中xw添加了8次硬件均值采样
    // GPIO_SET(PORTC,18,0);                //左边灯亮开始读静态偏置
     adc_init(ADC1, SE4a);                          //陀螺仪1——PTE0       转向用    8位AD
    
     adc_init(ADC0, SE8);                         //初始化ADC0_SE8 ,--PTB0    直立用    12位AD

     time_delay_ms(900);    //延时去抖
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

     GYROSCOPE1_OFFSET_Check();     //中间灯亮，直立与转向OFFSet ,ok   ,流水灯指示偏置获得错误  
     
                                 

//**************************两路脉冲计数初始化****************************//    
     DMA_count_Init(DMA_CH4, PTC0 ,0x7FFF,DMA_falling);     //C0    右边计数  ok
     
     DMA_count_Init(DMA_CH5, PTD0 ,0x7FFF,DMA_falling);     //Do    左边计数  ok
    
//*****************************CCD初始化**********************************// 
     CCD1_init();         //初始化AD14（B10）及用到的两个IO口B16，B17
     u8 k;   
     pixel_pt = Pixel;
     for(k=0; k<128; k++) 
     {
        *pixel_pt++ = 0;
     }
     

     //920us的时间取128个CCD的AD值
     CCD_GateValve = 110;
     Car_Middle_site = 63.5;
 
//**************************PIT定时器初始化*******************************//     
    //pit_init_ms(PIT0,1); 
     pit_init(PIT0, 20000); 
    
    //  LCD_P8x16Str(0,0,"angot:");          //英文字符串显示 
    //  LCD_P8x16Str(0,2,"speot: ");          //英文字符串显示
    
    EnableInterrupts;       //开总中断
    
  //  StartIntegration();     //中断开始运行前把CCD复位一次
     
    
    
    while(1)
    {    
        //  NRF_Rx_Dat(Rxbuf);
        //   if(Rxbuf[0]==5) Car_Stop();
      
    
    /*  PTE25_OUT = 1;    //单条40ns左右
      SamplingDelay();
      SamplingDelay();
      PTE25_OUT = 0;      //下面三条将近200ns
      SamplingDelay();    
      SamplingDelay();
    */  
      if(GPIO_GET_1bit(PORTD,5)==1) Roadblock_key = 1;
      
      if(ImageCCD_flag)       //4.5ms
      {//gpio_set (PORTE, 25, 0);  
          ImageCCD_flag = 0;
          ImageCapture(Pixel);      //3.7ms的时间取128个CCD的AD值,
          CCD_NRF_flag  = 1;     
          CalculateIntegrationTime();    //30us   
          CCD_Control();           //100us
          
          send_data_cnt++;    //用来发送CCD上位机的数据，计数           
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
          Stand_Check();      //倒车保护

      
      //   LED_TURN(LED0);  
          
      //   sum11 = ad_xwonce(ADC0, SE8, ADC_8bit); //采集某路模拟量的AD值,加八次硬件平均滤波    八次硬件均值滤波28.5us
      //   sum11 = ad_once(ADC0, SE8, ADC_8bit);     //8位AD一次6.5us
      //   time_delay_ms(500); 


//*********************虚拟示波器输出****************// 
    //    OutData[0] = Acc_Xvalue;
      /*  
         OutData[0] =    angle_m;
         
     
         OutData[1] =   gyro_m;
               
         OutData[2] =  g_fCarAngle; //angle; // 
         OutData[3] = g_fGyroscopeAngleSpeed;  //  angle_dot;// 
           
         OutPut_Data();
        */
        // gpio_turn(PORTE,25);

//*********************CCD部分****************//        

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
               // SendImageData(Pixel);   //24ms发128个点   37ms 
            }
            
           if(CCD_NRF_flag)   //10ms发送一次
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
         
//*********************以上为CCD部分****************//       
                 
            
    // LCD_Fill(0xff);//黑屏 
    // time_delay_ms(100); 
    // LCD_Fill(0x00);//亮屏
    // time_delay_ms(1000);      
    // ANG = (s16)(g_fAngleControlOut  *1000);
      
    // LCD_P8x16Str(64,0,"ANG");          //英文字符串显示
      
           
   }

}
