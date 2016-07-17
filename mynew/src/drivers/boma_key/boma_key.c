#include  "boma_key.h"
#include  "gpio.h"        //IO口操作
#include  "carcontrol.h"
/*****************************************************
*  C12低有效位，C15 C14 C13 C12 : 0000-1111 共16种选择
*****************************************************/
void  boma_key_init(void)
{
    gpio_init(PORTC,12,GPI,1);     //定义c端口12脚输入
    gpio_init(PORTC,13,GPI,1);     //定义c端口13脚输入
    gpio_init(PORTC,14,GPI,1);     //定义c端口14脚输入
    gpio_init(PORTC,15,GPI,1);     //定义c端口15脚输入
    
   // GPIO_DDR_4bit(PORTC,12,0x0);
}

void debug_boma_key(void)
{
    u8  i;
    i = GPIO_GET_4bit(PORTC,12);
    switch(i)
    {
      case  0:     g_fCarSpeedSet = 0.0;      
      /*     AngleControlP  = 0.15;
           AngleControlD  = 0.0022;
           SpeedControlP  = 0.125;
           SpeedControlI  = 0.02;
           Dir_mControl_P = 0.003;
           Dir_mControl_I = 0.0;
      */
        break;
      case  1:     g_fCarSpeedSet = 7.0; //7  //9
       /*    AngleControlP  = 0.15;
           AngleControlD  = 0.0022;
           SpeedControlP  = 0.125;
           SpeedControlI  = 0.02;
           Dir_mControl_P = 0.003;
           Dir_mControl_I = 0.0;
       */       break;
      case  2:     g_fCarSpeedSet = 10.0;//12
       /*    AngleControlP  = 0.172;
           AngleControlD  = 0.0029;
           SpeedControlP  = 0.13;
           SpeedControlI  = 0.02;
           Dir_mControl_P = 0.003;
           Dir_mControl_I = 0.0;
      */  break;
      case  3:  g_fCarSpeedSet = 12.0;
          //  AngleControlP  = 0.3160;
        break;
      case  4:  g_fCarSpeedSet = 13.0;
          //  AngleControlP  = 0.3360;
        break;
      case  5: g_fCarSpeedSet  = 14.0;
         //  AngleControlP  = 0.3560;
        break;
      case  6:  g_fCarSpeedSet = 15.0;
         //  AngleControlP  = 0.3760;
        break;
      case  7:  g_fCarSpeedSet = 16.0;
         //  AngleControlP  = 0.3860;
        break;
      case  8:  g_fCarSpeedSet = 17.0;
         //  AngleControlP  = 0.3960;
        break;
      case  9:  g_fCarSpeedSet = 6.0;
                slope_key=1;
         //  AngleControlP  = 0.4060;
        break;
      case  10: g_fCarSpeedSet = 9.0;
                slope_key=1;
         //  AngleControlP  = 0.4260;
        break;
      case  11: g_fCarSpeedSet = 12.0;
                slope_key=1;
         //  AngleControlP  = 0.4460;
        break;
      case  12: g_fCarSpeedSet = 14.0;
                slope_key=1;
         //  AngleControlP  = 0.4560;
        break;
      case  13: g_fCarSpeedSet = 16.0;
                slope_key=1;
         //  AngleControlP  = 0.4660;
        break;
      case  14: g_fCarSpeedSet = 18.0;
                slope_key=1;
         //  AngleControlP  = 0.4760;
        break;
      case  15: g_fCarSpeedSet = 19.0;
              slope_key=1;
        // AngleControlP  = 0.4860;
        break;
    }
}
