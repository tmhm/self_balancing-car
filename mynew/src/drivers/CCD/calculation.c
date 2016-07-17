
#include "include.h"
#include "calculation.h"



/*************************************************************************
*                             蓝宙电子工作室
*
*  函数名称：CCD1_init
*  功能说明：CCD初始化
*  参数说明：
*  函数返回：无
*  修改时间：2013-3-6 xw
*  备    注：
*************************************************************************/
void CCD1_init(void)
{
  gpio_init (PORTB , 16, GPO,HIGH);       //CLK
  gpio_init (PORTB , 17, GPO,HIGH);       //SI
  adc_init(ADC1, AD14) ;                  //B10
}

void CCDTwo_init(void)
{
  gpio_init (PORTE , 26, GPO,HIGH);       //CLK
  gpio_init (PORTE , 25, GPO,HIGH);       //SI
  adc_init(ADC0, AD17) ;                  //E26
}



/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：StartIntegration
*  功能说明：CCD启动程序
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void StartIntegration(void) {

    unsigned char i;

    SI_SetVal();            /* SI  = 1 */   
    
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */;  
    
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */

    for(i=127; i > 0; i--) 
    {
     
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
   
        
        SamplingDelay();
        CLK_ClrVal();       /* CLK = 0 */
     
    }
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */ 
}



void ImageCapture(unsigned char * ImageData) 
{

    unsigned char i;

    SI_SetVal();            /* SI  = 1 */
    
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    
    SamplingDelay();

    //Sampling Pixel 1

    *ImageData =  ad_once(ADC1, AD14, ADC_8bit);
    
    ImageData ++ ;
    
    CLK_ClrVal();           /* CLK = 0 */

    for(i=127; i>0; i--) 
    {
        SamplingDelay();
   
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        //Sampling Pixel 2~128

       *ImageData =  ad_once(ADC1, AD14, ADC_8bit);
        ImageData ++ ;
        
        CLK_ClrVal();       /* CLK = 0 */
        
    }

    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */

    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
}
 

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：CalculateIntegrationTime
*  功能说明：计算曝光时间
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/

void CalculateIntegrationTime(void) 
{

    /* 计算128个像素点的平均AD值 */
    PixelAverageValue = PixelAverage(128,Pixel);    //把除128直接改成了右移7位


    if( Roadblock_flag  || Corss_deal_Flag ){     //Corss_deal_Flag  == 1  || 
    } 
    else if(Roadblock_Intime_flag)
    {   
        Roadblock_Intime_flag = 0;
        IntegrationTime++;
    }
    else if(LittleS_flag)
    {
        LittleS_flag  = 0;
        IntegrationTime+=2;  //+1
    }
    else if(PixelAverageValue<85)            //比较点90,122,151，184，选定点142
    {
        IntegrationTime+=3;
    }
    else if(PixelAverageValue<100)   //95         //比较点90,122,151，184，选定点142
    {
        IntegrationTime+=2;
    }
    else if(PixelAverageValue<130)    //125        //比较点90,122,151，184，选定点142
    {
        IntegrationTime++;
    }
    
    
   
    if(PixelAverageValue>245)            //比较点90,122,151，184，选定点142
    {
        IntegrationTime-=5;
    }
    else if(PixelAverageValue>238)            //比较点90,122,151，184，选定点142
    {
        IntegrationTime-=4;
    }
    else if(PixelAverageValue>223)            //比较点90,122,151，184，选定点142
    {
        IntegrationTime-=3;
    }
    else if(PixelAverageValue>205)            //比较点90,122,151，184，选定点142
    {
        IntegrationTime-=2;
    }
  //  else if(Corss_deal_Flag )
  //  {
  //       IntegrationTime--;
 //   }
     else if(PixelAverageValue>180)     //175       //比较点90,122,151，184，选定点142
    {
        IntegrationTime--;
    }     
     else if( Roadblock_flag  || Corss_deal_Flag ){    //Corss_deal_Flag  == 1 ||  
    }
    
    
    if(IntegrationTime >= 26)
        IntegrationTime = 26;
    
    if(IntegrationTime <= 1)
        IntegrationTime = 1;
    
   
    integration_piont = 36 - IntegrationTime;     
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：PixelAverage
*  功能说明：求数组的均值程序
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/

u8 PixelAverage(u8 len, u8 *data)     //把除128直接改成了右移7位
{
    u8 i;
    u32 sum = 0;
    for(i = 0; i<len; i++)  //128个点
    {
      sum = sum + *data++;
     
    }
    CCDtotal=sum;
    return ((s16)(sum>>7));
}
/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：SendHex
*  功能说明：采集发数程序
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void SendHex(unsigned char hex) {
  unsigned char temp;
  temp = hex >> 4;
  if(temp < 10) 
  {
    uart_putchar(UART1,temp + '0');
  } 
  else 
  {
    uart_putchar(UART1,temp - 10 + 'A');
  }
  
  temp = hex & 0x0F;
  if(temp < 10) 
  {
    uart_putchar(UART1,temp + '0');
  } 
  else 
  {
   uart_putchar(UART1,temp - 10 + 'A');
  }
}
/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：SendImageData
*  功能说明：
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void SendImageData(unsigned char * ImageData) {

    unsigned char i;
    unsigned char crc = 0;

    /* Send Data */
    uart_putchar(UART1,'*');
    uart_putchar(UART1,'L');
    uart_putchar(UART1,'D');

    SendHex(0);
    SendHex(0);
    SendHex(0);
    SendHex(0);

    for(i=0; i<128; i++) 
    {
      SendHex(*ImageData++);
    }

    SendHex(crc);
    uart_putchar(UART1,'#');
}
/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：SamplingDelay
*  功能说明：CCD延时程序 200ns
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
 void SamplingDelay(void)
 {
  // volatile u8 i ;
 //  for(i=0;i<1;i++) {
    asm("nop");
    asm("nop");     //加2个CCD，降低延时
 
 //}
   
}