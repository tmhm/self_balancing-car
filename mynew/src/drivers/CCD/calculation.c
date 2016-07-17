
#include "include.h"
#include "calculation.h"



/*************************************************************************
*                             ������ӹ�����
*
*  �������ƣ�CCD1_init
*  ����˵����CCD��ʼ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2013-3-6 xw
*  ��    ע��
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
*                           ������ӹ�����
*
*  �������ƣ�StartIntegration
*  ����˵����CCD��������
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
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
*                           ������ӹ�����
*
*  �������ƣ�CalculateIntegrationTime
*  ����˵���������ع�ʱ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/

void CalculateIntegrationTime(void) 
{

    /* ����128�����ص��ƽ��ADֵ */
    PixelAverageValue = PixelAverage(128,Pixel);    //�ѳ�128ֱ�Ӹĳ�������7λ


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
    else if(PixelAverageValue<85)            //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime+=3;
    }
    else if(PixelAverageValue<100)   //95         //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime+=2;
    }
    else if(PixelAverageValue<130)    //125        //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime++;
    }
    
    
   
    if(PixelAverageValue>245)            //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime-=5;
    }
    else if(PixelAverageValue>238)            //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime-=4;
    }
    else if(PixelAverageValue>223)            //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime-=3;
    }
    else if(PixelAverageValue>205)            //�Ƚϵ�90,122,151��184��ѡ����142
    {
        IntegrationTime-=2;
    }
  //  else if(Corss_deal_Flag )
  //  {
  //       IntegrationTime--;
 //   }
     else if(PixelAverageValue>180)     //175       //�Ƚϵ�90,122,151��184��ѡ����142
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
*                           ������ӹ�����
*
*  �������ƣ�PixelAverage
*  ����˵����������ľ�ֵ����
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/

u8 PixelAverage(u8 len, u8 *data)     //�ѳ�128ֱ�Ӹĳ�������7λ
{
    u8 i;
    u32 sum = 0;
    for(i = 0; i<len; i++)  //128����
    {
      sum = sum + *data++;
     
    }
    CCDtotal=sum;
    return ((s16)(sum>>7));
}
/*************************************************************************
*                           ������ӹ�����
*
*  �������ƣ�SendHex
*  ����˵�����ɼ���������
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
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
*                           ������ӹ�����
*
*  �������ƣ�SendImageData
*  ����˵����
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
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
*                           ������ӹ�����
*
*  �������ƣ�SamplingDelay
*  ����˵����CCD��ʱ���� 200ns
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/
 void SamplingDelay(void)
 {
  // volatile u8 i ;
 //  for(i=0;i<1;i++) {
    asm("nop");
    asm("nop");     //��2��CCD��������ʱ
 
 //}
   
}