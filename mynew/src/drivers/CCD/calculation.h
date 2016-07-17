#ifndef  calculation_H
#define  calculation_H  

//#define TRUE        1
#define SI_SetVal()   PTB17_OUT = 1
#define SI_ClrVal()   PTB17_OUT = 0
#define CLK_ClrVal()  PTB16_OUT = 0
#define CLK_SetVal()  PTB16_OUT = 1

#define SITwo_SetVal()   PTE25_OUT = 1
#define SITwo_ClrVal()   PTE25_OUT = 0
#define CLKTwo_ClrVal()  PTE26_OUT = 0
#define CLKTwo_SetVal()  PTE26_OUT = 1

u8 Pixel[128];              //������main���治��ȫ�ֱ�������    Ҫ��main���涨�壬������������ļ�������
//u8 PixelTwo[128];  

//u8  If_CCD_start_flag;

/* 128�����ص��ƽ��ADֵ */
s16 PixelAverageValue;

/* �趨Ŀ��CCDƽ��ֵ*/
//s16 TargetPixelAverageValue;

/* �趨Ŀ��ƽ����ѹֵ��ʵ��ֵ��ƫ�ʵ�ʵ�ѹ��10�� */
//s16 PixelAverageVoltageError;

/* �趨Ŀ��ƽ����ѹֵ�����ƫ�ʵ�ʵ�ѹ��10�� */
//s16 TargetPixelAverageVoltageAllowError;



u8  send_data_cnt;

/* �ع�ʱ�䣬��λms */
s8 IntegrationTime;
//u8 TIME1flag_20ms;

u8 TimerCnt5ms;
u8 TimerCnt36ms;

u8 integration_piont;

u16 CCDtotal;
u32 CCD50total;
u8  CCD_C_Can_flag;

u8  CCD_count_start;  //��ʱ�ۼ�CCD��������

void StartIntegration(void) ;   
void ImageCapture(unsigned char * ImageData) ;
void SendHex(unsigned char hex) ;
void SamplingDelay(void) ;
void CCD1_init(void) ;
void CCDTwo_init(void);

void CalculateIntegrationTime(void) ;
u8 PixelAverage(u8 len, u8 *data) ;      //�ѳ�128ֱ�Ӹĳ�������7λ���˺���ֻ������128λ��ƽ��
void SendImageData(unsigned char * ImageData) ;


void StartIntegration(void) ;


#endif
