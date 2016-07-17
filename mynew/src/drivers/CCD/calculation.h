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

u8 Pixel[128];              //定义在main里面不算全局变量？？    要在main外面定义，才能在另外的文件里申明
//u8 PixelTwo[128];  

//u8  If_CCD_start_flag;

/* 128个像素点的平均AD值 */
s16 PixelAverageValue;

/* 设定目标CCD平均值*/
//s16 TargetPixelAverageValue;

/* 设定目标平均电压值与实际值的偏差，实际电压的10倍 */
//s16 PixelAverageVoltageError;

/* 设定目标平均电压值允许的偏差，实际电压的10倍 */
//s16 TargetPixelAverageVoltageAllowError;



u8  send_data_cnt;

/* 曝光时间，单位ms */
s8 IntegrationTime;
//u8 TIME1flag_20ms;

u8 TimerCnt5ms;
u8 TimerCnt36ms;

u8 integration_piont;

u16 CCDtotal;
u32 CCD50total;
u8  CCD_C_Can_flag;

u8  CCD_count_start;  //延时累加CCD总量计数

void StartIntegration(void) ;   
void ImageCapture(unsigned char * ImageData) ;
void SendHex(unsigned char hex) ;
void SamplingDelay(void) ;
void CCD1_init(void) ;
void CCDTwo_init(void);

void CalculateIntegrationTime(void) ;
u8 PixelAverage(u8 len, u8 *data) ;      //把除128直接改成了右移7位，此函数只能用于128位的平均
void SendImageData(unsigned char * ImageData) ;


void StartIntegration(void) ;


#endif
