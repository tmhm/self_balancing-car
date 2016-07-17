#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */

#include  "gpio.h"        //IO口操作
#include  "LED.H"         //流水灯
#include  "exti.h"        //EXTI外部GPIO中断
#include  "uart.h"        //串口
#include  "lptmr.h"       //低功耗定时器(延时)
#include  "adc.h"
#include  "mma8451.h"     //加速度计
//#include  "i2c.h"
#include  "dma.h"         // 脉冲计数
#include  "FTM.h"         // pwm输出
#include  "boma_key.h"    // 拨码开关用
#include  "NRF24L0.h"     //无线模块
#include  "PIT.h"         //中断定时器
#include  "exti.h"        //EXTI外部GPIO中断
#include  "arm_math.h"    //DSP库
#include  "calculation.h" //CCD
#include  "outputdata.h"  //虚拟示波器
#include  "carcontrol.h"      //控制小车所用的函数
//#include  "CONFIG.H"      //配置小车整定的参数

//#include    "xw12864.h"

#endif  //__INCLUDE_H__
