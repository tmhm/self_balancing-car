/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��isr.h
 * ����         �����º궨���жϺţ���ӳ���ж�����������жϺ�����ַ��
 *                ʹ��ָ��������������жϷ������������жϷ�����
 *                ���棺ֻ����"vectors.c"���������ұ�����"vectors.h"�����ĺ��棡����
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��Ұ��Ƕ��ʽ����������
 * �Ա���       ��http://firestm32.taobao.com
 * ����֧����̳ ��http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/



#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"

/*                          ���¶����ж�������
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��       #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���       #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 */
#undef  VECTOR_107
#define VECTOR_107    PORTE_IRQHandler    //���¶���104���ж�ΪPORTB_IRQHandler�ж�  


#undef  VECTOR_084 
#define VECTOR_084    PIT0_IRQHandler       //���¶���84���ж�ΪPIT0_IRQHandler�ж� 

#undef  VECTOR_085
#define VECTOR_085    PIT1_IRQHandler     //���¶���85���ж�ΪPIT1_IRQHandler�ж�


#undef  VECTOR_086
#define VECTOR_086    PIT2_IRQHandler     //���¶���86���ж�ΪPIT2_IRQHandler�ж�

//#undef  VECTOR_087
//#define VECTOR_087    PIT3_IRQHandler     //���¶���87���ж�ΪPIT2_IRQHandler�ж�

//#undef  VECTOR_107
//#define VECTOR_107    NRF_Handler     //���¶���107���ж�ΪNRF_Handler�ж�,����ģ��
//#undef  VECTOR_043
//#define VECTOR_043    SPI1_IRQHandler     //���¶���43���ж�Ϊspi1 Handler�ж�  ������

extern void   PORTE_IRQHandler();            //PORTD   extern void  LPT_Handler();
extern void   PIT0_IRQHandler();            //PIT0 ��ʱ�жϷ�����
extern void   PIT1_IRQHandler();            //PIT1 ��ʱ�жϷ�����
extern void   PIT2_IRQHandler();            //PIT2 ��ʱ�жϷ�����
extern void   PIT3_IRQHandler();            //PIT2 ��ʱ�жϷ�����
//extern void   SPI1_IRQHandler();            //spi1 ����ģ���жϷ�����
#endif  //__ISR_H

/* End of "isr.h" */
