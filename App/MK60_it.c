/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_it.c
 * @brief      ɽ��K60 ƽ̨�жϷ�����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2017-06-26
 */

#include "MK60_it.h"
#include "common.h"
#include "include.h"


int time=0;
void PIT0_IRQHandler()
{
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
  time++;
  Date_collect();  //�ɼ����ֵ
  duoji_PID();
  duoji_control();
 if(time>Star_time)
{ 
  speed_control();
}
  
}


void DMA_CH1_Handler(void)
{
    
    DMA_IRQ_CLEAN(DMA_CH1);                             //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
    DMA_EN(DMA_CH1);                                    //ʹ��ͨ��CHn Ӳ������      (�������ܼ�������DMA����)
}
void DMA_CH2_Handler(void)
{
    
    DMA_IRQ_CLEAN(DMA_CH2);                             //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
    DMA_EN(DMA_CH2);                                    //ʹ��ͨ��CHn Ӳ������      (�������ܼ�������DMA����)
}



