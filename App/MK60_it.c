/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_it.c
 * @brief      山外K60 平台中断服务函数
 * @author     山外科技
 * @version    v5.0
 * @date       2017-06-26
 */

#include "MK60_it.h"
#include "common.h"
#include "include.h"


int time=0;
void PIT0_IRQHandler()
{
  PIT_Flag_Clear(PIT0);       //清中断标志位
  time++;
  Date_collect();  //采集电感值
  duoji_PID();
  duoji_control();
 if(time>Star_time)
{ 
  speed_control();
}
  
}


void DMA_CH1_Handler(void)
{
    
    DMA_IRQ_CLEAN(DMA_CH1);                             //清除通道传输中断标志位    (这样才能再次进入中断)
    DMA_EN(DMA_CH1);                                    //使能通道CHn 硬件请求      (这样才能继续触发DMA传输)
}
void DMA_CH2_Handler(void)
{
    
    DMA_IRQ_CLEAN(DMA_CH2);                             //清除通道传输中断标志位    (这样才能再次进入中断)
    DMA_EN(DMA_CH2);                                    //使能通道CHn 硬件请求      (这样才能继续触发DMA传输)
}



