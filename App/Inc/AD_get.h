#ifndef __AD_GET_H__
#define __AD_GET_H__
#include "common.h"
#include "include.h"

#define AD1     ADC0_SE9        //PTB1
#define AD2     ADC0_SE12       //PTB2
#define AD3     ADC0_SE13       //PTB3
#define AD4     ADC1_SE11       //PTB5
#define GYRO    ADC0_SE18       //陀螺仪E24  

void Date_collect(void);     //采集电感值
void AD_saomiao(void);      //初始扫描，获取最大值用于归一化

#endif  //__AD_GET_H__