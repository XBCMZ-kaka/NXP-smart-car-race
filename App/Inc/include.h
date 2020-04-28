#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"
#define Star_time      5*333    // ��ʱ����5s
#define Warning_time   3*333    // 7s��������
#define Stop_time      15*333    // ������10S�ڲ�ͣ��
/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO�ڲ���
#include  "MK60_uart.h"     //����
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //�͹��Ķ�ʱ��(��ʱ)
#include  "MK60_i2c.h"      //I2C
//#include  "MK60_spi.h"      //SPI
#include  "MK60_ftm.h"      //FTM
#include  "MK60_pit.h"      //PIT
//#include  "MK60_rtc.h"      //RTC
#include  "MK60_adc.h"      //ADC
#include  "MK60_dac.h"      //DAC
#include  "MK60_dma.h"      //DMA
#include  "MK60_FLASH.h"    //FLASH
//#include  "MK60_can.h"      //CAN
//#include  "MK60_sdhc.h"     //SDHC
//#include  "MK60_usb.h"      //usb

#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_RTC_count.h"    //RTC ʱ��ת��
#include  "VCAN_LCD.h"          //Һ����ͷ�ļ�
#include  "AD_get.h"             //AD�ɼ�  
#include  "Date.h"               //����
#include  "OLED.h"              //OLED��ʾ
#include  "Car_control.h"       //С������
#include "Fuzzy.h"               //ģ��

#include  "vcan_img2sd.h"       //�洢ͼ��sd��һ���ļ�
#include  "vcan_sd_app.h"       //SD��Ӧ�ã���ʾsd����ͼƬ�̼���




#endif  //__INCLUDE_H__