#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"
#define Star_time      5*333    // 延时发车5s
#define Warning_time   3*333    // 7s发车警告
#define Stop_time      15*333    // 发车后10S内不停车
/*
 * Include 用户自定义的头文件
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO口操作
#include  "MK60_uart.h"     //串口
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //低功耗定时器(延时)
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
#include  "VCAN_RTC_count.h"    //RTC 时间转换
#include  "VCAN_LCD.h"          //液晶总头文件
#include  "AD_get.h"             //AD采集  
#include  "Date.h"               //参数
#include  "OLED.h"              //OLED显示
#include  "Car_control.h"       //小车控制
#include "Fuzzy.h"               //模糊

#include  "vcan_img2sd.h"       //存储图像到sd卡一个文件
#include  "vcan_sd_app.h"       //SD卡应用（显示sd看上图片固件）




#endif  //__INCLUDE_H__
