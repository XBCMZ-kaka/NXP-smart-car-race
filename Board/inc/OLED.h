/*
 * File:		nokia.h
 * Purpose:		Serial Input/Output routines
 *
 */

#ifndef _OELD_H
#define _OELD_H

#include "MK60_gpio.h"
     
#define	Brightness	0xCF 
#define X_WIDTH     128
#define Y_WIDTH     64
#define HANG             64        //行数
#define LIE              120       //列数
/********************************************************************/
/*-----------------------------------------------------------------------
LCD_init          : OLED初始化

编写日期          ：2012-11-01
最后修改日期      ：2012-11-01
-----------------------------------------------------------------------*/
 extern uint8 lanzhou96x64[768];
 void OLED_Init(void);
 void OLED_CLS(void);
 void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_Print(uint8 x, uint8 y, uint8 ch[]);
 void Display(uint8 x,uint8 y,int16 i);
 void DisplayFloat(uint8 x,uint8 y,float i);
 void OLED_PutPixel(uint8 x,uint8 y);
 void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);
 void OLED_Set_Pos(uint8 x, uint8 y);
 void OLED_WrDat(uint8 data);
 void Draw_LibLogo(void);
 void Draw_Landzo(void);
 void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]);
 void OLED_Fill(uint8 dat);
 void Dly_ms(uint16 ms);
 void OLED_Draw_Point(unsigned char x,unsigned char y);
 void OLED_Write_dc(unsigned char data, unsigned char dc);
 void OLED_Draw_Line(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2);
 void displayImage(uint8 dst[HANG][LIE]);



/********************************************************************/

#endif
