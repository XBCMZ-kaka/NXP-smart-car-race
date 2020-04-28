#ifndef __IMAGE_PROCESS_H
#define __IMAGE_PROCESS_H

#include "common.h"
#include "include.h"

#define duty 626   //舵机中值

#define HANG             64        //行数
#define LIE              120       //列数
#define WHITE_dot        255       //白点
#define BLACK_dot        0         //黑点
void imageExtract(uint8 dst[HANG][LIE], uint8 *src);
void imageTwoLines(void);
void ImageCenterLine_Filte(void);
void duoji_control();
void duoji_PID(void);
void dingxiang(void);
void Shizi_deal(void);
void Huanwan_deal(void);
int absolute(int i);
void Image_process(void);
void barrier_deal(void);
uint8 searchLittleS(void);
uint8 calculateVariance(void);
void imageprocess(void);
void Shizi_judge(void);
void Huanwan_judge(void);
uint8 CalculateVariance(void);
float xie(int a[64]);
uint8 saidaotype(void);

#endif