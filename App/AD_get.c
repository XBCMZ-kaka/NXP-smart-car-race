/**********电感采集、滤波*******************/
#include "common.h"
#include "include.h"
#include "AD_get.h"
#define  NUM  4

void vcan_sendware(uint8 *wareaddr, uint32 waresize);

uint16 AD_value[3],AD_MAX,AD_V[3][NUM],AD_sum[3],ad[3],ad_value[3] ;
uint16 Date_L,Date_M,Date_R,Date_S;//左中右电感的值、竖电感的值
/***********扫电感最大值***************/
void AD_saomiao()
{
  for(int i=0;i<50;i++)
  {
        ad_value[0]=adc_once(AD1,  ADC_10bit);//左
        ad_value[1]=adc_once(AD2,  ADC_10bit);//中
        ad_value[2]=adc_once(AD3,  ADC_10bit);//右
     AD_MAX=MAX(ad_value[1],ad_value[0] );
     AD_MAX=MAX(AD_MAX,ad_value[2] );
     
     
  }     
 

}

/////////////////////获取电感值并进行滤波//////////////////////////
void Date_collect()
{
  uint16  i,j,k;
  uint16  ad_valu[3][5],ad_valu1[3],ad_sum[3];
  Date_S=adc_once(AD4,  ADC_10bit);//竖
   for(i=0;i<5;i++)
     {
         ad_valu[0][i]=adc_once(AD1, ADC_10bit);
         ad_valu[1][i]=adc_once(AD2, ADC_10bit);
         ad_valu[2][i]=adc_once(AD3, ADC_10bit);
     }
  ///////////////冒泡排序////////////
     for(i=0;i<3;i++)
     {
        for(j=0;j<5;j++)
        {
           for(k=0;k<4-j;k++)
           {  
              if(ad_valu[i][k] > ad_valu[i][k+1])  //如果前面的比后面的大  则进行交换
              {
                 SWAP(ad_valu[i][k],ad_valu[i][k+1]);
              }
           }
        }
     }
     for(i=0;i<3;i++)    //求中间三项的和
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }
   
 /////////////////滑动平均滤波//////////////////
     for(i = 0;i < NUM-1;i ++)
     {
         AD_V[0][i] = AD_V[0][i + 1];
         AD_V[1][i] = AD_V[1][i + 1];
         AD_V[2][i] = AD_V[2][i + 1];
           
        
     }

     for(i=0;i<3;i++)
     {
       if((ad_valu1[i]-AD_V[i][NUM-1])>(AD_MAX/2))
         ad_valu1[i]=AD_V[i][NUM-1];
     }
   
     for(i=0;i<3;i++)
     {
         AD_V[i][NUM-1] =  ad_valu1[i];
     }
     
     for(i = 0;i < NUM;i ++)
     {
      
         AD_sum[0] +=AD_V[0][i];///AD_sum[0] += xishu[i]*AD_V[0][i];
         AD_sum[1] +=AD_V[1][i];
         AD_sum[2] +=AD_V[2][i];
     
     }
     

     for(i=0;i<3;i++)  //求平均
     {
         AD_value[i] = AD_sum[i] / NUM;
         AD_sum[i] = 0;
     }
      
//Date_L=(int)(AD_value[0]*90/AD_MAX);
//Date_R=(int)(AD_value[2]*90/AD_MAX);
Date_L=(int)(AD_value[0]*100/AD_MAX);
Date_R=(int)(AD_value[2]*100/AD_MAX);
Date_M=(int)(AD_value[1]*100/AD_MAX);
Date_S=(int)(Date_S*100/AD_MAX);

//printf("%5d %5d %5d %5d\n",Date_L,Date_M,Date_R,Date_S);
//vcan_sendware((uint8_t *)ad, sizeof(ad));
}

//发送波形到上位机显示
//不同的上位机，不同的命令
void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
#define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //yy_摄像头串口调试 使用的命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //yy_摄像头串口调试 使用的命令

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令

    uart_putbuff(VCAN_PORT, wareaddr, waresize); //再发送图像

    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
}
