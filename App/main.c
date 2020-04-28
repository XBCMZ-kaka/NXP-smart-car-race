/*      
 * @brief      
 * @author     LDH
 * @interface definition

               @电机：  A8, A9, A10, A11(FTM1,2);
               @舵机：  D7(FTM0_CH7)
               AD采集：3路：B1,B2,B3
                       
               @编码器：D6(L), C7(R)
               @OLED:   SCL  D5
                        SDA  D4
                        RST  D3
                        DC   D2
                        CS   D6
               @KEY:   B19, B20, B21, B22, B23, 

               

 * @date       2017-6-26
 */
#include "common.h"
#include "include.h"






void DMA_CH1_Handler();
void DMA_CH2_Handler();                                                                                                                                               
void PIT0_IRQHandler();
void INIT(void);   //初始化 
extern int time;
int Stop_flag=0;
void  main(void)
{ 
  DisableInterrupts;

   INIT();
    //LCD_str            (site,"Cam init OK!",FCOLOUR,BCOLOUR);
    //配置中断服务函数
    set_vector_handler(DMA1_VECTORn , DMA_CH1_Handler);     //设置DMA的中断服务函数为 DMA_CH1_Handler
    set_vector_handler(DMA2_VECTORn , DMA_CH2_Handler);     //设置DMA的中断服务函数为 DMA_CH2_Handler
    set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler);      //设置PIT0定时的中断服务函数为 PIT0_IRQHandler  
    enable_irq(PIT0_IRQn); 
     
  AD_saomiao();   //获取最大值用于归一化
  EnableInterrupts;
  while(1)
 { 
   
  // display(); 
  
   if(time< Warning_time)
   { display();}
   else if(time>Warning_time&&time<Star_time)
   { OLED_Fill(0x00);
     OLED_Print(30,2,"time over");
     OLED_Print(30,4,"time over");
   }
   else if(time>Star_time)
   { OLED_Fill(0x00);}  //起跑清屏
  if(time>Stop_time&&gpio_get(PTC4)==0)
  {Stop_flag=1;}
 
 }
 
}
void INIT()
{
   /******初始化舵机******/
   ftm_pwm_init(FTM0, FTM_CH7,50,duty);
   /******初始化电机******/
    ftm_pwm_init(FTM1, FTM_CH0,10000,0);            //PTA8  
    ftm_pwm_init(FTM1, FTM_CH1,10000,0);         //PTA9  左侧电机
    ftm_pwm_init(FTM2, FTM_CH0,10000,0);            //PTA10     
    ftm_pwm_init(FTM2, FTM_CH1,10000,0);        //PTA11 右侧电机
    //编码器初始化
  DMA_count_Init(DMA_CH1, PTD6, 0x7FFF, DMA_falling_keepon);//左
  DMA_count_Init(DMA_CH2, PTC7, 0x7FFF, DMA_falling_keepon);//右
  adc_init(AD1);
  adc_init(AD2);
  adc_init(AD3);
  adc_init(AD4);
  adc_init(GYRO);          
  gpio_init(PTB23, GPI, 1);  //拨码开关（下）：控制OLED
  gpio_init(PTB22, GPI, 1);  //拨码开关：控制环左右转
  gpio_init(PTB21, GPI, 1);  //拨码开关——速度档2
  gpio_init(PTB20, GPI, 1);  //拨码开关——速度档1
  gpio_init(PTA14, GPO, 1); //灯1
  gpio_init(PTA15, GPO, 1);  //灯2
  gpio_init(PTC4, GPI, 1);  //干簧管停车
  OLED_Init();
  pit_init_ms(PIT0,3);     //设置3ms的中断
}

