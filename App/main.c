/*      
 * @brief      
 * @author     LDH
 * @interface definition

               @�����  A8, A9, A10, A11(FTM1,2);
               @�����  D7(FTM0_CH7)
               AD�ɼ���3·��B1,B2,B3
                       
               @��������D6(L), C7(R)
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
void INIT(void);   //��ʼ�� 
extern int time;
int Stop_flag=0;
void  main(void)
{ 
  DisableInterrupts;

   INIT();
    //LCD_str            (site,"Cam init OK!",FCOLOUR,BCOLOUR);
    //�����жϷ�����
    set_vector_handler(DMA1_VECTORn , DMA_CH1_Handler);     //����DMA���жϷ�����Ϊ DMA_CH1_Handler
    set_vector_handler(DMA2_VECTORn , DMA_CH2_Handler);     //����DMA���жϷ�����Ϊ DMA_CH2_Handler
    set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler);      //����PIT0��ʱ���жϷ�����Ϊ PIT0_IRQHandler  
    enable_irq(PIT0_IRQn); 
     
  AD_saomiao();   //��ȡ���ֵ���ڹ�һ��
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
   { OLED_Fill(0x00);}  //��������
  if(time>Stop_time&&gpio_get(PTC4)==0)
  {Stop_flag=1;}
 
 }
 
}
void INIT()
{
   /******��ʼ�����******/
   ftm_pwm_init(FTM0, FTM_CH7,50,duty);
   /******��ʼ�����******/
    ftm_pwm_init(FTM1, FTM_CH0,10000,0);            //PTA8  
    ftm_pwm_init(FTM1, FTM_CH1,10000,0);         //PTA9  �����
    ftm_pwm_init(FTM2, FTM_CH0,10000,0);            //PTA10     
    ftm_pwm_init(FTM2, FTM_CH1,10000,0);        //PTA11 �Ҳ���
    //��������ʼ��
  DMA_count_Init(DMA_CH1, PTD6, 0x7FFF, DMA_falling_keepon);//��
  DMA_count_Init(DMA_CH2, PTC7, 0x7FFF, DMA_falling_keepon);//��
  adc_init(AD1);
  adc_init(AD2);
  adc_init(AD3);
  adc_init(AD4);
  adc_init(GYRO);          
  gpio_init(PTB23, GPI, 1);  //���뿪�أ��£�������OLED
  gpio_init(PTB22, GPI, 1);  //���뿪�أ����ƻ�����ת
  gpio_init(PTB21, GPI, 1);  //���뿪�ء����ٶȵ�2
  gpio_init(PTB20, GPI, 1);  //���뿪�ء����ٶȵ�1
  gpio_init(PTA14, GPO, 1); //��1
  gpio_init(PTA15, GPO, 1);  //��2
  gpio_init(PTC4, GPI, 1);  //�ɻɹ�ͣ��
  OLED_Init();
  pit_init_ms(PIT0,3);     //����3ms���ж�
}

