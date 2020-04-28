#include "Car_control.h"
#include "include.h"

extern int  duty_steer;                        //������ʵ��PWM
extern int error_now;
extern float   duty_Alter;                  //����ı�ֵ  
extern int yuanhuan_flag,podao_flag;
extern int Stop_flag;
extern uint8 imgData[HANG][LIE];  
int  duty_motor_L=0,duty_motor_R=0;         //������ҵ��ʵ��PWM
int  Count_Left,Count_Right;                //���ұ�������ֵ
int  Count_Left_Last,Count_Right_Last;     //�ϴ����ұ�������ֵ
int GoalSpeed_L=0,GoalSpeed_R=0;    //����Ŀ���ٶ�
float basespeed;    //40,43,45��47 
int n,Error;
int  IncrementSpeed_L,IncrementSpeed_R; //������������
float ActualSpeed_L,ActualSpeed_R,ActualSpeed;       //����ʵ���ٶ�
float err_now_L=0,err_next_L=0,err_last_L=0;   //����������ε�ƫ��(L)
float err_now_R=0,err_next_R=0,err_last_R=0;   //����������ε�ƫ��(R)
void differentspeed();

/******�ٶȻ�ȡ*******/
void speed_get()
{
  
  Count_Left = DMA_count_get(DMA_CH1);
  Count_Right= DMA_count_get(DMA_CH2);
  
    DMA_count_reset(DMA_CH1);
    DMA_count_reset(DMA_CH2);
 // printf("%5d %5d\n",Count_Left,Count_Right);
  
   if(Count_Left>250)
     Count_Left=250;
   if(Count_Right>250)
     Count_Right=250;
    
    DMA_EN(DMA_CH1);
    DMA_EN(DMA_CH2);
    //printf("%5d %5d\n",Count_Left,Count_Right);
   Count_Left_Last=Count_Left;
   Count_Right_Last=Count_Right;
}
/******��ͬ�ٶ�******/
void differentspeed()
{
  if(gpio_get(PTB20)==1&&gpio_get(PTB21)==1)
  {basespeed=43;} 
  if(gpio_get(PTB20)==1&&gpio_get(PTB21)==0)
  {basespeed=40;}
  if(gpio_get(PTB20)==0&&gpio_get(PTB21)==1)
  {basespeed=37;}
  if(gpio_get(PTB20)==0&&gpio_get(PTB21)==0)
  {basespeed=35;}
  
  
  ActualSpeed_L=0.8*Count_Left+0.2*Count_Left_Last;
  ActualSpeed_R=0.8*Count_Right+0.2*Count_Right_Last;
//  ActualSpeed =(ActualSpeed_L+ActualSpeed_R)*0.5;
//  Error=ABS(error_now);
  
//  basespeed=Fuzzy_Update(&Dianji_duty,Error,ActualSpeed);
 
 if(yuanhuan_flag==1||podao_flag==1)                   //����
 { GoalSpeed_L=(int)(basespeed-3);
   GoalSpeed_R=(int)(basespeed-3);
 }
 else if(yuanhuan_flag==0&&podao_flag==0)
 {if(duty_Alter>=-10&&duty_Alter<=10)  //ֱ��
  {GoalSpeed_L=(int)(basespeed+3);
   GoalSpeed_R=(int)(basespeed+3);
  } 
 else if(duty_Alter>10&&duty_Alter<=55)              //��ת
  {GoalSpeed_L=(int)(basespeed-(duty_Alter-10)*0.1);
   GoalSpeed_R=(int)(basespeed+(duty_Alter-10)*0.1);
  }
 else if(duty_Alter<-10&&duty_Alter>=-55)              //��ת
  {GoalSpeed_L=(int)(basespeed-(duty_Alter+10)*0.1);
   GoalSpeed_R=(int)(basespeed+(duty_Alter+10)*0.1);
  }
 }
 if(Stop_flag==1)
 {Stop_car();}  
}
/******�ٶ�PID*******/         
void speed_PID(void)
{
  
  
  /*
  n=n+1;
  float integral_L,integral_R;
  float Kp_L=500,Ki_L=40,Kd_L=10;                        //������������֡�΢��ϵ��(L)
  float Kp_R=500,Ki_R=40,Kd_R=10;                        //������������֡�΢��ϵ��(R)
  
  ActualSpeed_L=0.8*Count_Left+0.2*Count_Left_Last;
  ActualSpeed_R=0.8*Count_Right+0.2*Count_Right_Last; 
  
  err_now_L=GoalSpeed_L-ActualSpeed_L;
  err_now_R=GoalSpeed_R-ActualSpeed_R;
  if(n<=6)
  {
  integral_L+=err_now_L;
  integral_R+=err_now_R;
  }
  else if(n>6)
  {
   n=0;
   integral_L=0;
   integral_R=0;
  }
 // printf("%d\n",n);
  IncrementSpeed_L=(int)(Kp_L*err_now_L+Ki_L*integral_L+Kd_L*(err_now_L-err_last_L));
  IncrementSpeed_R=(int)(Kp_R*err_now_R+Ki_R*integral_R+Kd_R*(err_now_R-err_last_R));
  
  duty_motor_L=IncrementSpeed_L;
  duty_motor_R=IncrementSpeed_R;
  
  err_last_L=err_now_L;
  err_last_R=err_now_R;
  */
 
  float Kp_L=80,Ki_L=4,Kd_L=2;                        //������������֡�΢��ϵ��(L)
  float Kp_R=80,Ki_R=4,Kd_R=2;                        //������������֡�΢��ϵ��(R)
  
  
  
  err_now_L=GoalSpeed_L-ActualSpeed_L;
  err_now_R=GoalSpeed_R-ActualSpeed_R;
    IncrementSpeed_L=(int)(Kp_L*(err_now_L-err_next_L)+Ki_L*err_now_L+Kd_L*(err_now_L-2*err_next_L+err_last_L));
    duty_motor_L+=IncrementSpeed_L;
  
  IncrementSpeed_R=(int)(Kp_R*(err_now_R-err_next_R)+Ki_R*err_now_R+Kd_R*(err_now_R-2*err_next_R+err_last_R));
  duty_motor_R+=IncrementSpeed_R;
  
  
  err_last_L=err_next_L;
  err_next_L=err_now_L;
  err_last_R=err_next_R;
  err_next_R=err_now_R;
 
  /*******�޷�******/
  if(duty_motor_L<0)
  {duty_motor_L=0; }
  if(duty_motor_L>7000)
  {duty_motor_L=7000; }
  if(duty_motor_R<0)
  {duty_motor_R=0; }
  if(duty_motor_R>7000)
  {duty_motor_R=7000; }
  
  
}
/******�ٶȿ���*******/
void speed_control()
{ 
  speed_get();
  differentspeed();
  speed_PID();
 // printf("%5d %5d\n",GoalSpeed_L,GoalSpeed_R);
 
  if(duty_motor_L>=0)
  {ftm_pwm_duty(FTM1, FTM_CH0,0);                     //PTA8  
   ftm_pwm_duty(FTM1, FTM_CH1,duty_motor_L);         //PTA9  �����
  }
  if(duty_motor_L<0)
  {ftm_pwm_duty(FTM1, FTM_CH0,-duty_motor_L);                     //PTA8  
   ftm_pwm_duty(FTM1, FTM_CH1,0);         //PTA9  �����
  }
  if(duty_motor_R>=0)
  {ftm_pwm_duty(FTM2, FTM_CH0,0);                   //PTA10     
   ftm_pwm_duty(FTM2, FTM_CH1,duty_motor_R);       //PTA11 �Ҳ���
  }
  if(duty_motor_R<0)
  {ftm_pwm_duty(FTM2, FTM_CH0,-duty_motor_R);                   //PTA10     
   ftm_pwm_duty(FTM2, FTM_CH1,0);       //PTA11 �Ҳ���
  }
 
}
void Stop_car()
{
    GoalSpeed_L=0;
    GoalSpeed_R=0;
}
/******�������*******/
void duoji_control()
{  
  
   ftm_pwm_duty(FTM0, FTM_CH7, duty_steer);
  
}




