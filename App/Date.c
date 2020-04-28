/**********各种参数*********/
#include "common.h"
#include "include.h"
#include "Date.h"


float Kp,Kd;//舵机的PD控制
extern uint16 Date_L,Date_M,Date_R,Date_S,AD_MAX;
extern uint16 ad_value[3];
uint16 date_sum,date_min,date_l,date_r;                                     //三水平电感之和
int error_now,error_last,error_provious;
float date_plus,date_minus,date_divide,date_minus_last;
int duty_steer=duty,duty_steer_last=duty;           //舵机实际值
int flag,error_change,yuanhuan_flag=0,podao=0,podao_flag=0,yuanhuan_time,podao_time;
int direction;
float duty_Alter=0,duty_Alter_last;            //舵机改变值 

//舵机模糊表
FuzzyStruct DuoJi_P={
 Fuzzy_Rank7,
 0,
 0,
 {-30,-20,-10,0,10,20,30},            //E
 {-6,-4,-2,0,2,4,6},                   //EC                   //EC
  
 
 {1.8,1.95,2.1,2.25,2.4,2.55,2.7},
 //{1.6,1.75,1.9,2.05,2.2,2.35,2.5},
  // {0.9,1.2,1.5,1.8,2.1,2.4,2.7},
 //{1.75,1.95,2.15,2.35,2.55,2.75,2.95},
 //{1.9,2,2.1,2.2,2.3,2.4,2.5},
  { 
    /*
    {6,5,4,3,3,2,1},
    {5,4,3,2,2,1,2},
    {4,3,2,1,1,2,3},
    {3,2,1,0,1,2,3},
    {3,2,1,1,2,3,4},
    {2,1,2,2,3,4,5},
    {1,2,3,3,4,5,6},
    */
       
      {6,6,5,4,4,4,4},                //横为E,竖为EC
      {6,5,4,3,2,3,4},
      {5,4,2,1,1,3,4},
      {4,3,1,0,1,3,4},
      {4,3,1,1,2,4,5},
      {4,3,2,3,4,5,6},
      {4,4,4,4,5,6,6},       
    
  }
} ;


void Direction()
{ 
  flag=0;
  PTA14_OUT=1;
  PTA15_OUT=1;
  /**********差和比***********/
  date_minus=Date_L-Date_R;    //左减右
  //date_minus=(int)(1.4*date_minus);    //左减右
  date_plus=Date_L+Date_R+1;    //左加右
  if(Date_M<33)
  {
   date_minus=date_minus_last*1.3;
  }
  date_divide=date_minus/date_plus;//差/和
  date_sum=Date_L+Date_R+Date_M;   //三水平电感之和
  date_l=Date_M-Date_L;
  date_r=Date_M-Date_R;
  //三电感最小值
  date_min=MIN(Date_L,Date_M);
  date_min=MIN(date_min,Date_R);
  
  
  error_now=(int)(date_divide*40);
  error_change=-(adc_once(GYRO, ADC_8bit)/4-31);
  direction=error_change+2;
  
  if(Date_M>90&&Date_L>60&&Date_R>60&&Date_M>Date_L&&Date_M>Date_R)  //直道
  flag=1;
  if(Date_L<45||Date_R<45)                                            //弯道
  flag=2;
  
  switch(flag)
  {
  case 0: error_now=(int)(1.0*error_now);break;
  case 1: error_now=(int)(0.8*error_now);break;
  case 2: error_now=(int)(1.3*error_now);break;
  }
  /***********识别圆环********/
  if(podao_flag==0&&yuanhuan_flag==0&&date_sum>90&&date_sum<150&&date_plus<100&&Date_M>25&&Date_M<55&&Date_M>Date_L&&Date_M>Date_R&&date_min>25&&Date_S<5&&direction==0)
  {
    yuanhuan_flag=1;
  }  
   
  if(yuanhuan_flag==1)
  {
    yuanhuan_time++;
    
     if(yuanhuan_time>jinhuan_TIME)
   {yuanhuan_flag=0;yuanhuan_time=0;}
   
  }
  /**********识别坡道*********/
 // if(podao_flag==0&&podao==0&&date_sum>320&&date_min>40&&Date_M>150)   //大坡
  if(podao_flag==0&&podao==0&&date_sum>290&&date_min>40&&Date_M>130)
  {
    podao_flag=1;
    podao=1;
  }  
   
  if(podao_flag==1)
  {
    podao_time++;
  
     if(podao_time>podao_TIME)
   {podao_flag=0;podao_time=0;} //坡道解除
   
  }
  
  
 //error_change=error_now-error_last;
 date_minus_last=date_minus;
}
void duoji_PID()
{ 
  Kd=6;
 // Kd=12;
  Direction();


  Kp=1.2*Fuzzy_Update(&DuoJi_P,error_now,error_change);
  //Kp=0.0016*error_now*error_now+1.5;
  //Kp=2.5;
 
  /**************特殊处理*********************/ 
  if(yuanhuan_flag==1)
  { 
     duty_Alter=55;
    // duty_Alter=-55; 
     PTA15_OUT=0;
       
  }
  else if(yuanhuan_flag==0)
  {
  if(date_sum>15)
  {
  duty_Alter=Kp*error_now+Kd*error_change;
  }
  else if(date_sum<=15&&date_sum>5)
  { 
    
    if(duty_steer_last>duty)  duty_Alter=55;
    else                      duty_Alter=-55;  
  }
  else if(date_sum<=5)
  {
    duty_Alter=duty_Alter_last;
  }
  
  }
  if(podao_flag==1)
  {
   // duty_Alter=0;
    PTA14_OUT=0;
  } 
  
    if(duty_Alter>55)
      duty_Alter=55;
    else if(duty_Alter<-55)
      duty_Alter=-55;
    
     duty_steer=(int)(duty+duty_Alter);
     /*
     //防止舵机突变
     if(duty_steer-duty_steer_last>20)
       duty_steer=duty_steer_last+20;
     if(duty_steer-duty_steer_last<-20)
       duty_steer=duty_steer_last-20;
 */
     error_last=error_now;
     duty_Alter_last=duty_Alter;
     duty_steer_last=duty_steer;
}

void display()
{
 /**********在OLED上显示************/

if(gpio_get(PTB23)==0)
{
Display(30,0,AD_MAX);
Display(30,2,error_now);
Display(30,4,error_last);
Display(30,6,direction);
}
  
if(gpio_get(PTB23)==1)
{
Display(30,0,Date_L);
Display(30,2,Date_M);
Display(30,4,Date_R);
Display(30,6,Date_S);
//Display(30,6,error_now);
}
//printf("%5d %5d\n",error_now,error_change);
}


