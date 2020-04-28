/****************************************************************************
Copyright: --------------湖北工业大学--启明星辰（力创团队）------------
FileName:        image_process.c,增量
Author:           liudeheng 
Date:             2017-2-13
Description:      图像处理，提取中线求偏差 控制舵机打角
			
Function List:	    Road_Type imageProcess();                       
						

****************************************************************************/
#include "Image_process.h"


const uint8  separateLines[HANG]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,   //15hang

                                  16,17,19,20,22,23,25,26,28,29,
                                  31,32,34,35,37,38,40,41,43,44,        //20hang
                                 
                                  46,48,50,52,54,56,58,60,62,64,66,68,70,72,74, //15hang
                                
                                  77,80,83,
                                  86,89,92,
                                  95,98,101,104,      //10hang
                                  
                                  108,112,116,119            //4hang
                                  };
/*
const uint8  separateLines[HANG]={2,4,6,8,10,12,14,16,18,20,22,24,26,28,   //14hang

                                  30,32,34,36,38,40,42,44,46,48,
                                  50,52,53,54,56,57,58,59,60,61,        //20hang
                                 
                                  62,63,64,66,68,70,72,74,76,78,80,82,84,86,88, //15hang
                                
                                  90,92,94,
                                  96,98,100,
                                  102,104,106,108,      //10hang
                                  
                                  110,112,114,116,118            //5hang
                                  };
*/
extern uint8  imgData[HANG][LIE];   
uint8 out_flag;    //停车标志位
uint8 right_barrier_flag=0,left_barrier_flag=0;//障碍标志位
uint8 Zhidao_flag=0,Zhidao=0;     //直道标志位
uint8 LittleS_flag=0,preS_flag=0;          //小S标志位

float error_now=0;            //当前的偏差
float error_last=0;          //前一次的偏差
float   duty_Alter=0;            //舵机改变值  
int duty_steer=duty;     //舵机实际值
int duty_steer_last=duty;        //上次舵机值
int chazhi,chazhibian,N,M,Q;
int stop1=0,stop2=0,dianflag=1,count=0;
int Yuanhuan=0;
uint8 stop_flag=0,stopjudge_flag=0;
uint8 turn_L=0,turn_R=0;

uint8 L_lost1[HANG],R_lost1[HANG],Middle_line[HANG];  //巡双线用到
int16  varianceTemp;  //偏差绝对值和
int16  VarianceTemp;
int RoadWidth,Shizi_L=0,Shizi_R=0,Huanwan_L=0,Huanwan_R=0;
uint8  shizi_type1=0,shizi_type2=0;
int Middle=61,effectiveROW,BlackEndM=0;
int Image_L[HANG]={0},Image_R[HANG]={0};
/*
uint8 effectiveLine[20]={29,29,30,30,31,31,32,32,33,33,
                         34,34,35,35,36,36,37,37,38,38};
*/

uint8 effectiveLine[20]={35,35,36,36,37,37,38,38,38,38,
                         39,39,39,39,40,40,41,41,42,42};
/*
uint8 effectiveLine[20]={33,33,33,33,34,34,34,34,35,35,
                         35,35,36,36,36,36,37,37,37,37};
*/
//舵机模糊表
FuzzyStruct DuoJi_P={
 Fuzzy_Rank7,
 0,
 0,
 {-30,-20,-10,0,10,20,30},            //E
 {-6,-4,-2,0,2,4,6},                   //EC                   //EC
  
 //{2.05,2.15,2.25,2.45,2.65,2.75,2.85},
 //{1.95,2.1,2.25,2.4,2.5,2.6,2.7},   
 //{2,2.15,2.3,2.45,2.6,2.7,2.8},
 //{2,2.15,2.3,2.45,2.55,2.65,2.75},//----
 //{2,2.13,2.26,2.39,2.52,2.65,2.78},
 //{1.9,2.0,2.1,2.2,2.3,2.4,2.5},
 {2,2.12,2.24,2.36,2.48,2.6,2.72},
 //{2,2.11,2.22,2.33,2.44,2.55,2.66},
 
 //----------------------
 //{2,2.14,2.28,2.32,2.46,2.60,2.74},
 //{2,2.13,2.26,2.29,2.42,2.55,2.68},
 
  {  
       /*E*/
 /*C*/{6,6,5,4,4,4,4},                //横为E,竖为EC
      {6,5,4,3,2,3,4},
      {5,4,2,1,1,3,4},
      {4,3,1,0,1,3,4},
      {4,3,1,1,2,4,5},
      {4,3,2,3,4,5,6},
      {4,4,4,4,5,6,6},       
  }
} ;


int absolute(int i)   //取绝对值函数
 {
  if(i<0) 
  {
    i=0-i;
  } 
  return i;  
 }

/****************************************
----------------环弯处理-----------------
*****************************************/
void Huanwan_deal(void)
{
  int i,guaidian_l=0,guaidian_r=0,k1,k2; //左右拐点
  uint8 Huanwan_L_flag=0,Huanwan_R_flag=0;
  
  for(i=59;i>20;i--)
  {
      //找环弯左边线拐点
     if((Image_L[i+1]-Image_L[i])*(Image_L[i]-Image_L[i-1])<=0&&!((Image_L[i+1]-Image_L[i])==0&&(Image_L[i]-Image_L[i-1])==0))
     {
         k1=Image_L[i+3]-Image_L[i];
         k2=Image_L[i]-Image_L[i-3];
         if((k1*k2<0)&&absolute(k1-k2)>5)
         {
             Huanwan_L_flag=1;
             guaidian_l=i;
            
         }
        
     }
     //找环弯右边线拐点
     if((Image_R[i+1]-Image_R[i])*(Image_R[i]-Image_R[i-1])<=0&&!((Image_R[i+1]-Image_R[i])==0&&(Image_R[i]-Image_R[i-1])==0))
     {
         k1=Image_R[i+3]-Image_R[i];
         k2=Image_R[i]-Image_R[i-3];
       if((k1*k2<0)&&absolute(k1-k2)>5)
         {
             Huanwan_R_flag=1;
             guaidian_r=i;
            
          }
     }     
  }
  
  //补环弯右边线
  if(Huanwan_R_flag==1)
  {
     uint16 x_R_sum=0,x2_R_sum=0,y_R_sum=0,xy_R_sum=0,size;
     float k_R,b_R;
     for(i=63;i>guaidian_r;i--)
     {
        x_R_sum+=i;
        x2_R_sum+=i*i;
        y_R_sum+=Image_R[i]-56;
        xy_R_sum+=(Image_R[i]-56)*i;           
     }
     size=63-guaidian_r;
     
     k_R=(float)(size*xy_R_sum-x_R_sum*y_R_sum)/(float)(size*x2_R_sum-x_R_sum*x_R_sum);
     b_R=(float)(x2_R_sum*y_R_sum-xy_R_sum*x_R_sum)/(float)(size*x2_R_sum-x_R_sum*x_R_sum);
      
     for(i=guaidian_r;i>15;i--)
     {
        Image_R[i]=(int)(k_R*i+b_R+50);
        if(Image_R[i]<2)
          Image_R[i]=2;
        else if(Image_R[i]>110)
          Image_R[i]=110;
        
      //  imgData[i][Image_R[i]]=128;
     }
  }
  
  
    
  for(i=63;i>15;i--) 
  {
     Middle_line[i]=(Image_L[i]+Image_R[i])/2-25; 
    if( Middle_line[i]<3)
     Middle_line[i]=3;  
  }
}



/****************************************
----------------十字处理-----------------
*****************************************/
void Shizi_deal(void)
{
  shizi_type2=0;
  int i,guaidian_L=0,guaidian_R=0,k1,k2; //左右拐点
  uint8 Shizi_L_flag=0,Shizi_R_flag=0;
  
  for(i=59;i>10;i--)
  {
      //找十字左边线拐点
     if((Image_L[i+1]-Image_L[i])*(Image_L[i]-Image_L[i-1])<=0&&!((Image_L[i+1]-Image_L[i])==0&&(Image_L[i]-Image_L[i-1])==0))
     {
         k1=Image_L[i+3]-Image_L[i];
         k2=Image_L[i]-Image_L[i-3];
         if((k1*k2<0)&&absolute(k1-k2)>5)
         {
             Shizi_L_flag=1;
             guaidian_L=i;
            
         }
        
     }
     //找十字右边线拐点
     if((Image_R[i+1]-Image_R[i])*(Image_R[i]-Image_R[i-1])<=0&&!((Image_R[i+1]-Image_R[i])==0&&(Image_R[i]-Image_R[i-1])==0))
     {
         k1=Image_R[i+3]-Image_R[i];
         k2=Image_R[i]-Image_R[i-3];
       if((k1*k2<0)&&absolute(k1-k2)>5)
         {
             Shizi_R_flag=1;
             guaidian_R=i;
            
          }
     }     
  }
  
  //补十字左边线
  if(Shizi_L_flag==1)
  {
     uint16 x_L_sum=0,x2_L_sum=0,y_L_sum=0,xy_L_sum=0,size;
     float k_L,b_L;
     for(i=63;i>guaidian_L;i--)
     {
        x_L_sum+=Image_L[i];
        x2_L_sum+=Image_L[i]*Image_L[i];
        y_L_sum+=i;
        xy_L_sum+=Image_L[i]*i;    
     }
     size=63-guaidian_L;
     
     k_L=(float)(size*xy_L_sum-x_L_sum*y_L_sum)/(float)(size*x2_L_sum-x_L_sum*x_L_sum);
     b_L=(float)(x2_L_sum*y_L_sum-xy_L_sum*x_L_sum)/(float)(size*x2_L_sum-x_L_sum*x_L_sum);

     for(i=guaidian_L;i>15;i--)
     {
        //Image_L[i]=(int)(k_L*i+b_L);
        Image_L[i]=(int)(1.0/k_L*(i-b_L));
        if(Image_L[i]<2)
          Image_L[i]=2;
        else if(Image_L[i]>110)
          Image_L[i]=110;
        
       imgData[i][Image_L[i]]=128;
     }
  }  
//补十字右边线
  if(Shizi_R_flag==1)
  {
     uint16 x_R_sum=0,x2_R_sum=0,y_R_sum=0,xy_R_sum=0,size;
     float k_R,b_R;
     for(i=63;i>guaidian_R;i--)
     {
        x_R_sum+=i;
        x2_R_sum+=i*i;
        y_R_sum+=Image_R[i]-56;
        xy_R_sum+=(Image_R[i]-56)*i;           
     }
     size=63-guaidian_R;
     
     k_R=(float)(size*xy_R_sum-x_R_sum*y_R_sum)/(float)(size*x2_R_sum-x_R_sum*x_R_sum);
     b_R=(float)(x2_R_sum*y_R_sum-xy_R_sum*x_R_sum)/(float)(size*x2_R_sum-x_R_sum*x_R_sum);
      
     for(i=guaidian_R;i>15;i--)
     {
        Image_R[i]=(int)(k_R*i+b_R+56);
        if(Image_R[i]<2)
          Image_R[i]=2;
        else if(Image_R[i]>110)
          Image_R[i]=110;
        
       imgData[i][Image_R[i]]=128;
     }
  }
  
  if((Shizi_L_flag==1)||(Shizi_R_flag==1))
      shizi_type2=1;
    
  for(i=63;i>15;i--) 
  {
     Middle_line[i]=(Image_L[i]+Image_R[i])/2; 
    // Middle_line[i]=(int)(Middle_line[i]*0.5+Middle_line[i+1]*0.25+15);
  }
}
/****************************************
----------------判断环弯---------------
*****************************************/
void Huanwan_judge(void)
{
  Yuanhuan=0;
 if(imgData[22][30]==255&&imgData[22][31]==255&&imgData[22][60]==0&&imgData[22][62]==0&&imgData[22][90]==255&&imgData[22][91]==255&&imgData[21][31]==255&&imgData[21][30]==255&&imgData[21][60]==0&&imgData[21][62]==0&&imgData[21][90]==255&&imgData[21][91]==255)
  {
    Yuanhuan=1;
    Huanwan_deal();
    gpio_set(PTB22,1);
      
   }
     else
   {
       gpio_set(PTB22,0);
       Yuanhuan=0;
   }
  
}




/****************************************
----------------判断十字----------------
*****************************************/
void Shizi_judge(void)
{
  uint8  shizi_type1=0;
  int i,j;
  
   for(i=40;i<44;i=i+2)   //判断是否为前面几行均为全白的十字类型
   {
      for(j=20;j<100;j=j+2)
      {
         if(imgData[i][j]!=WHITE_dot)
             break;
         else if(j==100)
             shizi_type1=1;
      }
   }
   
   if(shizi_type1==1) 
      return;
   
   else if((Shizi_L>2)&&(Shizi_R>2))
     Shizi_deal();
      
}

/****************************************
----------------判断直道----------------
*****************************************/
uint8 CalculateVariance(void)
{ VarianceTemp = 0;//赋初值
  int r;
  
  
  for(r = 13;r < HANG - 5;r++)
  {
    VarianceTemp += absolute((Middle_line[r] - 56)); 
  }
 // OLED_P6x8int(0, 1, VarianceTemp, 0);
  if(VarianceTemp < 100) 
    return 1;            //直道高速
  else if(VarianceTemp < 300)
    return 2;            //直道高速
  else
    return 0;           //弯道低速
}

 /****************************************
----------------判断小S----------------
*****************************************/
uint8 calculateVariance(void)
{

   int16 sum=0,varianceTemp=0,lastTemp=0;  //偏差绝对值和
   int r,c;
  
  for(r = 19;r < HANG ; r++)
  {
    sum += (Middle_line[r]);
  }
  for(r = 19;r < HANG ; r++)
  {
    varianceTemp += (int)(((Middle_line[r] - sum*0.022)*(Middle_line[r] - sum*0.022))*0.1); 
  }
  c=varianceTemp-lastTemp;
  lastTemp=varianceTemp;
 // OLED_P6x8int(30, 2,c, 0);
 if(c< 100) 
    return 1;            //直道高速
  if(c < 250)
    return 2;            //直道高速
  else
    return 0;           //弯道低速
    
  
}


/****************************************
----------------判断小S----------------
*****************************************/
uint8 searchLittleS(void)
{
   int16 diffTemp;
   int leftTemp=0;
   int rightTemp=0;
   int r;
   
   for(r=(HANG-5);r>10;r--)
   {
     diffTemp = Middle_line[r] - Middle_line[r+1];
     if((diffTemp>0)&&(diffTemp<2))
     {
       rightTemp++;
     }
     else if((diffTemp<0)&&(diffTemp>-2))
     {
       leftTemp++;
     }   
   }
 //  OLED_P6x8int(30, 0, rightTemp, 0);
  // OLED_P6x8int(0, 0, leftTemp, 0);
   if((leftTemp>3)&&(rightTemp>3))
    {
      //gpio_turn(PTE0);
      return 1;
    }
    else
    {
      return 0;
    }   
}


/****************************************
-------------急弯重新巡线-------------
*****************************************/
void imageprocess(void)
{
  int r,j;
  for(r=(HANG-5);r>15;r--)
  {
    if((L_lost1[r]==1)&&(R_lost1[r]==1))
       RoadWidth=Image_R[r]-Image_L[r];
    
    else if((L_lost1[r]==0)&&(R_lost1[r]==1))
    {
      if(Image_R[r]>RoadWidth/2)
        Middle_line[r]=Image_R[r]-RoadWidth/2;
      else if(Image_R[r]<=RoadWidth/2)
        Middle_line[r]=2;
    }
    
    else if((L_lost1[r]==1)&&(R_lost1[r]==0))
    {
      if((Image_L[r]+RoadWidth/2)<=112)
        Middle_line[r]=Image_L[r]+RoadWidth/2;
      else if((Image_L[r]+RoadWidth/2)>112)
        Middle_line[r]=110;
    }  
    
    else if((L_lost1[r]==0)&&(R_lost1[r]==0))
       Middle_line[r]=Middle_line[r+1];
    
    if(Middle_line[r]<16)
    {
      for(j=r-1;j>15;j--)
      {
        Middle_line[j]=16;
      }
      return;
    }
    
    else if(Middle_line[r]>104)
    {
      for(j=r-1;j>15;j--)
      {
        Middle_line[j]=104;
      }
      return;
    }
    
  }
}

/****************************************
----------------障碍处理-----------------
*****************************************/
void barrier_deal(void)
{
  int i,j,left_BLACK_dot=0,right_BLACK_dot=0;
  for(i=25;i<60;i=i+2)
  {
    for(j=Middle_line[i]+1;j<Middle_line[i]+16;j=j+2)
    {
      if(imgData[i][j]==BLACK_dot)
        right_BLACK_dot++;
    }
    
    for(j=Middle_line[i]-1;j>Middle_line[i]-16;j=j-2)
    {
      if(imgData[i][j]==BLACK_dot)
        left_BLACK_dot++;
    }
  }
  if((right_BLACK_dot<5)||(left_BLACK_dot<5))     //防止和起跑线误判
  {
    if(right_BLACK_dot-left_BLACK_dot>5)
    {
      right_barrier_flag=1;
      //gpio_turn(PTE0);
    }
    else if(left_BLACK_dot-right_BLACK_dot>5)
    {
      left_barrier_flag=1;
      //gpio_turn(PTE1);
    }
  }

}


uint16 max(uint16 a,uint16 b)
{
  uint16 s;
  if(a<b)
    s=b;
  else 
    s=a;
  return s;
}


/****************************************
----------判断最大有效行与定向-----------
*****************************************/
void dingxiang(void)
{
  int BlackEndL=0,BlackEndR=0,BlackEndMax=0;
  uint8 M_flag=0,L_flag=0,R_flag=0;
    turn_L=0,turn_R=0,BlackEndM=0;
    
  for(int i=HANG-1;i>0;i--)
  {
    //1/2处
    if((imgData[i][62]==WHITE_dot)&&(!M_flag))
     {
	BlackEndM++;
     }
    else if(((i+2)>0)&&(imgData[i-1][62]==BLACK_dot)&&(imgData[i-2][62]==BLACK_dot))
     {
        M_flag=1;
     }
    //右 1/4处
    if((imgData[i][32]==WHITE_dot)&&(!L_flag))
     {
	BlackEndL++;
     }
    else if(((i+2)>0)&&(imgData[i-1][32]==BLACK_dot)&&(imgData[i-2][32]==BLACK_dot))
     {
        L_flag=1;
     }
    //左 3/4处
    if((imgData[i][92]==WHITE_dot)&&(!R_flag))
     {
	BlackEndR++;
     }
    else if(((i+2)>0)&&(imgData[i-1][92]==BLACK_dot)&&(imgData[i-2][92]==BLACK_dot))
     {
        R_flag=1;
     }
  }
  
  /*
  if(stopjudge_flag==1)                           //停车处理
  {
     if(BlackEndM>50)
     {
       for(int i=HANG-1;i>0;i--)
       {
          if(imgData[i][38]==WHITE_dot)
           {
	     QL++;
           }
          else 
            break;
       }
      for(int i=HANG-1;i>0;i--)
       {
          if(imgData[i][74]==WHITE_dot)
           {
	     QR++;
           }
          else 
            break;
       }
       
       if((QL<8)&&(QR<8))
           stop_flag=1;
     }
  }
  */
  if(BlackEndL-BlackEndR>=5)
     turn_L=1;
  else if(BlackEndL-BlackEndR<=-5)
     turn_R=1;
  //printf("%5d %5d %5d\n",BlackEndL,BlackEndM,BlackEndR);
  BlackEndMax=max(BlackEndL,BlackEndM);
  BlackEndMax=max(BlackEndMax,BlackEndR);

  effectiveROW=BlackEndMax;

/*  OLED_P6x8int(0, 3,  QL, 0);
  OLED_P6x8int(30, 3,  QR, 0);  
  
  OLED_P6x8int(0, 1,  BlackEndL, 0);
  OLED_P6x8int(30, 1, BlackEndM, 0);
  OLED_P6x8int(60, 1, BlackEndR, 0);
  OLED_P6x8int(90, 1, effectiveROW, 0);*/
}



/****************************************
----------------双边巡线函数--------------
*****************************************/
void imageTwoLines(void)
{
  right_barrier_flag=0;      //障碍判断标志
  left_barrier_flag=0;
  LittleS_flag=0;
  Zhidao_flag=0;
  Shizi_L=0,Shizi_R=0;
  Huanwan_L=0,Huanwan_R=0;
  int i,j,a;
  //----前5行，从中间向两边检测----// 
  for(i=HANG-1;i>=HANG-4;i--)
  {
     L_lost1[i]=0;
     R_lost1[i]=0;
     //左线检测
     for(j=Middle;j>=3;j--)
     {
       if((imgData[i][j]==WHITE_dot)&&(imgData[i][j-1]==WHITE_dot)&&(imgData[i][j-2]==BLACK_dot)&&(imgData[i][j-3]==BLACK_dot))
       {
           Image_L[i]=(unsigned char)(j-2);
           L_lost1[i]=1;
           break;
       } 
     }
     if(L_lost1[i]==0)
     {
        Image_L[i]=2;
     }
     //右线检测 
     for(j=Middle;j<=109;j++)
     {
       if((imgData[i][j]==WHITE_dot)&&(imgData[i][j+1]==WHITE_dot)&&(imgData[i][j+2]==BLACK_dot)&&(imgData[i][j+3]==BLACK_dot))
       {
           Image_R[i]=(unsigned char)(j+2);
           R_lost1[i]=1;
           break;
       } 
     } 
    if(R_lost1[i]==0)
     {
        Image_R[i]=110;
     }
     Middle_line[i]=(Image_L[i]+Image_R[i])/2;
     RoadWidth=Image_R[i]-Image_L[i];
     if(RoadWidth>84)
        RoadWidth=84;
  }
 //后59行找中线，在前一行的基础上找 
  for(i=HANG-2;i>=0;i--)
  {
     L_lost1[i]=0;
     R_lost1[i]=0;
     //左线检测
     //上一行没有找到左边线
     if(L_lost1[i+1]==0)
     {
        for(j=Middle_line[i+1];j>3;j--)
         {
          if((imgData[i][j]==WHITE_dot)&&(imgData[i][j-1]==WHITE_dot)&&(imgData[i][j-2]==BLACK_dot)&&(imgData[i][j-3]==BLACK_dot))
           {
           Image_L[i]=(unsigned char)(j-2);
           L_lost1[i]=1;
           break;
           } 
         }     
     }
     //上一行找到左边线
     else if(L_lost1[i+1]==1)
     {
        for(j=Image_L[i+1]+5;(j>Image_L[i+1]-5)&&(j>3);j--)
        {
          if((imgData[i][j]==WHITE_dot)&&(imgData[i][j-1]==WHITE_dot)&&(imgData[i][j-2]==BLACK_dot)&&(imgData[i][j-3]==BLACK_dot))
          {
             Image_L[i]=(unsigned char)(j-2);
             L_lost1[i]=1;
             break;
          } 
        }
     } 
     if(L_lost1[i]==0)
     {
        Image_L[i]=2;
        if((i<55)&&(i>20))
        {
           for(a=30;a>=0;a=a-2)
           {
              if(imgData[i][a]!=WHITE_dot)
                break;
              else if(a==0)
                Shizi_L++;
                Huanwan_L++;
           }
        }
     }
     //右线检测 
     //上一行右边线没有找到
     if(R_lost1[i+1]==0)
     {
        for(j=Middle_line[i+1];j<109;j++)
        {
           if((imgData[i][j]==WHITE_dot)&&(imgData[i][j+1]==WHITE_dot)&&(imgData[i][j+2]==BLACK_dot)&&(imgData[i][j+3]==BLACK_dot))
           {
             Image_R[i]=(unsigned char)(j+2);
             R_lost1[i]=1;
             break;
           } 
        }      
     }
     //上一行右边线找到
     else if(R_lost1[i+1]==1)
     {
        for(j=Image_R[i+1]-5;(j<Image_R[i+1]+5)&&(j<109);j++)
        {
           if((imgData[i][j]==WHITE_dot)&&(imgData[i][j+1]==WHITE_dot)&&(imgData[i][j+2]==BLACK_dot)&&(imgData[i][j+3]==BLACK_dot))
           {
              Image_R[i]=(unsigned char)(j+2);
              R_lost1[i]=1;
              break;
           } 
        }
     }
    if(R_lost1[i]==0)
     {
        Image_R[i]=110;
        if((i<55)&&(i>20))
        {
           for(a=82;a<=112;a=a+2)
           {
              if(imgData[i][a]!=WHITE_dot)
                break;
              else if(a==112)
                Shizi_R++;
                Huanwan_R++;
           }
        }
           
     }
     //两边都没有找到边线
    if((L_lost1[i]==0)&&(R_lost1[i]==0))
     {
        Middle_line[i]=Middle_line[i+1];
     }
    else
     {
        Middle_line[i]=(Image_L[i]+Image_R[i])/2;    
     }
    
    imgData[i][Middle_line[i]]=128;
    //imgData[i][Image_L[i]]=128;
   // imgData[i][Image_R[i]]=128;
  }
  
 
  
      Shizi_judge();
   //   Huanwan_judge();
  
      
   if(effectiveROW<50)
  {   imageprocess(); }    
   
 // ImageCenterLine_Filte();//对中心线进行滤波
 /*
  //对障碍进行处理
  if(effectiveROW>58)
   {
     if(Image_R[15]-Image_L[15]>=30)
        barrier_deal(); 
   }
  if((effectiveROW>45)&&(right_barrier_flag==0)&&(left_barrier_flag==0))
   {
    
     preS_flag=searchLittleS();
     if(preS_flag==1)
     {
       LittleS_flag=calculateVariance();
     
      if((BlackEndM>60)&&(LittleS_flag==1))
         LittleS_flag=1;
      else
         LittleS_flag=2;
     }
   }    
   
  if((effectiveROW>48)&&(right_barrier_flag==0)&&(left_barrier_flag==0)&&(LittleS_flag==0))
   {
       Zhidao=CalculateVariance();
      if((BlackEndM>60)&&(Zhidao==1))
         Zhidao_flag=1;
      else if(BlackEndM>50)
      {
          if(Zhidao==1)
            Zhidao_flag=2;
          else if(Zhidao==2)
            Zhidao_flag=3; 
      }
   }   
 
  if(effectiveROW<=5)
    out_flag=1;
 */
}


void duoji_PID(void)
{
    float y,Kp=2.2,Kd=8;        //设定舵机的PD
    float center_Set=60;          //设定中线
    float center_Actual;         //实际中线
    int i;
  if(effectiveROW>=20)
  {  
    if(effectiveROW>=45)
    {
      for(i=0;i<20;i++)
      {
        y+=Middle_line[effectiveLine[i-1]];
      }
    
    }
    if(effectiveROW>=30&&effectiveROW<45)
    {
      for(i=0;i<20;i++)
      {
        y+=Middle_line[effectiveLine[i-1]];
      }
    }
    if(effectiveROW>=20&&effectiveROW<30)
    {
      for(i=0;i<20;i++)
      {
        y+=Middle_line[effectiveLine[i-1]];
      }
    }
    
     center_Actual=y*0.05;
     error_now=center_Set-center_Actual;
     //Kp=Fuzzy_Update(&DuoJi_P,error_now,error_now-error_last);
     duty_Alter=Kp*error_now+Kd*(error_now-error_last);
     error_last=error_now;
             
 } 
 else if (effectiveROW<20)     //超过最小有效行，锁死
    {
      if(turn_L==1)
        duty_Alter=(76+effectiveROW*0.2);
      else if(turn_R==1)
         duty_Alter=-(76+effectiveROW*0.2);
    }
   error_last=error_now;
    if(duty_Alter>80)
      duty_Alter=80;
    else if(duty_Alter<-80)
      duty_Alter=-80;
    
     duty_steer=(int)(duty+duty_Alter);
     //防止舵机突变
     if(duty_steer-duty_steer_last>20)
       duty_steer=duty_steer_last+20;
     if(duty_steer-duty_steer_last<-20)
       duty_steer=duty_steer_last-20;
    //chazhibian=duty_Alter-chazhi;    
    //chazhi=duty_Alter;

   // OLED_P6x8int(0, 5, fudu, 0);
    duty_steer_last=duty_steer;   
}


/******图像处理*******/
void Image_process(void)
{
 
   dingxiang();
   imageTwoLines(); 
   duoji_PID();
   displayImage(imgData);
   
}


/***************************************************************************/
/* *  @brief     二值压缩图像解压 
 *  @param      dst            
 *  @param      src                    
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    
 ***************************************************************************/
void imageExtract(uint8 dst[HANG][LIE], uint8 *src)
{
    uint8 ii,jj,column;  
    uint8 colour[2] = {255,0}; 
    //鹰眼0表示白 1表示黑      解压后，255表示白，0表示黑  
    uint8 tmpsrc;
     
    for(ii=0;ii<HANG;ii++)
    {
      column=0;
      for(jj=0;jj<LIE;jj+=6)
        {
          tmpsrc = *(src+20*separateLines[ii]+column);
          column++;
          dst[ii][jj]   = colour[ (tmpsrc >> 6 ) & 0x01 ];
          dst[ii][jj+1] = colour[ (tmpsrc >> 5 ) & 0x01 ];
          dst[ii][jj+2] = colour[ (tmpsrc >> 4 ) & 0x01 ];
          dst[ii][jj+3] = colour[ (tmpsrc >> 2 ) & 0x01 ];
          dst[ii][jj+4] = colour[ (tmpsrc >> 1 ) & 0x01 ];
          dst[ii][jj+5] = colour[ (tmpsrc >> 0 ) & 0x01 ];
	}
    }
				
}


/****************************************
----------------赛道类型----------------
*****************************************/
 /*
uint8 saidaotype()
{
   int i;
   float k_L=0.0,x_L_E=0.0,y_L_E=0.0,X2_L_sum=0.0,XY_L_sum=0.0, 
         k_R=0.0,x_R_E=0.0,y_R_E=0.0,X2_R_sum=0.0,XY_R_sum=0.0,
         k_M=0.0,x_M_E=0.0,y_M_E=0.0,X2_M_sum=0.0,XY_M_sum=0.0; 
  // k1= xie(Image_L);//左边界斜率
 //  k2= xie(Image_R);//右边界斜率
   for(i=63;i>0;i--)
     {
        x_L_E  += i;
        y_L_E  += Image_L[i]; 
        x_R_E  += i;
        y_R_E  += (Image_R[i]-56); 
        x_M_E  += i;
        y_M_E  += Middle_line[i]; 
        
     }
    x_L_E /= 63 ;
    y_L_E /= 63 ;
    x_R_E /= 63 ;
    y_R_E /= 63 ;
    x_M_E /= 63 ;
    y_M_E /= 63 ;
    for(i=63;i>0;i--)
     {
        X2_L_sum += (i-x_L_E)*(i-x_L_E);
        XY_L_sum += (Image_L[i]- y_L_E)* (i-x_L_E);
        X2_R_sum += (i-x_R_E)*(i-x_R_E);
        XY_R_sum += (Image_R[i]-56- y_R_E)* (i-x_R_E);
        X2_M_sum += (i-x_M_E)*(i-x_M_E);
        XY_M_sum += (Middle_line[i]- y_M_E)* (i-x_M_E);
        
     }
     k_L = XY_L_sum/X2_L_sum;
     k_R = XY_R_sum/X2_R_sum;
     k_M = XY_M_sum/X2_M_sum;
   

    
  // OLED_P6x8int(0, 2,(int)(k_L*1000), 0); 
  // OLED_P6x8int(0, 5,(int)(k_R*1000), 0);
  // OLED_P6x8int(0, 5,(int)(k_M*1000), 0);
}

*/
