#ifndef _FUZZY_H
#define _FUZZY_H


typedef enum
{
Fuzzy_Rank7=7,
Fuzzy_Rank5=5,
Fuzzy_Rank3=3,
}Fuzzy_Rank_e; // Fuzzy Rank Enum

typedef struct
{
Fuzzy_Rank_e Rank;  // 分级数量
float fe;  // e(k)
float fec; // e(k)'
float eRule[7];   //误差隶属度函数中心值
float ecRule[7];  //误差变化隶属度函数中心值
float U1Rule[7];  //输出隶属函数中心值
int rule[7][7];
}FuzzyStruct;  // 模糊结构体


extern float Fuzzy_Update(FuzzyStruct* F_S,float ek,float ekc);
extern float FuzzyCtrl(FuzzyStruct* Fuzzy_S);

#endif