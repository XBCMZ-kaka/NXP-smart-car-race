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
Fuzzy_Rank_e Rank;  // �ּ�����
float fe;  // e(k)
float fec; // e(k)'
float eRule[7];   //��������Ⱥ�������ֵ
float ecRule[7];  //���仯�����Ⱥ�������ֵ
float U1Rule[7];  //���������������ֵ
int rule[7][7];
}FuzzyStruct;  // ģ���ṹ��


extern float Fuzzy_Update(FuzzyStruct* F_S,float ek,float ekc);
extern float FuzzyCtrl(FuzzyStruct* Fuzzy_S);

#endif