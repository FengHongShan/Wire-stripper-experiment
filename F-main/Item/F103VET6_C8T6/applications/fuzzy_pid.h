#ifndef _FUZZY_PID_H_
#define _FUZZY_PID_H_
#include "struct_typedef.h"


#define ID_dKp 1
#define ID_dKi 2
#define ID_dKd 3
/*
 ��ec��ģ������Ϊ��׼ec[-120,120]
 e��ģ������Ϊ[-180,180]
 dKp��ģ������Ϊ[-2,2]
 dKi��ģ������Ϊ[-0.1,0.1]
 dKd��ģ������Ϊ[-2,2]
*/
static const float Ge = 0.5;//����ϵ��
static const float Gkp = 0;//����ϵ��
static const float Gki = 450;//����ϵ��
static const float Gkd = 6.0;//����ϵ��

#define NB   -120
#define NM	 -80
#define NS	 -40
#define ZO	 0
#define PS	 40
#define PM	 80
#define PB	 120
/*
 
dKp   ec
	    NB	NM	NS	ZO	PS	PM	PB
e	NB	PB	PB	PM	PM	PS	ZO	ZO
	NM	PB	PB	PM	PS	PS	ZO	NS
	NS	PM	PM	PM	PS	ZO	NS	NS
	ZO	PM	PM	PS	ZO	NS	NM	NM
	PS	PS	PS	ZO	NS	NS	NM	NM
	PM	PS	ZO	NS	NM	NM	NM	NB
	PB	ZO	ZO	NM	NM	NM	NB	NB
	---------------------------------
	 
dKi     ec
	    NB	NM	NS	ZO	PS	PM	PB
e	NB	NB	NB	NM	NM	NS	ZO	ZO
	NM	NB	NB	NM	NS	NS	ZO	ZO
	NS	NB	NM	NS	NS	ZO	PS	PS
	ZO	NM	NM	NS	ZO	PS	PM	PM
	PS	NM	NS	ZO	PS	PS	PM	PB
	PM	ZO	ZO	PS	PS	PM	PB	PB
	PB	ZO	ZO	PS	PM	PM	PB	PB
------------------------------------
 
dKd   ec
	    NB	NM	NS	ZO	PS	PM	PB
e	NB	PS	NS	NB	NB	NB	NM	PS
	NM	PS	NS	NB	NM	NM	NS	ZO
	NS	ZO	NS	NM	NM	NS	NS	ZO
	ZO	ZO	NS	NS	NS	NS	NS	ZO
	PS	ZO	ZO	ZO	ZO	ZO	ZO	ZO
	PM	PB	NS	PS	PS	PS	PS	PB
	PB	PB	PM	PM	PM	PS	PS	PB


*/
static const fp32 fuzzyRuleKp[7][7]={
	NB,	NB,	NM,	NM,	NS,	ZO,	ZO,
	NB,	NM,	NM,	NS,	NS,	ZO,	ZO,
	NB,	NM,	NS,	NS,	ZO,	PS,	PS,
	NM,	NM,	NS,	ZO,	PS,	PM,	PM,
	NM,	NS,	ZO,	PS,	PS,	PM,	PB,
	ZO,	ZO,	PS,	PS,	PM,	PB,	PB,
	ZO,	ZO,	PM,	PM,	PM,	PB,	PB
};
//static const fp32 fuzzyRuleKp[7][7]={
//	PB,	PB,	PM,	PM,	PS,	ZO,	ZO,
//	PB,	PM,	PM,	PS,	PS,	ZO,	ZO,
//	PB,	PM,	PS,	PS,	ZO,	PS,	PS,
//	PM,	PM,	PS,	ZO,	PS,	PM,	PM,
//	PM,	PS,	ZO,	PS,	PS,	PM,	PB,
//	ZO,	ZO,	PS,	PS,	PM,	PB,	PB,
//	ZO,	ZO,	PM,	PM,	PM,	PB,	PB
//};

//static const fp32 fuzzyRuleKi[7][7]={
//	PB,	PB,	PM,	PM,	PS,	ZO,	ZO,
//	PB,	PB,	PM,	PS,	PS,	ZO,	NS,
//	PM,	PM,	PM,	PS,	ZO,	NS,	NS,
//	PM,	PM,	PS,	ZO,	NS,	NM,	NM,
//	PS,	PS,	ZO,	NS,	NS,	NM,	NB,
//	PS,	ZO,	NS,	NM,	NM,	NM,	NB,
//	ZO,	ZO,	NM,	NM,	NM,	NB,	NB
//};
static const fp32 fuzzyRuleKi[7][7]={
	PB,	PB,	PM,	PM,	PS,	ZO,	ZO,
	PB,	PB,	PM,	PS,	PS,	ZO,	PS,
	PM,	PM,	PM,	PS,	ZO,	PS,	PS,
	PM,	PM,	PS,	ZO,	PS,	PM,	PM,
	PS,	PS,	ZO,	PS,	PS,	PM,	PB,
	PS,	ZO,	PS,	PM,	PM,	PM,	PB,
	ZO,	ZO,	PM,	PM,	PM,	PB,	PB
};

static const fp32 fuzzyRuleKd[7][7]={
	PB,	PB,	PB,	NB,	NB,	NS,	NS,
	PB,	PB,	PM,	NM,	ZO,	PS,	PM,
	PB,	PM,	PM,	NS,	PM,	PB,	PB,
	ZO,	ZO,	ZO,	ZO,	ZO,	ZO,	ZO,
	PB,	PB,	PM,	NS,	PM,	PM,	PB,
	PM,	PS,	ZO,	NM,	PM,	PB,	PB,
	PB,	PM,	PS,	PS,	PS,	PM,	PB
};
typedef struct{
	fp32 dKp;
	fp32 dKi;
	fp32 dKd;
}dPID;//ģ��PID���������ṹ��

//ģ��PIDʵ�ֺ���
dPID fuzzy(fp32 e,fp32 ec);
float DeFuzzy(int eLevel,int ecLevel,u8 ID_item);
#endif/**/

