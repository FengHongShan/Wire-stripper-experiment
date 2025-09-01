#ifndef _SINGLE_NERVE_PID_H_
#define _SINGLE_NERVE_PID_H_
#include "struct_typedef.h"
#include "math.h"
typedef struct
{
    fp32 setpoint;//�趨ֵ
    fp32 feeddata;//����ֵ
    fp32 result;//������ֵ
    fp32 lasterror;   /*ǰһ��ƫ��*/
    fp32  preerror;   /*ǰ����ƫ��*/
    fp32 maximum;     /*���ֵ������*/
    fp32 minimum;     /*���ֵ������*/
    fp32 deadband;     /*����*/
    fp32 wp;       //����Ԫ������Ȩϵ��
    fp32 wi;       //����Ԫ���ּ�Ȩϵ��
    fp32 wd;      //����Ԫ΢�ּ�Ȩϵ��
    fp32 kcoef;  //����Ԫ�������ϵ��
    fp32 kp;    //����Ԫ����ѧϰ�ٶ�
    fp32 ki;   //����Ԫ����ѧϰ�ٶ�
    fp32 kd;   //����Ԫ΢��ѧϰ�ٶ�

}NERVEPID;

void NERVEPIDInitialization(NERVEPID *vPID,float vMin,float vMax);

static void NeureLearningRules(NERVEPID *vPID,float zk,float uk,float *xi);
fp32 NERVEPID_cal(NERVEPID *vPID, fp32 set,fp32 ref);

#endif /**/


