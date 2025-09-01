#ifndef _SINGLE_NERVE_PID_H_
#define _SINGLE_NERVE_PID_H_
#include "struct_typedef.h"
#include "math.h"
typedef struct
{
    fp32 setpoint;//设定值
    fp32 feeddata;//反馈值
    fp32 result;//输出结果值
    fp32 lasterror;   /*前一拍偏差*/
    fp32  preerror;   /*前两拍偏差*/
    fp32 maximum;     /*输出值的上限*/
    fp32 minimum;     /*输出值的下限*/
    fp32 deadband;     /*死区*/
    fp32 wp;       //单神经元比例加权系数
    fp32 wi;       //单神经元积分加权系数
    fp32 wd;      //单神经元微分加权系数
    fp32 kcoef;  //单神经元输出比例系数
    fp32 kp;    //单神经元比例学习速度
    fp32 ki;   //单神经元积分学习速度
    fp32 kd;   //单神经元微分学习速度

}NERVEPID;

void NERVEPIDInitialization(NERVEPID *vPID,float vMin,float vMax);

static void NeureLearningRules(NERVEPID *vPID,float zk,float uk,float *xi);
fp32 NERVEPID_cal(NERVEPID *vPID, fp32 set,fp32 ref);

#endif /**/


