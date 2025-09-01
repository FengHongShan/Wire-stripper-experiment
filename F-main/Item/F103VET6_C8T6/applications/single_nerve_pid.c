#include "single_nerve_pid.h"
#include "math.h"
void NERVEPIDInitialization(NERVEPID *vPID,float vMin,float vMax)
{
  vPID->kcoef=0.12; /*神经元输出比例*/
  vPID->kp=0.4;                         /*比例学习速度*/
  vPID->ki=0.35;                        /*积分学习速度*/
  vPID->kd=0.4;   

  vPID->lasterror=0.0;                  /*前一拍偏差*/
  vPID->preerror=0.0;                   /*前两拍偏差*/
  vPID->wp=0.10; /*比例加权系数*/
  vPID->wi=0.10; /*积分加权系数*/
  vPID->wd=0.10; /*微分加权系数*/
  vPID->maximum=vMax;                   /*输出值上限*/
  vPID->minimum=vMin;                   /*输出值下限*/  

  vPID->result=vMin;
  vPID->deadband=(vMax-vMin)*0.0005;    /*死区*/

  
}





fp32 NERVEPID_cal(NERVEPID *vPID, fp32 set,fp32 ref)
{
  float x[3];
  float w[3];
  float sabs;
  float error;
  float result;
  float deltaResult;
 
  error=set-ref;
  result=vPID->result;

  if(fabs(error)>vPID->deadband)
  {
    x[0]=error;
    x[1]=error-vPID->lasterror;
    x[2]=error-vPID->lasterror*2+vPID->preerror;
  
    sabs=fabs(vPID->wi)+fabs(vPID->wp)+fabs(vPID->wd);
    w[0]=vPID->wi/sabs;
    w[1]=vPID->wp/sabs;
    w[2]=vPID->wd/sabs;
    
    deltaResult=(w[0]*x[0]+w[1]*x[1]+w[2]*x[2])*vPID->kcoef;
  }
  else
  {
    deltaResult=0;
  }

  result+=deltaResult;
  if(result>vPID->maximum)
  {
    result=vPID->maximum;
  }
  if(result<vPID->minimum)
  {
    result=vPID->minimum;
  }
  vPID->result=result;

  NeureLearningRules(vPID,error,result,x);
  
  vPID->preerror=vPID->lasterror;
  vPID->lasterror=error;
  return vPID->result;
}

/*单神经元学习规则函数*/
static void NeureLearningRules(NERVEPID *vPID,float zk,float uk,float *xi)
{
  vPID->wi=vPID->wi+vPID->ki*zk*uk*xi[0];
  vPID->wp=vPID->wp+vPID->kp*zk*uk*xi[1];
  vPID->wd=vPID->wd+vPID->kd*zk*uk*xi[2];
}




