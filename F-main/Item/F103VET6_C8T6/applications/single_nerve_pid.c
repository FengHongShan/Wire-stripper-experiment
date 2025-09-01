#include "single_nerve_pid.h"
#include "math.h"
void NERVEPIDInitialization(NERVEPID *vPID,float vMin,float vMax)
{
  vPID->kcoef=0.12; /*��Ԫ�������*/
  vPID->kp=0.4;                         /*����ѧϰ�ٶ�*/
  vPID->ki=0.35;                        /*����ѧϰ�ٶ�*/
  vPID->kd=0.4;   

  vPID->lasterror=0.0;                  /*ǰһ��ƫ��*/
  vPID->preerror=0.0;                   /*ǰ����ƫ��*/
  vPID->wp=0.10; /*������Ȩϵ��*/
  vPID->wi=0.10; /*���ּ�Ȩϵ��*/
  vPID->wd=0.10; /*΢�ּ�Ȩϵ��*/
  vPID->maximum=vMax;                   /*���ֵ����*/
  vPID->minimum=vMin;                   /*���ֵ����*/  

  vPID->result=vMin;
  vPID->deadband=(vMax-vMin)*0.0005;    /*����*/

  
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

/*����Ԫѧϰ������*/
static void NeureLearningRules(NERVEPID *vPID,float zk,float uk,float *xi)
{
  vPID->wi=vPID->wi+vPID->ki*zk*uk*xi[0];
  vPID->wp=vPID->wp+vPID->kp*zk*uk*xi[1];
  vPID->wd=vPID->wd+vPID->kd*zk*uk*xi[2];
}




