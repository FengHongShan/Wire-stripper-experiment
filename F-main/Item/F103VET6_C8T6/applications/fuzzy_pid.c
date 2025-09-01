#include "fuzzy_pid.h"


float DeFuzzy(int eLevel,int ecLevel,u8 ID_item)
{
 
		switch(ID_item)
		{
		 
				case ID_dKp:
				return (((fuzzyRuleKp[eLevel][ecLevel]*1)/40)-1);
				 
				case ID_dKi:
				return fuzzyRuleKi[eLevel][ecLevel] / Gki;
				 
				case ID_dKd:
				return fuzzyRuleKd[eLevel][ecLevel] / Gkd;
				 
				default:
				return 0;
		}
 
}




dPID fuzzy(fp32 e,fp32 ec)
{
	int pex,pey,pecx,pecy;
	fp32 eFuzzy[2]={0,0};
	fp32 ecFuzzy[2]={0,0};
	static fp32 E_TABLE[]={NB,NM,NS,ZO,PS,PM,PB};
  static fp32 EC_TABLE[]={Ge*NB,Ge*NM,Ge*NS,ZO,Ge*PS,Ge*PM,Ge*PB};
	dPID fuzzy_PID;
	//求e的隶属度
	if(e<E_TABLE[0])
	{
		eFuzzy[0]=1.0;
		pex=0;
		pey=0;
	}
	else if(e>=E_TABLE[0]&&e<E_TABLE[1])
	{
		eFuzzy[0]=(E_TABLE[1]-e)/(E_TABLE[1]-E_TABLE[0]);
		pex=0;
		pey=1;
	}
		else if(e>=E_TABLE[1]&&e<E_TABLE[2])
	{
		eFuzzy[0]=(E_TABLE[2]-e)/(E_TABLE[2]-E_TABLE[1]);
		pex=1;
		pey=2;		
	}
		else if(e>=E_TABLE[2]&&e<E_TABLE[3])
	{
		eFuzzy[0]=(E_TABLE[3]-e)/(E_TABLE[3]-E_TABLE[2]);
		pex=2;
		pey=3;
	}
		else if(e>=E_TABLE[3]&&e<E_TABLE[4])
	{
		eFuzzy[0]=(E_TABLE[4]-e)/(E_TABLE[4]-E_TABLE[3]);
		pex=3;
		pey=4;
	}
		else if(e>=E_TABLE[4]&&e<E_TABLE[5])
	{
		eFuzzy[0]=(E_TABLE[5]-e)/(E_TABLE[5]-E_TABLE[4]);
		pex=4;
		pey=5;
	}
		else if(e>=E_TABLE[5]&&e<E_TABLE[6])
	{
		eFuzzy[0]=(E_TABLE[6]-e)/(E_TABLE[6]-E_TABLE[5]);
		pex=5;
		pey=6;
	}
	else
	{
	  eFuzzy[0]=1;
		pex=6;
		pey=6;
	}
	eFuzzy[1]=1-eFuzzy[0];
	
	//ec的隶属度
	if(ec<EC_TABLE[0])
	{
		ecFuzzy[0]=1.0;
		pecx=0;
		pecy=0;
	}
	else if(ec>=EC_TABLE[0]&&ec<EC_TABLE[1])
	{
		ecFuzzy[0]=(EC_TABLE[1]-ec)/(EC_TABLE[1]-EC_TABLE[0]);
		pecx=0;
		pecy=1;
	}
		else if(ec>=EC_TABLE[1]&&ec<EC_TABLE[2])
	{
		ecFuzzy[0]=(EC_TABLE[2]-ec)/(EC_TABLE[2]-EC_TABLE[1]);
		pecx=1;
		pecy=2;
	}
		else if(ec>=EC_TABLE[2]&&ec<EC_TABLE[3])
	{
		ecFuzzy[0]=(EC_TABLE[3]-ec)/(EC_TABLE[3]-EC_TABLE[2]);
		pecx=2;
		pecy=3;
	}
		else if(ec>=EC_TABLE[3]&&ec<EC_TABLE[4])
	{
		ecFuzzy[0]=(EC_TABLE[4]-ec)/(EC_TABLE[4]-EC_TABLE[3]);
		pecx=3;
		pecy=4;
	}
		else if(ec>=EC_TABLE[4]&&ec<EC_TABLE[5])
	{
		ecFuzzy[0]=(EC_TABLE[5]-ec)/(EC_TABLE[5]-EC_TABLE[4]);
		pecx=4;
		pecy=5;
	}
		else if(ec>=EC_TABLE[5]&&ec<EC_TABLE[6])
	{
		ecFuzzy[0]=(EC_TABLE[6]-ec)/(EC_TABLE[6]-EC_TABLE[5]);
		pecx=5;
		pecy=6;
	}
		else
	{
	  ecFuzzy[0]=1;
		pecx=6;
		pecy=6;
	}
	ecFuzzy[1]=1-ecFuzzy[0];
	
	//解模糊
	fuzzy_PID.dKp = (eFuzzy[0] * ecFuzzy[0] *  DeFuzzy(pex,pecx,ID_dKp)                   
					+ eFuzzy[0]*ecFuzzy[1] * DeFuzzy(pex,pecy,ID_dKp)
					+eFuzzy[1]*ecFuzzy[0] * DeFuzzy(pey,pecx,ID_dKp)
					+ eFuzzy[1]*ecFuzzy[1]* DeFuzzy(pey,pecy,ID_dKp));
 
	fuzzy_PID.dKi =  (eFuzzy[0] * ecFuzzy[0] * DeFuzzy(pex,pecx,ID_dKi)                   
					+ eFuzzy[0]*ecFuzzy[1] * DeFuzzy(pex,pecy,ID_dKi)
					+eFuzzy[1]*ecFuzzy[0] * DeFuzzy(pey,pecx,ID_dKi)
					+ eFuzzy[1]*ecFuzzy[1]* DeFuzzy(pey,pecy,ID_dKi));
 
	fuzzy_PID.dKd = (eFuzzy[0] * ecFuzzy[0] *  DeFuzzy(pex,pecx,ID_dKd)                    
					+ eFuzzy[0]*ecFuzzy[1] * DeFuzzy(pex,pecy,ID_dKd)
					+eFuzzy[1]*ecFuzzy[0] * DeFuzzy(pey,pecx,ID_dKd)
					+ eFuzzy[1]*ecFuzzy[1]* DeFuzzy(pey,pecy,ID_dKd));
	
	
	
	
	return fuzzy_PID;
}
