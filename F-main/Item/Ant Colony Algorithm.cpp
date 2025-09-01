//��Ⱥ�㷨
#include <iostream.h> 
#include <fstream.h> 
#include <math.h> 
#include <time.h> 
#include <conio.h> 
#include <stdlib.h> 
#include <iomanip.h> 

#define N 13 //city size 
#define M 13 //ant number 

double inittao=1; //��ʼ��Ϣ���Ķ���
double tao[N][N]; //ÿ��·���ϵ���Ϣ��
double detatao[N][N]; //���ӣ�������Ӧ·���ϵ���Ϣ������ 
double distance[N][N]; //���о������distance[i][j]=distance[j][i]
double yita[N][N]; //������������ֵyita[i][j]=1/distance[i][j] 
int tabu[M][N]; //���ɱ�tabu[i][j]=1��ʾ����i�Ѿ��߹���j����? 
int route[M][N]; //��������k��·��������Ϊroute[k][N] 
double solution[M]; 
int BestRoute[N]; 
double BestSolution=10000000000; 
double alfa,beta,rou,Q; //0<rou<1,��ʾ��Ϣ�ص�Ũ�ȣ�(1-rou)��ʾ��Ϣ�صĻӷ��ٶȣ�Q��ʾ��Ϣ��ǿ��
int NcMax; 
/*Pijk(t)��ʾtʱ������k�ɳ���iת�Ƶ�����j��״̬ת�Ƹ��ʣ� 
alfa����Ϣ����ʽ���ӣ���ʾ�켣�������Ҫ�ԣ���ӳ�������˶������������۵���Ϣ�������˶�ʱ��������ã���ֵԽ���������Խ������ѡ���������Ͼ�����·��������֮���Э����Խǿ 
beta����������ʽ���ӣ���ʾ�ܼ��ȵ������Ҫ�ԣ���ӳ���˶�������������Ϣ������ѡ��·���е������ӳ̶ȣ���ֵԽ�����״̬ת�Ƹ���Խ�ӽ���̰�Ĺ��� 
rou����Ϣ��������(�����ϲ�ͬ��������rou��ʾ��Ϣ�ӷ�ϵ��������1-rou��ʾ��Ϣ��������) 
QΪ��Ϣ��ǿ�ȣ����ڼ�����������·���ϵ���Ϣ��*/
void initparameter(void); // initialize the parameters of basic ACA (��ʼ�������Ļ�������)
double EvalueSolution(int *a); // evaluate the solution of TSP, and calculate the length of path(�������������TSP ��������·���ĳ���)
void InCityXY( double x[], double y[], char *infile ); // input the nodes' coordinates of TSP (����ڵ�������TSP)

void initparameter(void) 
{
	alfa=1; beta=5; rou=0.9; Q=100; //�������
    NcMax=200; 
} 

void main(void) 
{
	int NC=0; 
    initparameter(); //��ʼ������
    double x[N]; 
    double y[N]; 
    InCityXY( x, y, "city13.tsp" ); //��ȡ����
    for(int i=0;i<N;i++) 
		for(int j=i+1;j<N;j++) 
		{ 
			distance[j][i]=sqrt((x[i]-x[j])*(x[i]-x[j])+(y[i]-y[j])*(y[i]-y[j])); 
			distance[i][j]=distance[j][i]; 
		} // calculate the heuristic parameters (����ʽ����)
    for(i=0;i<N;i++) 
		for(int j=0;j<N;j++) 
		{ 
			tao[i][j]=inittao; 
			if(j!=i) 
				yita[i][j]=100/distance[i][j]; //ѭ��t�о���·��ij����Ϣ������
		} 
    for(int k=0;k<M;k++) 
		for(i=0;i<N;i++) 
			route[k][i]=-1;//��ʾ·����δ�߹�,�����ϵ�·���������ÿ�,Ϊ��һ��ѭ����׼�� 
	srand(time(NULL)); //srand()������һ���������������������˼����ָC�������������������������Ʋ����ģ�
    for(k=0;k<M;k++) 
	{ 
		route[k][0]=k%N; //ÿֻ���Ϸ������λ�õ�һ������. Ϊÿֻ���Ϸ���һ����ʼ����. ��Nȡģ(%N)��ֹ�����ܵĳ��и���. 
		tabu[k][route[k][0]]=1; //����ʼ���з�����ɱ� 
	} 
    //each ant try to find the optiamal path 
    do
	{ 
		int s=1; 
		double partsum; 
		double pper; 
		double drand; 
	//��ʼ�����kֻ���ϵ�·�� 
    while(s<N) 
	{ 
		for(k=0;k<M;k++) 
		{ 
			int jrand=rand()%3000; //����һ��0��3000֮��������
			drand=jrand/3000.; 
			printf("drand=%f",drand);
			partsum=0; 
			pper=0; //ת�Ƹ��� 
            for(int j=0;j<N;j++) //���ݸ��ʺ����������ϵ�ת�Ƹ���
			{ 
				if(tabu[k][j]==0) 
                partsum+=pow(tao[route[k][s-1]][j],alfa)*pow(yita[route[k][s-1]][j],beta); 
				//route[k][s-1]��ʾ����k��s-1���ߵ��ĳ��У�s==1��ʾ���ϴӳ�ʼ���г���
				//tao[route[k][s-1]][j]��ʾ����k������ǰһ�����е���һ��û�����ʵĳ���j����Ϣ��
			} 
            for(j=0;j<N;j++) 
			{
				if(tabu[k][j]==0) 
					pper+=pow(tao[route[k][s-1]][j],alfa)*pow(yita[route[k][s-1]][j],beta)/partsum; 
                if(pper>drand) //��drand���ڵ�j��������ʱ��ѡ��j����
					break; 
			} 
			tabu[k][j]=1;//���ɱ��÷��ʱ�־  
			route[k][s]=j; //��������k�ĵ�s�������ĳ��� 
		} 
		s++; 
	} 
    // the pheromone is updated ��Ϣ�ظ���
	//��N��ѭ�����������ϵĽ��ɱ�������,����ÿ�������߹���·���ĳ��ȣ����ҵ����·�����棬��¼��·����������Ϣ�ء� 
    for(i=0;i<N;i++) 
		for(int j=0;j<N;j++) 
			detatao[i][j]=0; 
   //����ÿ�����Ͼ���·���ĳ��ȣ����浽��ǰ��Ϊֹ��õĽ�·�����䳤��
	for(k=0;k<M;k++) 
	{ 
		solution[k]=EvalueSolution(route[k]); //����k������·�ߵ��ܳ��� 
		if(solution[k]<BestSolution) 
		{ 
			BestSolution=solution[k]; 
			for(s=0;s<N;s++) 
				BestRoute[s]=route[k][s]; 
		} 
	} 
	//�������·���ϵ���Ϣ������ 
    for(k=0;k<M;k++) 
	{ 
		for(s=0;s<N-1;s++) 
        detatao[route[k][s]][route[k][s+1]]+=Q/solution[k]; 
        detatao[route[k][N-1]][route[k][0]]+=Q/solution[k]; 
	} 
    for(i=0;i<N;i++) 
		for(int j=0;j<N;j++) 
		{
			tao[i][j]=rou*tao[i][j]+detatao[i][j]; 
            if(tao[i][j]<0.00001) //�½� 
				tao[i][j]=0.00001; 
            if(tao[i][j]>20) //��Ϣ���Ͻ磬����94�к�101���У�������Ϣ��·����Ϣ��û 
				tao[i][j]=20; 
		} 
		//�����ϵ�·���������ÿ�,Ϊ��һ��ѭ����׼��,Ҫ��Ȼÿ�����ϵ�·�����Ѿ�����,��û�а취������һ�ε�����. 
		for(k=0;k<M;k++) 
			for(int j=1;j<N;j++) //ע����ʼ���У���j=0û�б���� 
			{ 
				tabu[k][route[k][j]]=0; 
				route[k][j]=-1; 
			} 
			NC++; 
	} while(NC<NcMax); 
    //output the calculating results 
    fstream result; //���ļ��������
    result.open("optimal_results.log", ios::app); 
    if(!result) 
	{ 
		cout<<"can't open the <optimal_results.log> file!\n"; 
		exit(0); 
	} 
    result<<"*-------------------------------------------------------------------------*"<<endl; 
    result<<"the initialized parameters of ACA are as follows:"<<endl; 
    result<<"alfa="<<alfa<<", beta="<<beta<<", rou="<<rou<<", Q="<<Q<<endl; 
    result<<"the maximum iteration number of ACA is:"<<NcMax<<endl; 
    result<<"the shortest length of the path is:"<<BestSolution<<endl; 
    result<<"the best route is:"<<endl; 
    for(i=0;i<N;i++) 
		result<<BestRoute[i]<<" "; 
    result<<endl; 
    result<<"*-------------------------------------------------------------------------*"<<endl<<endl; 
    result.close(); 
    cout<<"the shortest length of the path is:"<<BestSolution<<endl; 
} 


double EvalueSolution(int *a) 
{ 
	double dist=0; 
    for(int i=0;i<N-1;i++) 
    dist+=distance[a[i]][a[i+1]]; 
    dist+=distance[a[i]][a[0]]; 
    return dist; 
} 


void InCityXY( double x[], double y[], char *infile ) 
{ 
	fstream inxyfile( infile, ios::in | ios::nocreate ); 
    if( !inxyfile )
	{ 
		cout<<"can't open the <"<<infile<<"> file!\n"; 
		exit(0); 
	} 
    int i=0; 
    while( !inxyfile.eof() ) 
	{ 
		inxyfile>>x[i]>>y[i]; 
        if( ++i >= N ) break; 
	} 
} 
