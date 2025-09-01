//蚁群算法
#include <iostream.h> 
#include <fstream.h> 
#include <math.h> 
#include <time.h> 
#include <conio.h> 
#include <stdlib.h> 
#include <iomanip.h> 

#define N 13 //city size 
#define M 13 //ant number 

double inittao=1; //初始信息量的多少
double tao[N][N]; //每条路径上的信息量
double detatao[N][N]; //Δτ，代表相应路径上的信息素增量 
double distance[N][N]; //城市距离矩阵，distance[i][j]=distance[j][i]
double yita[N][N]; //启发函数，其值yita[i][j]=1/distance[i][j] 
int tabu[M][N]; //禁忌表，tabu[i][j]=1表示蚂蚁i已经走过了j城市? 
int route[M][N]; //保存蚂蚁k的路径的数组为route[k][N] 
double solution[M]; 
int BestRoute[N]; 
double BestSolution=10000000000; 
double alfa,beta,rou,Q; //0<rou<1,表示信息素的浓度，(1-rou)表示信息素的挥发速度，Q表示信息素强度
int NcMax; 
/*Pijk(t)表示t时刻蚂蚁k由城市i转移到城市j的状态转移概率， 
alfa是信息启发式因子，表示轨迹的相对重要性，反映蚂蚁在运动过程中所积累的信息在蚂蚁运动时所起的作用，其值越大，则该蚂蚁越倾向于选择其他蚂蚁经过的路径，蚂蚁之间的协作性越强 
beta是期望启发式因子，表示能见度的相对重要性，反映在运动过程中启发信息在蚂蚁选择路径中的受重视程度，其值越大，则该状态转移概率越接近于贪心规则 
rou是信息残留因子(与书上不同，书上用rou表示信息挥发系数，而用1-rou表示信息残留因子) 
Q为信息素强度，用于计算蚂蚁留在路径上的信息量*/
void initparameter(void); // initialize the parameters of basic ACA (初始化参数的基本抗体)
double EvalueSolution(int *a); // evaluate the solution of TSP, and calculate the length of path(评估解决方案中TSP ，并计算路径的长度)
void InCityXY( double x[], double y[], char *infile ); // input the nodes' coordinates of TSP (输入节点坐标中TSP)

void initparameter(void) 
{
	alfa=1; beta=5; rou=0.9; Q=100; //输入参数
    NcMax=200; 
} 

void main(void) 
{
	int NC=0; 
    initparameter(); //初始化参数
    double x[N]; 
    double y[N]; 
    InCityXY( x, y, "city13.tsp" ); //读取参数
    for(int i=0;i<N;i++) 
		for(int j=i+1;j<N;j++) 
		{ 
			distance[j][i]=sqrt((x[i]-x[j])*(x[i]-x[j])+(y[i]-y[j])*(y[i]-y[j])); 
			distance[i][j]=distance[j][i]; 
		} // calculate the heuristic parameters (启发式参数)
    for(i=0;i<N;i++) 
		for(int j=0;j<N;j++) 
		{ 
			tao[i][j]=inittao; 
			if(j!=i) 
				yita[i][j]=100/distance[i][j]; //循环t中经过路径ij的信息素增量
		} 
    for(int k=0;k<M;k++) 
		for(i=0;i<N;i++) 
			route[k][i]=-1;//表示路径都未走过,将蚂蚁的路径再重新置空,为下一次循环做准备 
	srand(time(NULL)); //srand()函数是一个随机数产生函数，其意思就是指C语言里的随机数都是由它来控制产生的！
    for(k=0;k<M;k++) 
	{ 
		route[k][0]=k%N; //每只蚂蚁分配出发位置的一个方法. 为每只蚂蚁分配一个起始城市. 对N取模(%N)防止超出总的城市个数. 
		tabu[k][route[k][0]]=1; //把起始城市放入禁忌表 
	} 
    //each ant try to find the optiamal path 
    do
	{ 
		int s=1; 
		double partsum; 
		double pper; 
		double drand; 
	//开始计算第k只蚂蚁的路径 
    while(s<N) 
	{ 
		for(k=0;k<M;k++) 
		{ 
			int jrand=rand()%3000; //生成一个0到3000之间的随机数
			drand=jrand/3000.; 
			printf("drand=%f",drand);
			partsum=0; 
			pper=0; //转移概率 
            for(int j=0;j<N;j++) //根据概率函数计算蚂蚁的转移概率
			{ 
				if(tabu[k][j]==0) 
                partsum+=pow(tao[route[k][s-1]][j],alfa)*pow(yita[route[k][s-1]][j],beta); 
				//route[k][s-1]表示蚂蚁k第s-1步走到的城市，s==1表示蚂蚁从初始城市出发
				//tao[route[k][s-1]][j]表示蚂蚁k经过的前一个城市到下一个没被访问的城市j的信息量
			} 
            for(j=0;j<N;j++) 
			{
				if(tabu[k][j]==0) 
					pper+=pow(tao[route[k][s-1]][j],alfa)*pow(yita[route[k][s-1]][j],beta)/partsum; 
                if(pper>drand) //当drand落在第j个城市上时，选择j城市
					break; 
			} 
			tabu[k][j]=1;//禁忌表置访问标志  
			route[k][s]=j; //保存蚂蚁k的第s步经过的城市 
		} 
		s++; 
	} 
    // the pheromone is updated 信息素更新
	//在N次循环后，所有蚂蚁的禁忌表都已填满,计算每个蚂蚁走过的路径的长度，并找到最短路径保存，记录此路径并更改信息素。 
    for(i=0;i<N;i++) 
		for(int j=0;j<N;j++) 
			detatao[i][j]=0; 
   //计算每个蚂蚁经过路径的长度，保存到当前代为止最好的解路径及其长度
	for(k=0;k<M;k++) 
	{ 
		solution[k]=EvalueSolution(route[k]); //蚂蚁k经过的路线的总长度 
		if(solution[k]<BestSolution) 
		{ 
			BestSolution=solution[k]; 
			for(s=0;s<N;s++) 
				BestRoute[s]=route[k][s]; 
		} 
	} 
	//计算各个路径上的信息素增量 
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
            if(tao[i][j]<0.00001) //下界 
				tao[i][j]=0.00001; 
            if(tao[i][j]>20) //信息素上界，避免94行和101行中，启发信息被路径信息淹没 
				tao[i][j]=20; 
		} 
		//将蚂蚁的路径再重新置空,为下一次循环做准备,要不然每个蚂蚁的路径都已经满了,则没有办法进行下一次迭代了. 
		for(k=0;k<M;k++) 
			for(int j=1;j<N;j++) //注意起始城市，即j=0没有被清空 
			{ 
				tabu[k][route[k][j]]=0; 
				route[k][j]=-1; 
			} 
			NC++; 
	} while(NC<NcMax); 
    //output the calculating results 
    fstream result; //向文件输出内容
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
