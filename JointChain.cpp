#include <iostream>
#include <random>
#include "JointChain.h"
using namespace std;

JointChain::JointChain(VectorXd Pos0, int Num, double *length)
{
	assert(Pos0.rows() == 4);
	assert(Pos0[3] == 1);

	//Pos0 must be general coordinates
	JointNum = Num;
	chain = new AJoint[Num];
	Pos = new VectorXd[Num + 1];
	
	// add: revise joints' sta tes. 
	for (int i = 0; i<Num; i++)
	{
		//cout << Pos0;
		Pos[i] = Pos0;
		chain[i].SetPos(Pos0);

		//我在这里所定义的角度，x y z三个角度，不是当前向量真的和x y z成什么角度，而是当前向量和(1, 0, 0)比，其x y z三个角差了多少。
		// 全部设置为0,0,0的意思是：向量呆在1,0,0处，不要动。
		chain[i].SetTheta(0, 0);
		chain[i].SetTheta(1, 0);
		chain[i].SetTheta(2, 0);

		chain[i].SetLength(length[i]);
		VectorXd tmp = VectorXd::Zero(4);
		tmp(0) = length[i];
		tmp(3) = 1;
		//cout << tmp;
		tmp = (*chain[i].GetRot())*tmp;
		//cout << tmp;
		//cout << Pos0;
		Pos0 += tmp;
		Pos0[3] = 1;
	}
	Pos[Num] = Pos0;
}

int JointChain::SetTheta(int id, int xyz, double theta)
{//这些theta角都是弧度制的，求cos也都是弧度制的球法
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	VectorXd Pos0 = Pos[id];
 	chain[id].SetTheta(xyz, theta);

	for (int i = id; i<JointNum; i++)
	{
		//cout << "Pos0:" << endl << Pos0 << endl;
		Pos[i] = Pos0;
		chain[i].SetPos(Pos0);
		double length = chain[i].GetLength();
		VectorXd tmp = VectorXd::Zero(4);
		tmp(0) = length;
		tmp(3) = 1;
		MatrixXd X = *chain[i].GetRot();
		//cout << endl << X << endl;
		//cout << endl << tmp<<endl;
		tmp = X * tmp;
		//cout << "tmp:" << endl << tmp << endl;
		Pos0 += tmp;
		Pos0[3] = 1;
	}
	Pos[JointNum] = Pos0;
	return 0;
}

double * JointChain::GetTheta(int i)
{
	assert(i < this->JointNum);
	return chain[i].GetTheta();
}
VectorXd * JointChain::GetState()
{
	return Pos;
}
int JointChain::GetNum()
{
	return JointNum;
}
double JointChain::GetLength(int id)
{
	assert(id < this->JointNum);
	return this->chain[id].GetLength();
}

MatrixXd * JointChain::GetRot(int id)
{
	assert(id < this->JointNum);
	return this->chain[id].GetRot();
}

VectorXd JointChain::GetDerRot(int id, int xyz)
{
	assert(id < this->JointNum);

	MatrixXd res = MatrixXd::Identity(4, 4);
	chain[id].GetDerRot(xyz, res);
	cout << "DerRot:\n" << res << endl;
	VectorXd vec = VectorXd::Zero(3);
	vec[0] = 1;//may be length;
	vec = res.block<3, 3>(0, 0) * vec;
	cout << "vec:" << vec;
	return vec;
}

void JointChain::SetEndPos(VectorXd endpos)
{
	
	assert(3 == endpos.rows());
	std::cout.precision(3);
	//std::cout << std::scientific;

	// random initialize optimization
	int MAX_ITER = 100;
	double CONVERGENCE_LIMIT = 10e-8;
	int N = JointNum * 3;
	unsigned int status = OPTI_CONT;// 2 is continue optimize, 1 is optimize succ, 0 is optimize fail

	std::random_device rd;
	std::default_random_engine engine(rd());
	std::uniform_int_distribution<> dis(0, 100);
	auto dice = std::bind(dis, engine);
	double *x = new double[N];
	double *x_back = new double[N];

	for (int i = 0; i < this->GetNum(); i++)
	{
		double * tmp = this->GetTheta(i);
		for (int j = 0; j < 3; j++)
		{
			x_back[i * 3 + j] = tmp[j];//backup joint status, if opti fail, restore it.
			x[i * 3 + j] = tmp[j] + 10e-5*dice();//random initialize
		}
	}
	

	// optimize
	int iter = 0;
	double f_value = 0;
	double step = 0.01;
	double *gradient = new double[N];
	double *h = new double[N];
	bool gradient_check = 1;
	memset(gradient, 0, sizeof(double)*N);
	memset(h, 0, sizeof(double)*N);
	MatrixXd Jacobian = MatrixXd::Zero(3, N);
	while (1)
	{
		iter++;

		// update x
		cout << "x:";
		for (int i = 0; i < N; i++)
		{
			x[i] += h[i];
			cout << x[i] << " ";
		}
		cout << endl;

		// calculate function_value & gradient & h(x_delta)
		evalfunc(N, x, &f_value, gradient, Jacobian, h,this, endpos, gradient_check);
		cout << "function value:" << f_value << endl;
		
		// change status
		if (f_value < CONVERGENCE_LIMIT) status = OPTI_SUCC;
		if (iter>MAX_ITER) status = OPTI_FAIL;

		// summary 
		cout << "now:\n" << this->GetState()[this->GetNum()] << endl;
		cout << "goal:\n" << endpos[0] << " " << endpos[1] << " " << endpos[2] << endl;
		if (OPTI_SUCC == status)
		{
			cout << "optimization succ! iter:"<<iter<<endl; break;
		}
		else if (OPTI_FAIL == status)
		{
			cout << "optimization fail!\n"; break;
		}
	}
	// if optimize fail, restore status
	if (0 == OPTI_SUCC)
	{
		for (int i = 0; i < this->GetNum(); i++)
		{
			double * tmp = this->GetTheta(i);
			for (int j = 0; j < 3; j++)
			{
				this->SetTheta(i, j, x_back[i * 3 + j]);// restore
			}
		}
	}
	
}