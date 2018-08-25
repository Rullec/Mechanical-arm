#include <iostream>
#include <random>
#include "JointChain.h"
using namespace std;

extern void Optimize_by_HLBFGS(int N, double *init_x, int num_iter, int M, int T, bool with_hessian, VectorXd &pos);

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
		cout << tmp;
		tmp = (*chain[i].GetRot())*tmp;
		cout << tmp;
		cout << Pos0;
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
	std::cout.precision(8);
	//std::cout << std::scientific;

	// initialize optimization
	int M = 7;
	int T = 0;
	int N = 3 * JointNum;
	/*
	double t1 = 2, t2 = 1, t3 = 0;
	double sum = pow(cos(t1)*cos(t1) + cos(t2)*cos(t2) + cos(t3)*cos(t3), 0.5);
	endpos[0] = cos(t1) / sum;
	endpos[1] = cos(t2) / sum;
	endpos[2] = cos(t3) / sum;
	*/
	// different initialize 
	std::random_device rd;
	std::default_random_engine engine(rd());
	std::uniform_int_distribution<> dis(0, 100);
	auto dice = std::bind(dis, engine);

	vector<double> x(N, 0);
	for (int i = 0; i < this->GetNum(); i++)
	{
		double * tmp = this->GetTheta(i);
		for (int j = 0; j < 3; j++)
		{
			x[i * 3 + j] = tmp[j] + 0.000001*dice();
		}
	}
	Optimize_by_HLBFGS(N, &x[0], 1000, M, T, false, endpos);  // it is LBFGS(M) actually, T is not used
	for (int i = 0; i < JointNum; i++)
	{
		for (int j = 0; j < 3; j++) this->SetTheta(i, j, x[3 * i + j]);//这个API设置的，是从(1,0,0)旋转到当前位置需要转过多少度。
	}
	cout << "now:\n" << this->GetState()[this->GetNum()] << endl;
	cout << "goal:\n" << endpos[0] << " " << endpos[1] << " " << endpos[2] << endl;

}