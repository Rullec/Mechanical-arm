#include <iostream>
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
		cout << "Pos0:" << endl << Pos0 << endl;
		Pos[i] = Pos0;
		chain[i].SetPos(Pos0);
		double length = chain[i].GetLength();
		VectorXd tmp = VectorXd::Zero(4);
		tmp(0) = length;
		tmp(3) = 1;
		MatrixXd X = *chain[i].GetRot();
		cout << endl << X << endl;
		cout << endl << tmp<<endl;
		tmp = X * tmp;
		cout << "tmp:" << endl << tmp << endl;
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