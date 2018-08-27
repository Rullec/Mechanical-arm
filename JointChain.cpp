#include <iostream>
#include <random>
#include "JointChain.h"
#include "Optimizer.h"
using namespace std;
extern Optimizer *opt;

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

		//��������������ĽǶȣ�x y z�����Ƕȣ����ǵ�ǰ������ĺ�x y z��ʲô�Ƕȣ����ǵ�ǰ������(1, 0, 0)�ȣ���x y z�����ǲ��˶��١�
		// ȫ������Ϊ0,0,0����˼�ǣ���������1,0,0������Ҫ����
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

	// initilize optimizer
	// opt.SetPtrChain(this);
}

int JointChain::SetTheta(int id, int xyz, double theta)
{//��Щtheta�Ƕ��ǻ����Ƶģ���cosҲ���ǻ����Ƶ���
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
	//assert(1 == 0);
	opt->use(endpos);
	//std::cout << std::scientific;
}