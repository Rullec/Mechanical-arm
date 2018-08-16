#pragma once
#include <Eigen/Dense>
#include "AJoint.h"
using namespace Eigen;

class JointChain {
	//represent joints' chain.
public:
	JointChain(VectorXd Pos0, int Num, double *length);
	int SetTheta(int id, double theta);
	double GetTheta(int i);
	VectorXd * GetState();
	int GetNum();

private:
	int JointNum = 0;
	AJoint *chain = NULL;
	VectorXd *Pos = NULL; // Pos���ڴ洢�������꣬����˵Ӧ����JointNum+1��Ԫ�ء�
};