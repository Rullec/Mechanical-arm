#pragma once
#include <Eigen/Dense>
#include "AJoint.h"
using namespace Eigen;

class JointChain {
	//represent joints' chain.
public:
	JointChain(VectorXd Pos0, int Num, double *length);
	int SetTheta(int id, int xyz, double theta);// manipulate theta between 0 - 2 * PI
	double * GetTheta(int i);
	VectorXd * GetState();
	double GetLength(int id);
	int GetNum();
	MatrixXd *GetRot(int id);

private:
	int JointNum = 0;
	AJoint *chain = NULL;
	VectorXd *Pos = NULL; // Pos���ڴ洢�������꣬����˵Ӧ����JointNum+1��Ԫ�ء�
};