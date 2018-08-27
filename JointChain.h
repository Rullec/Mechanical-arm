#pragma once
#include <Eigen/Dense>
#include "AJoint.h"
#define OPTI_SUCC 2
#define OPTI_CONT 1
#define OPTI_FAIL 0
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
	VectorXd GetDerRot(int id, int xyz);// used to get derivate value
	void SetEndPos(VectorXd endpos);//this method is used to directly set end effector's position with Inverse Kinametics knowledge.
	

private:
	int JointNum = 0;
	AJoint *chain = NULL;
	VectorXd *Pos = NULL; // Pos用于存储铰链坐标，所以说应该是JointNum+1个元素。
};

extern void evalfunc(int N, double* x, double *f, double *g, MatrixXd &Jacob, double *x_delta, JointChain *J, VectorXd end_effector, bool check);