#include "AJoint.h"
#include <iostream>
using namespace std;

AJoint::AJoint()
{
	//initialize Rotation Matrix
	Rot = MatrixXd::Identity(4, 4);
	Rotx = MatrixXd::Identity(4, 4);
	Roty = MatrixXd::Identity(4, 4);
	Rotz = MatrixXd::Identity(4, 4);

	//initialize Position Vector
	Pos = VectorXd::Zero(4);
	Pos[3] = 1;

	//initialize three x y z axises' rotation angle
	Theta = new double[3];
	memset(Theta, 0, sizeof(double) * 3);

	//initialize length of this link
	Length = 1;
}
int AJoint::SetTheta(int id, double theta)// set rotation theta [0,2*Pi]
{
	assert(id <= 2);
	Theta[id] = theta;
	
	int first = 0, second = 1;
	switch (id) {
	case 0:first = 1, second = 2; break;
	case 1:first = 0, second = 2; break;
	case 2:first = 0, second = 1; break;
	default:assert(0);
	}
	MatrixXd tmp = MatrixXd::Identity(4, 4);
	
	//cout << endl << Rot << endl;
	tmp(first, first) = cos(theta);
	tmp(first, second) = -1 * sin(theta);
	tmp(second, first) = sin(theta);	
	tmp(second, second) = cos(theta);

	// 左手系特例：绕Y轴旋转矩阵取反
	if (1 == id)
	{
		tmp(first, second) =  sin(theta);
		tmp(second, first) = -1 * sin(theta);
	}
	switch (id) {
	case 0:Rotx = tmp; break;
	case 1:Roty = tmp; break;
	case 2:Rotz = tmp; break;
	default:assert(0);
	}
	Rot = Rotz * Roty * Rotx;
	//cout << Rot;
	return 0;
}
int AJoint::SetPos(VectorXd pos) // set links' position, passed in a VectorXd pos
{
	assert(pos.rows() == 4);
	assert(pos[3] == 1);
	Pos = pos;
	return 0;
}
int AJoint::SetLength(double length)// set link's length, a double scalar
{
	Length = length;
	return 0;
}
double *AJoint::GetTheta() // get the pointer to joint's theta angles s' array 
{
	return Theta;
}
double AJoint::GetLength()// query the length of links. return double scalar
{
	return Length;
}
MatrixXd * AJoint::GetRot(int id) // get rotation matrix, return a pointer to Rot MatrixXd
{
	switch (id) {
	case 0:return &Rotx; break;
	case 1:return &Roty; break;
	case 2:return &Rotz; break;
	default:assert(0); return NULL;
	}
}
MatrixXd *AJoint::GetRot()
{
	return &Rot;
}
VectorXd * AJoint::GetPos()// get the pointer to position vector
{
	return &Pos;
}
void AJoint::GetDerRot(int xyz, MatrixXd &res)			// get the Jacobian left mulitply matrix used in IK
{

	assert(xyz <= 2);

	int first = 0, second = 1;
	switch (xyz) {
	case 0:first = 1, second = 2; break;
	case 1:first = 0, second = 2; break;
	case 2:first = 0, second = 1; break;
	default:assert(0);
	}
	MatrixXd tmp = MatrixXd::Zero(4, 4);
	//res = MatrixXd::Identity(4, 4);
	for (int i = 0; i < 3; i++) cout << i << ": " << Theta[i] << endl;
	cout << endl << Rot << endl;
	tmp(first, second) = -1 * cos(Theta[xyz]);
	tmp(second, first) = cos(Theta[xyz]);
	tmp(first, first) = -1 * sin(Theta[xyz]);
	tmp(second, second) = -1 * sin(Theta[xyz]);

	// 左手系特例：绕Y轴旋转矩阵取反
	if (1 == xyz)
	{
		tmp(first, second) = cos(Theta[xyz]);
		tmp(second, first) = -1 * cos(Theta[xyz]);
	}
	cout << "tmp:\n" << tmp << endl;
	switch (xyz) {
	case 0:res = Rotz * Roty * tmp; break;
	case 1:res = Rotz * tmp * Rotx; break;
	case 2:res = tmp * Roty * Rotx; break;
	default:assert(0);
	}
}