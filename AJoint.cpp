#include "AJoint.h"
#include <iostream>
using namespace std;

AJoint::AJoint()
{
	Rot = MatrixXd::Identity(3, 3);
	Pos = VectorXd::Zero(3);
	Pos[2] = 1;
	Length = 1;
	Theta = 0;
}
int AJoint::SetTheta(double theta)// set rotation theta [0,2*Pi]
{
	Theta = theta;
	
	// calculate rotation matrix
	Rot = MatrixXd::Zero(3, 3);
	//cout << endl << Rot << endl;
	Rot(0, 0) = cos(theta);
	Rot(0, 1) = -1 * sin(theta);
	Rot(1, 0) = sin(theta);
	Rot(1, 1) = cos(theta);
	Rot(2, 2) = 1;

	return 0;
}
int AJoint::SetPos(VectorXd pos) // set links' position, passed in a VectorXd pos
{
	assert(pos.rows() == 3);
	assert(pos[2] == 1);
	Pos = pos;
	return 0;
}
int AJoint::SetLength(double length)// set link's length, a double scalar
{
	Length = length;
	return 0;
}
double AJoint::GetTheta() // get the joint's theta angle
{
	return Theta;
}
double AJoint::GetLength()// query the length of links. return double scalar
{
	return Length;
}
MatrixXd * AJoint::GetRot() // get rotation matrix, return a pointer to Rot MatrixXd
{
	return &Rot;
}
VectorXd * AJoint::GetPos()// get the pointer to position vector
{
	return &Pos;
}