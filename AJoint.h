#pragma once
#include <Eigen/Dense>
using namespace Eigen;

class AJoint {
	// represent a joint
public:
	AJoint();
	int SetTheta(double theta);// set rotation theta [0,2*Pi]
	int SetPos(VectorXd pos);// set links' position, passed in a VectorXd pos
	int SetLength(double length);// set link's length, a double scalar
	double GetTheta();// get the joint's theta angle
	double GetLength();// query the length of links. return double scalar
	MatrixXd *GetRot();// get rotation matrix, return a pointer to Rot MatrixXd
	VectorXd *GetPos();// get the pointer to position vector

private:
	MatrixXd Rot; // rotation matrix 3*3
	VectorXd Pos; // joint position 3*1
	double Length, Theta;
};