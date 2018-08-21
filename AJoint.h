#pragma once
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
// 2018/8/16 15:00 from 2d to 3d

class AJoint {
	// represent a joint
public:
	AJoint();
	int SetTheta(int xyz, double theta);	// set x/y/z axis 's rotation theta [0,2*Pi]
	int SetPos(VectorXd pos);				// set links' position, passed in a VectorXd pos 4*1
	int SetLength( double length);			// set link's length, a double scalar
	double *GetTheta();						// get the joint's three theta angles [theta_x, theta_y, theta_z]
	double GetLength();						// query the length of links. return double array 3*1
	MatrixXd *GetRot(int i);				// get xyz rotation matrix, return a pointer to Rot MatrixXd 4*4
	MatrixXd *GetRot();						// get SUM rotation matrix, return a pointer to Rot MatrixXd 4*4
	VectorXd *GetPos();						// get the pointer to position vector: 4*1

private:
	MatrixXd Rot, Rotx, Roty, Rotz; // rotation matrix 4*4
	VectorXd Pos; // joint position 4*1
	double Length;
	double *Theta;
};