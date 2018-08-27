#include "JointChain.h"
#include <iostream>
#include <Eigen/Dense>

double calNumGra(double *x, int j, int i, JointChain *J, VectorXd end_effector);
void calNumJac(double *x, MatrixXd Jacob, JointChain *J);
//////////////////////////////////////////////////////////////////////////

void evalfunc(int N, double* x, double *f, double *g, MatrixXd &Jacob, double *x_delta, JointChain *J, VectorXd end_effector, bool check) //get function value,
{	//N is varialbes number, x is varibles valur, prev_x is last iteration value
	// f is the function and g is gradient value arrays
	// y = x^ - 2
	int JointNum = J->GetNum();
	//cout << endl;
	for (int i = 0; i < JointNum; i++)
	{
		for (int j = 2; j >= 0; j--) J->SetTheta(i, j, x[3 * i + j]);//这个API设置的，是从(1,0,0)旋转到当前位置需要转过多少度。
	}

	// get *f
	Eigen::VectorXd end_pos = (J->GetState())[JointNum];
	std::cout << "end_pos" << end_pos << std::endl;
	VectorXd tmp = VectorXd::Zero(3);
	*f = 0;// clear *f
	for (int i = 0; i < 3; i++)
	{
		tmp[i] = end_pos[i] - end_effector[i];
		*f += tmp[i] * tmp[i] / 2;
	}
	std::cout << *f << std::endl;

	// get df = g[3*i + j]
	for (int i = 0; i < JointNum; i++)
	{
		g[3 * i] = g[3 * i + 1] = g[3 * i + 2] = 0;
		for (int j = 0; j < 3; j++)
		{
			VectorXd vec = J->GetDerRot(i, j);
			g[3 * i + j] = tmp[0] * vec[0] + tmp[1] * vec[1] + tmp[2] * vec[2];
			Jacob(0, 3 * i + j) =  vec[0];
			Jacob(1, 3 * i + j) = vec[1];
			Jacob(2, 3 * i + j) =  vec[2];

			std::cout << "gradient:" << j << ":" << g[3 * i + j] << std::endl;
			if (check)
			{
				double c = calNumGra(x, i, j, J, end_effector);
				if (abs(c - g[3 * i + j]) > 0.001)
				{
					std::cout << "ERROR! exceed limit!\n";
				}
			}

		}
	}
	std::cout << "\nJacobi:" << Jacob << std::endl;
	calNumJac(x, Jacob, J);
	//calculate x_delta
	MatrixXd Par = Jacob.transpose() * Jacob;
	double det = Par.determinant();
	if (abs(det) < 10e-6)
	{
		Par = Par + 10e-6*MatrixXd::Identity(N, N);
	}

	VectorXd h;
	h = -1 * Par.inverse() * Jacob.transpose() * tmp;
	std::cout << "h is :";
	for (int i = 0; i < N; i++)
	{
		x_delta[i] = h[i];
		std::cout << x_delta[i];
	}
	std::cout << std::endl;
}
void calNumJac(double *x, MatrixXd Jacob, JointChain *J)
{
	double delta = 0.00001;
	for (int i = 0; i < J->GetNum(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			J->SetTheta(i, j, x[3 * i + j] + delta);
			VectorXd s = (J->GetState())[J->GetNum()];
			J->SetTheta(i, j, x[3 * i + j] - delta);
			VectorXd t = (J->GetState())[J->GetNum()];

			s = (s - t) / (2 *  delta);
			std::cout << s << std::endl;
			for (int m = 0; m < 3; m++)
			{
				double c = Jacob(m, 3 * i + j) - s[m];
				if (abs(c) > 0.001)
				{
					std::cout << "error! jacob wrong!\n";
				}
			}
			J->SetTheta(i, j, x[3 * i + j]);

		}
	}
}
double calNumGra(double *x, int j, int i, JointChain *J, VectorXd end_effector)
{	//第i个铰链的第j个角。
	double delta = 0.0001;

	VectorXd s, t;
	double f1 = 0, f2 = 0;
	J->SetTheta(j, i, x[3 * j + i] + delta);
	s = J->GetState()[J->GetNum()];
	//cout << s << endl;
	f1 = ((s[0] - end_effector[0]) * (s[0] - end_effector[0]) + (s[1] - end_effector[1]) * (s[1] - end_effector[1])\
		+ (s[2] - end_effector[2]) * (s[2] - end_effector[2])) / 2;

	J->SetTheta(j, i, x[3 * j + i] - delta);
	t = J->GetState()[J->GetNum()];
	//cout << t << endl;
	f2 = ((t[0] - end_effector[0]) * (t[0] - end_effector[0]) + (t[1] - end_effector[1]) * (t[1] - end_effector[1])\
		+ (t[2] - end_effector[2]) * (t[2] - end_effector[2])) / 2;

	double g1 = (f1 - f2) / (2 * delta);
	std::cout << "Numerical Gradient: " << i << " : " << g1 << std::endl;
	J->SetTheta(j, i, x[3 * j + i]);

	return g1;
}
