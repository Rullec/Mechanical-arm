#include "../JointChain.h"
#include <iostream>
Lite_Sparse_Matrix* m_sparse_matrix = 0;

// varialbe declare
extern JointChain *J;
extern const int JointNum;
// Global variable 
VectorXd end_effector;

double calNumGra(double *x, int i, int j);
//////////////////////////////////////////////////////////////////////////
void evalfunc(int N, double* x, double *prev_x, double* f, double* g) //get function value,
{	//N is varialbes number, x is varibles valur, prev_x is last iteration value
	// f is the function and g is gradient value arrays
	// y = x^ - 2

	//cout << endl;
	for (int i = 0; i < JointNum; i++)
	{
		for (int j = 0; j < 3; j++) J->SetTheta(i, j, x[3 * i + j]);//这个API设置的，是从(1,0,0)旋转到当前位置需要转过多少度。
	}
	
	// get *f
	Eigen::VectorXd end_pos = (J->GetState())[JointNum];
	std::cout << "end_pos" << end_pos << std::endl;
	double tmp[3];
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
			std::cout << "gradient:" << j << ":" << g[3 * i + j] << std::endl;
			double c = calNumGra(x, i, j);
			if (abs(c - g[3 * i + j]) > 0.001)
			{
				std::cout << "ERROR! exceed limit!\n";
			}
		}
	}
}
double calNumGra(double *x, int j, int i)
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

//////////////////////////////////////////////////////////////////////////
void newiteration(int iter, int call_iter, double *x, double *f, double *g, double* gnorm)
{ 	//collect internal function information after each iteration
	std::cout << iter << ": " << "call_iter " << call_iter << " f: " << *f << " gnorm: " << *gnorm << std::endl;
	//std::cout << cal(3, x)<<std::endl;
}
//////////////////////////////////////////////////////////////////////////
void Optimize_by_HLBFGS(int N, double *init_x, int num_iter, int M, int T, bool with_hessian, VectorXd & endpos)
{
	end_effector = endpos;
	double parameter[20];
	int info[20];
	//initialize
	INIT_HLBFGS(parameter, info);
	info[0] = 200;
	info[3] = 1;
	info[4] = 10 * num_iter;
	info[6] = T;
	info[7] = with_hessian ? 1 : 0;//使用hessian矩阵
	info[10] = 0;
	info[11] = 1;

	HLBFGS(N, M, init_x, evalfunc, 0, HLBFGS_UPDATE_Hessian, newiteration, parameter, info);
}