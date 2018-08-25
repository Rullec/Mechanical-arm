#ifdef _DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif

#include <iostream>
#include <vector>
#include "HLBFGS.h"
#include "Lite_Sparse_Matrix.h"
#include "JointChain.h"
#define PI 3.14159265359
using namespace std;

JointChain *J = NULL;
const int JointNum = 2;
std::vector<double> end_effector(3);
Lite_Sparse_Matrix* m_sparse_matrix = 0;


void calNumGra(double *x);
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
	Eigen::VectorXd end_pos = (J->GetState())[J->GetNum()];
	cout << "end_pos" << end_pos << endl;
	double tmp[3];
	*f = 0;
	for (int i = 0; i < 3; i++)
	{
		tmp[i] = end_pos[i] - end_effector[i];
		*f += tmp[i] * tmp[i] / 2;
	}
	cout << *f;
	for (int i = 0; i < JointNum; i++)
	{
		g[3 * i] = g[3 * i + 1] = g[3 * i +2] = 0;
		for (int j = 0; j < 3; j++)
		{
			VectorXd vec = J->GetDerRot(i, j);
			g[3 * i + j] = tmp[0] * vec[0] + tmp[1] * vec[1] + tmp[2] * vec[2];
			cout << "gradient:" << j <<":"<< g[3 * i + j] << endl;
		}
	}
	//calNumGra(x);
}
void calNumGra(double *x)
{
	double delta = 0.01;
	for (int j = 0; j < JointNum; j++)
	{
		for (int i = 0; i < 3; i++)
		{
			VectorXd s, t;
			double f1 = 0, f2 = 0;
			J->SetTheta(j, i, x[i] + delta);
			s = J->GetState()[J->GetNum()];
			//cout << s << endl;
			f1 = ((s[0] - end_effector[0]) * (s[0] - end_effector[0]) + (s[1] - end_effector[1]) * (s[1] - end_effector[1])\
				+ (s[2] - end_effector[2]) * (s[2] - end_effector[2])) / 2;

			J->SetTheta(j, i, x[i] - delta);
			t = J->GetState()[J->GetNum()];
			//cout << t << endl;
			f2 = ((t[0] - end_effector[0]) * (t[0] - end_effector[0]) + (t[1] - end_effector[1]) * (t[1] - end_effector[1])\
				+ (t[2] - end_effector[2]) * (t[2] - end_effector[2])) / 2;

			double g1 = (f1 - f2) / (2 * delta);
			cout << "Numerical Gradient: " << i << " : " << g1 << endl;
			J->SetTheta(0, i, x[i]);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void newiteration(int iter, int call_iter, double *x, double *f, double *g, double* gnorm)
{ 	//collect internal function information after each iteration
	std::cout << iter << ": " << "call_iter "<<call_iter << " f: " << *f << " gnorm: " << *gnorm << std::endl;
	//cout << cal(3, x)<<endl;
}
//////////////////////////////////////////////////////////////////////////
void evalfunc_h(int N, double *x, double *prev_x, double *f, double *g, HESSIAN_MATRIX& hessian)
{
	// it is a evalfunc_h function that should be use in Hessian optimaztion, add HESSIAN_MATRIX &
	// WE MUST CREATE A NEW HESSIAN_MATRIX AND FILL IT WITH CERTAIN VALUE!
	// the following code is not optimal if the pattern of hessian matrix is fixed.

	if (m_sparse_matrix)
	{
		delete m_sparse_matrix;
	}

	m_sparse_matrix = new Lite_Sparse_Matrix(N, N, SYM_LOWER, CCS, FORTRAN_TYPE, true);

	m_sparse_matrix->begin_fill_entry();

	static bool first = true;
	double *diag = m_sparse_matrix->get_diag();

	if (first)
	{
		// you need to update f and g
		*f = 0;
		double tmp;
		for (int i = 0; i < N; i += 2)
		{
			tmp = x[i] * x[i];
			double T1 = 1 - x[i];
			double T2 = 10 * (x[i + 1] - tmp);
			*f += T1 * T1 + T2 * T2;
			g[i + 1] = 20 * T2;
			g[i] = -2 * (x[i] * g[i + 1] + T1);
			diag[i] = 2 + 1200 * tmp - 400 * x[i + 1];
			diag[i + 1] = 200;
			m_sparse_matrix->fill_entry(i, i + 1, -400 * x[i]);
		}
	}
	else
	{
		for (int i = 0; i < N; i += 2)
		{
			diag[i] = 2 + 1200 * x[i] * x[i] - 400 * x[i + 1];
			diag[i + 1] = 200;
			m_sparse_matrix->fill_entry(i, i + 1, -400 * x[i]);
		}
	}

	m_sparse_matrix->end_fill_entry();

	hessian.set_diag(m_sparse_matrix->get_diag());
	hessian.set_values(m_sparse_matrix->get_values());
	hessian.set_rowind(m_sparse_matrix->get_rowind());
	hessian.set_colptr(m_sparse_matrix->get_colptr());
	hessian.set_nonzeros(m_sparse_matrix->get_nonzero());
	first = false;
}
//////////////////////////////////////////////////////////////////////////
void Optimize_by_HLBFGS(int N, double *init_x, int num_iter, int M, int T, bool with_hessian)
{
	double parameter[20];
	int info[20];
	//initialize
	INIT_HLBFGS(parameter, info);
	info[4] = num_iter;
	info[6] = T;
	info[7] = with_hessian ? 1 : 0;//使用hessian矩阵
	info[10] = 0;
	info[11] = 1;

	HLBFGS(N, M, init_x, evalfunc, 0, HLBFGS_UPDATE_Hessian, newiteration, parameter, info);
}
//////////////////////////////////////////////////////////////////////////

void main()
{

	std::cout.precision(8);
	//std::cout << std::scientific;
	
	// initialize the JointChain
	double *length = new double[JointNum];
	for (int i = 0; i < JointNum; i++) length[i] = 1;
	Eigen::VectorXd initpos = VectorXd::Zero(4);
	initpos[3] = 1;
	J = new JointChain(initpos, JointNum, length);//初始位置在0,0,0

	// initialize optimization
	int M = 7;
	int T = 0;
	int N = 3 * JointNum;
	double t1 =2, t2 = 1, t3 = 0;
	double sum = pow(cos(t1)*cos(t1) + cos(t2)*cos(t2) + cos(t3)*cos(t3), 0.5);
	end_effector[0] = cos(t1)/sum;
	end_effector[1] = cos(t2)/sum;
	end_effector[2] = cos(t3)/sum;

	vector<double> x(N, 0);
	x[4] = PI / 4;
	Optimize_by_HLBFGS(N, &x[0], 2000, M, T, false);  // it is LBFGS(M) actually, T is not used
	cout << "now:\n" << J->GetState()[J->GetNum()] << endl;
	cout << "goal:\n" << end_effector[0] <<" "<< end_effector[1] << " " << end_effector[2] << endl;

	int a;
	std::cin >> a;
	return;
}