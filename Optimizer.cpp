#include "Optimizer.h"

Optimizer::Optimizer(JointChain *pJ)
{
	J = pJ;
	method = LMA;					// set optimization method
	rand_init = true;				// open random initialize 
	MAX_ITER = 100;					// set max iteration times
	CONVERGENCE_F_LIMIT = 10e-8;	// set funcation value converge limit
	status = OPTI_CONT;				// initialize status
	init_amp = 10e-5;				// non zero rand initialize amplitude
	gradient_check = true;			// open gradient_check
}

void Optimizer::BindChain(JointChain *pJ)
{
	J = pJ;
}

bool Optimizer::use( VectorXd &endpos)
{// this function is used to optimize jointchain's tail to goal postion, such as from (4,0,0) to (1,-2,-1)

	// random initialize optimization to avoid "all zero begin"
	int N = 3 * J->GetNum();
	double *x = new double[N], *x_back = new double[N];
	this->Init(x, x_back);


	// optimize
	int iter = 0;
	double f_value = 0;
	double step = 0.01;
	double *gradient = new double[N];
	double *h = new double[N];

	memset(gradient, 0, sizeof(double)*N);
	memset(h, 0, sizeof(double)*N);
	MatrixXd Jacobian = MatrixXd::Zero(3, N);
	while (1)
	{
		iter++;

		// update x
		std::cout << "x:";
		for (int i = 0; i < N; i++)
		{
			x[i] += h[i];
			std::cout << x[i] << " ";
		}
		std::cout << std::endl;

		// calculate function_value & gradient & h(x_delta)
		evalfunc(N, x, &f_value, gradient, Jacobian, h, this->J, endpos, gradient_check);
		
		// judge status --- convergence or divergent
		Judge(N, &f_value, gradient, iter, endpos);
		
		if (OPTI_CONT != status) break;
	}
	// if optimize fail, restore status
	if (OPTI_FAIL == status)
	{
		for (int i = 0; i < this->J->GetNum(); i++)
		{
			double * tmp = this->J->GetTheta(i);
			for (int j = 0; j < 3; j++)
			{
				this->J->SetTheta(i, j, x_back[i * 3 + j]);// restore
			}
		}
	}
	if (OPTI_SUCC == status) return true;
	else return false;
}

bool Optimizer::Init(double *x, double *x_back)
{
	// random initialize optimization
	int N = J->GetNum() * 3;
	unsigned int status = OPTI_CONT;// 2 is continue optimize, 1 is optimize succ, 0 is optimize fail

	std::random_device rd;
	std::default_random_engine engine(rd());
	std::uniform_int_distribution<> dis(0, 100);
	auto dice = std::bind(dis, engine);


	for (int i = 0; i < this->J->GetNum(); i++)
	{
		double * tmp = this->J->GetTheta(i);
		for (int j = 0; j < 3; j++)
		{
			x_back[i * 3 + j] = tmp[j];	//backup joint status, if opti fail, restore it.
			x[i * 3 + j] = tmp[j] + init_amp * dice();//random initialize
		}
	}

	status = OPTI_CONT;					// initialize status
	return true;
}

void Optimizer::Judge(int N, double *f_value, double *gradient, int iter, VectorXd endpos)
{
	// output function value
	std::cout << "function value:" << f_value << std::endl;

	// change status
	if (*f_value < CONVERGENCE_F_LIMIT) status = OPTI_SUCC;
	if (iter > MAX_ITER) status = OPTI_FAIL;

	// summary 
	std::cout << "now:\n" << J->GetState()[this->J->GetNum()] << std::endl;
	std::cout << "goal:\n" << endpos[0] << " " << endpos[1] << " " << endpos[2] << std::endl;
	if (OPTI_SUCC == status)
	{
		std::cout << "optimization succ! iter:" << iter << std::endl;
	}
	else if (OPTI_FAIL == status)
	{
		std::cout << "optimization fail!\n";
	}
}