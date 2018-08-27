#pragma once
#include <random>
#include "JointChain.h"

enum Method {
	GNA,
	LMA
};
enum Status {
	OPTI_SUCC,
	OPTI_CONT,
	OPTI_FAIL
};

class Optimizer {
public: 
	Optimizer(JointChain *j);
	bool use(VectorXd &endpos);
	void BindChain(JointChain *pJ);

private:
	JointChain * J;				// pointer to chain
	enum Method method;			// set optimization method
	bool rand_init;				// open random init or not
	int MAX_ITER;				// max iteration time
	double CONVERGENCE_F_LIMIT;	// converenge principle
	enum Status status;
	double init_amp;
	bool gradient_check;		// numericial gradient or not

	bool Init(double *x, double *x_back);				// init varibles, called by Optimizer::use()
	void Judge(int N, double *f_value,\
		double *gradient, int iter, VectorXd endpos);	// judge converge, called by Optimizer::use()
};

extern void evalfunc(int N, double* x, double *f, double *g, MatrixXd &Jacob, double *x_delta, JointChain *J, VectorXd end_effector, bool check);