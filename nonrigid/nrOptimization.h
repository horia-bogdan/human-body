#ifndef NROPTIMIZATION_H
#define NROPTIMIZATION_H

#include "nrEnergyFunction.h"

extern "C" {
#include<lbfgs.h>
}

#include<Eigen\Dense>

#include<memory>

namespace nonrigid
{

enum Result {
	SUCCESS,
	FAIL
};

template<class InputType, class DerivativeType>
class nrEnergyFunction;

//optimization base class
template<class InputType, class DerivativeType>
class nrOptimization
{
public:

	nrOptimization(std::shared_ptr<nrEnergyFunction<InputType, DerivativeType>>);

	virtual Result run() = 0;

	virtual InputType argmin() = 0;
	
	virtual ~nrOptimization();

protected:
	int current_iteration;
	
	virtual bool converged() = 0;

	virtual Result one_step() = 0;

	std::shared_ptr<nrEnergyFunction<InputType, DerivativeType>> objective;
};

class nrGaussNewtonOptimization
	:nrOptimization<Eigen::VectorXd, std::shared_ptr<Jacobian>>
{
public:
	nrGaussNewtonOptimization(
		std::shared_ptr<nrLeastSquaresFunction<Eigen::VectorXd>>,
		const Eigen::VectorXd& starting_point);
	
	virtual Eigen::VectorXd argmin();

	virtual Result run();

	virtual ~nrGaussNewtonOptimization();

protected:
	bool converged();

	virtual Result one_step();

	virtual double step_size(
		const Eigen::VectorXd& point,
		const Eigen::VectorXd& step,
		const double& value) const;

private:
	Eigen::VectorXd current_point;
	
	const double eps = 1e-8;
	bool should_converge;

	std::vector<Eigen::VectorXd> x_k;
	std::vector<Eigen::VectorXd> delta_k;
	std::vector<std::shared_ptr<Eigen::VectorXd>> residuals_all;
	std::vector<double> values;

	std::vector<std::shared_ptr<Eigen::RowVectorXd>> grad;
};


namespace nrLBFGSOptimization 
{
	extern std::shared_ptr<nrLeastSquaresFunction<Eigen::VectorXd>> objective;
	extern Eigen::VectorXd current_point;
	
	extern lbfgsfloatval_t* point;

	Result run();
	Eigen::VectorXd argmin();

	static lbfgsfloatval_t evaluate(
		void *instance,
		const lbfgsfloatval_t *x,
		lbfgsfloatval_t *g,
		const int n,
		const lbfgsfloatval_t step);

	static int progress(
		void *instance,
		const lbfgsfloatval_t *x,
		const lbfgsfloatval_t *g,
		const lbfgsfloatval_t fx,
		const lbfgsfloatval_t xnorm,
		const lbfgsfloatval_t gnorm,
		const lbfgsfloatval_t step,
		int n,
		int k,
		int ls);
}


}



#endif

