#include "nrOptimization.h"

#include <iostream>

namespace nonrigid
{
	std::shared_ptr<nrLeastSquaresFunction<Eigen::VectorXd>> 
		nrLBFGSOptimization::objective;
	Eigen::VectorXd nrLBFGSOptimization::current_point;

	lbfgsfloatval_t* nrLBFGSOptimization::point;

	Result nrLBFGSOptimization::run()
	{
		int status;

		point = static_cast<lbfgsfloatval_t*>(current_point.data());
		lbfgs_parameter_t params;
		lbfgs_parameter_init(&params);
		lbfgsfloatval_t fx;

		status = lbfgs(current_point.size(), point, &fx, evaluate, progress, NULL, &params);

		std::cout << "fx: " << fx  << std::endl;
		std::cout << "status: " << status;

		if (status)
		{
			return Result::FAIL;
		}
		else {
			return Result::SUCCESS;
		}
	}

	Eigen::VectorXd nrLBFGSOptimization::argmin()
	{
		Eigen::Map<Eigen::VectorXd> map(static_cast<double*>(point), current_point.size());
		return map;
	}

	lbfgsfloatval_t nrLBFGSOptimization::evaluate(
		void *instance,
		const lbfgsfloatval_t *x,
		lbfgsfloatval_t *g,
		const int n,
		const lbfgsfloatval_t step)
	{
		Eigen::Map<const Eigen::VectorXd> map(static_cast<const double*>(x), n);
		double result = objective->operator()(map);

		objective->df(map);
		std::shared_ptr<Eigen::RowVectorXd> gr = objective->gradient();		
		for (int i = 0; i < n; i++)
		{
			g[i] = gr->operator[](i);
		}

		lbfgsfloatval_t sqnorm = 0.0;
		for (int i = 0; i < n; i++)
		{
			sqnorm += g[i] * g[i];
		}
		std::cout << "gr norm: " << std::sqrt(sqnorm) << std::endl;
		std::cout << "step: " << step << std::endl;

		return static_cast<lbfgsfloatval_t>(result);
	}

	int nrLBFGSOptimization::progress(
		void *instance,
		const lbfgsfloatval_t *x,
		const lbfgsfloatval_t *g,
		const lbfgsfloatval_t fx,
		const lbfgsfloatval_t xnorm,
		const lbfgsfloatval_t gnorm,
		const lbfgsfloatval_t step,
		int n,
		int k,
		int ls)
	{
		std::cout << "fx: " << fx << '\t' << "gnorm: " << gnorm << std::endl;

		return 0;
	}
}