#include "nrOptimization.h"

#include "nrTransformation.h"

#include "unsupported/Eigen/SparseExtra"

#include <iostream>
#include <chrono>

namespace nonrigid
{

template<class InputType, class DerivativeType>
nrOptimization<InputType, DerivativeType>::nrOptimization(
	std::shared_ptr<nrEnergyFunction<InputType, DerivativeType>> objective)
	:objective(objective),
	current_iteration(0)
{
}

template<class InputType, class DerivativeType>
nrOptimization<InputType, DerivativeType>::~nrOptimization()
{
}

nrGaussNewtonOptimization::nrGaussNewtonOptimization(
	std::shared_ptr<nrLeastSquaresFunction<Eigen::VectorXd>> objective,
	const Eigen::VectorXd& starting_point)
	:nrOptimization(objective),
	current_point(starting_point),
	should_converge(false)
{
	x_k.push_back(current_point);
}

Eigen::VectorXd nrGaussNewtonOptimization::argmin()
{
	return x_k.back();
}

Result nrGaussNewtonOptimization::run()
{
	Result result;

	while (true)
	{
		result = one_step();

		if (result == Result::FAIL)
		{
			return result;
		}

		if (converged())
		{
			break;
		}
	}

	return Result::SUCCESS;
}

nrGaussNewtonOptimization::~nrGaussNewtonOptimization()
{
}

bool nrGaussNewtonOptimization::converged()
{
	if (should_converge)
	{
		return true;
	}

	if (current_iteration <= 1)
	{
		return false;
	}

	double F_k = values[current_iteration - 1];
	double F_kminus1 = values[current_iteration - 2];

	/*
	if (current_iteration > 15)
	{
		return true;
	}*/

	//FIRST CONDITION
	bool condition = std::abs(F_k - F_kminus1) < eps * (1 + F_k);
	if (condition)
	{
		return true;
	}
	/*
	//SECOND CONDITION
	std::shared_ptr<Eigen::RowVectorXd> gradient = grad[current_iteration - 1];

	double gradient_inf = std::numeric_limits<double>::max();
	if (gradient->size() != 0)
	{
	gradient_inf = gradient->lpNorm<Eigen::Infinity>();
	}

	condition = gradient_inf < std::pow(eps, 1.0 / 3) * (1 + F_k);
	if (condition)
	{
	return true;
	}

	//THIRD CONDITION
	double delta_inf = delta_k[current_iteration - 1].lpNorm<Eigen::Infinity>();
	condition = delta_inf < std::sqrt(eps)*(1 + delta_inf);

	if (condition)
	{
		return true;
	}*/

	return false;
}

Result nrGaussNewtonOptimization::one_step()
{
	std::chrono::time_point<std::chrono::high_resolution_clock> start =
		std::chrono::high_resolution_clock::now();

	current_iteration++;
	int size = current_point.size();

	std::shared_ptr<Jacobian> j = objective->df(current_point);
	const Jacobian jt = j->transpose();

	/*
	Eigen::saveMarket(*j, std::string("jacobian") + std::to_string(current_iteration));
	std::ofstream f4(std::string("x") + std::to_string(current_iteration));
	f4 << current_point;*/

	std::shared_ptr<Eigen::VectorXd> residuals =
		std::dynamic_pointer_cast<nrLeastSquaresFunction<Eigen::VectorXd>>(objective)->
		eval_residuals(current_point);
	double current_value = 0.5 * residuals->squaredNorm();
	values.push_back(current_value);
	
	std::cout << "iteration " << current_iteration << ": " << current_value << std::endl;
	std::ofstream f(std::string("residuals") + std::to_string(current_iteration));
	f << *residuals;
	
	std::shared_ptr<Eigen::RowVectorXd> gr =
		std::dynamic_pointer_cast<nrLeastSquaresFunction<Eigen::VectorXd>>(
			objective)->gradient();
	grad.push_back(gr);
	std::cout << "gradient squared norm: " << gr->squaredNorm() << std::endl;
	
	if (jt.cols() <= jt.rows())
	{
		return Result::FAIL;
	}

	Eigen::SparseMatrix<double> jtj = (jt * (*j));		
	Eigen::SparseMatrix<double> m_eps(size, size);
	m_eps.setIdentity();
	m_eps = eps * m_eps;

	jtj += m_eps;

	//Eigen::saveMarket(jtj, std::string("jtj") + std::to_string(current_iteration));

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
	ldlt.compute(jtj);

	if (ldlt.info() != Eigen::Success)
	{
		return Result::FAIL;
	}
	
	Eigen::VectorXd b = -jt * (*residuals);

	/*
	std::ofstream f2(std::string("b") + std::to_string(current_iteration));
	f2 << b;*/

	Eigen::VectorXd delta = ldlt.solve(b);

	/*
	std::ofstream f3(std::string("delta") + std::to_string(current_iteration));
	f3 << delta;*/

	delta_k.push_back(delta);	

	residuals_all.push_back(residuals);
	
	std::chrono::time_point<std::chrono::high_resolution_clock> end =
		std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_elapsed = end - start;
	std::cout << "step time: " << time_elapsed.count() << std::endl;
	
	double alpha = step_size(current_point, delta, current_value);	
	
	if (alpha > eps)
	{
		current_point += alpha * delta;
		x_k.push_back(current_point);
	}
	else {
		should_converge = true;
	}	

	return Result::SUCCESS;
}

double nrGaussNewtonOptimization::step_size(
	const Eigen::VectorXd& point,
	const Eigen::VectorXd& step,
	const double& value) const
{
	double alpha = 1.0;

	Eigen::VectorXd new_point = point + alpha*step;

	while (alpha > eps)
	{
		std::cout << "step size: " << alpha << std::endl;

		double new_value = objective->operator()(new_point);

		if (new_value < value)
		{
			return alpha;
		}

		alpha /= 2.0;
	}

	return 0.0;
}

}