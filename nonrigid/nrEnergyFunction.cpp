#include "nrEnergyFunction.h"
#include "nrTransformation.h"

#include<iostream>

namespace nonrigid
{

template<class InputType, class DerivativeType>
nrEnergyFunction<InputType, DerivativeType>::
	nrEnergyFunction()
	: last_value_cache(std::numeric_limits<double>::max())
{
}

template<class InputType, class DerivativeType>
double nrEnergyFunction<InputType, DerivativeType>::last_eval_value() const
{
	return last_value_cache;
}

template<class InputType, class DerivativeType>
nrEnergyFunction<InputType, DerivativeType>::
	~nrEnergyFunction()
{
}

template<class InputType>
nrLeastSquaresFunction<InputType>::nrLeastSquaresFunction(
	std::unique_ptr<std::vector<std::unique_ptr<nrEnergyFunction<InputType, std::unique_ptr<JacobianRow>>>>>
		function_vector)
	:nrEnergyFunction<InputType, std::shared_ptr<Jacobian>>(),
	function_vector(std::move(function_vector))
{
}

template<class InputType>
std::shared_ptr<Eigen::VectorXd> nrLeastSquaresFunction<InputType>::eval_residuals(
	const InputType& input)
{
	int size = function_vector->size();

	residuals = std::make_unique<Eigen::VectorXd>();
	residuals->resize(size);
	residuals->setZero();
	
	for (int i = 0; i < size; i++)
	{
		double res = function_vector->operator[](i)->operator()(input);

		residuals->operator()(i) = res;
	}

	return residuals;
}

template<class InputType>
double nrLeastSquaresFunction<InputType>::operator()(const InputType& input)
{
	std::shared_ptr<Eigen::VectorXd> result = eval_residuals(input);

	double sq_norm = result->squaredNorm();
	last_value_cache = sq_norm;

	return 0.5 * sq_norm;
}

template<class InputType>
std::shared_ptr<Jacobian> nrLeastSquaresFunction<InputType>::df(const InputType& input)
{
	if (function_vector->empty())
	{
		return std::make_unique<Jacobian>();
	}

	std::unique_ptr<JacobianRow> derivative = std::make_unique<JacobianRow>();
	
	jacobian = std::make_shared<Jacobian>(
		function_vector->size(), 
		input.size());
	
	double eps = std::numeric_limits<double>::epsilon();
	double d = 0.0;
	std::vector<Eigen::Triplet<double>> triplets;

	for (int i = 0; i < function_vector->size(); i++)
	{
		derivative = function_vector->operator[](i)->df(input);

		for (JacobianRow::InnerIterator it(*derivative); it; ++it)
		{
			d = it.value();

			if (d > eps || d < -eps)
			{
				triplets.push_back(Eigen::Triplet<double>(i, it.index(), d));
			}
		}
	}

	jacobian->setFromTriplets(triplets.begin(), triplets.end());

	return jacobian;
}

//eval_residuals() and df() need to be called before
template<class InputType>
std::shared_ptr<Eigen::RowVectorXd> 
	nrLeastSquaresFunction<InputType>::gradient()
{
	if (residuals->size() == 0 || jacobian->size() == 0)
	{
		return nullptr;
	}

	last_gradient = 
		std::make_shared<Eigen::RowVectorXd>(residuals->size());
	last_gradient->setZero();

	int row;
	for (int i = 0; i < jacobian->outerSize(); i++)
	{
		for (Jacobian::InnerIterator it(*jacobian, i); it; ++it)
		{
			row = it.row();
			last_gradient->operator()(row) += 
				residuals->operator[](row) * it.value();
		}
	}

	return last_gradient;
}

template<class InputType>
nrLeastSquaresFunction<InputType>::~nrLeastSquaresFunction<InputType>()
{
}

nrERigidOneTerm::nrERigidOneTerm(
	std::shared_ptr<nrDeformationGraph> dg, 
	const double& alpha, 
	const int& index,
	const int& sum_term)
	: nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>(),
	dg(dg),
	alpha(alpha),
	index(index),
	sum_term(sum_term)
{
}

double nrERigidOneTerm::operator()(const Eigen::VectorXd& input)
{
	const int size_var = dg->size_variable();

	Eigen::Vector3d a_1(
		input[size_var*index], 
		input[size_var*index+3], 
		input[size_var*index+6]);

	Eigen::Vector3d a_2(
		input[size_var * index + 1], 
		input[size_var * index + 4], 
		input[size_var * index + 7]);

	Eigen::Vector3d a_3(
		input[size_var * index + 2], 
		input[size_var * index + 5], 
		input[size_var * index + 8]);

	double result = 0.0;

	switch (sum_term)
	{
	case 1: result = a_1.transpose() * a_2; break;
	case 2: result = a_1.transpose() * a_3; break;
	case 3: result = a_2.transpose() * a_3; break;
	case 4: result = 1 - a_1.transpose() * a_1; break;
	case 5: result = 1 - a_2.transpose() * a_2; break;
	case 6: result = 1 - a_3.transpose() * a_3; break;
	}

	result *= std::sqrt(alpha);

	return result;
}

std::unique_ptr<JacobianRow> nrERigidOneTerm::df(const Eigen::VectorXd& input)
{
	const int size_var = dg->size_variable();

	std::unique_ptr<JacobianRow> result = 
		std::make_unique<JacobianRow>();
	result->resize(input.size());
	int vector_index = 0;

	double factor = std::sqrt(alpha);

	switch (sum_term)
	{
	case 1:
	{
		result->coeffRef(size_var*index) = factor * input[size_var*index + 1];
		result->coeffRef(size_var*index + 1) = factor * input[size_var*index];
		result->coeffRef(size_var*index + 3) = factor * input[size_var*index + 4];
		result->coeffRef(size_var*index + 4) = factor * input[size_var*index + 3];
		result->coeffRef(size_var*index + 6) = factor * input[size_var*index + 7];
		result->coeffRef(size_var*index + 7) = factor * input[size_var*index + 6];
		break;
	}
	case 2:
	{
		result->coeffRef(size_var*index) = factor * input[size_var*index + 2];
		result->coeffRef(size_var*index + 2) = factor * input[size_var*index];
		result->coeffRef(size_var*index + 3) = factor * input[size_var*index + 5];
		result->coeffRef(size_var*index + 5) = factor * input[size_var*index + 3];
		result->coeffRef(size_var*index + 6) = factor * input[size_var*index + 8];
		result->coeffRef(size_var*index + 8) = factor * input[size_var*index + 6];
		break;
	}
	case 3:
	{
		result->coeffRef(size_var*index + 1) = factor * input[size_var*index + 2];
		result->coeffRef(size_var*index + 2) = factor * input[size_var*index + 1];
		result->coeffRef(size_var*index + 4) = factor * input[size_var*index + 5];
		result->coeffRef(size_var*index + 5) = factor * input[size_var*index + 4];
		result->coeffRef(size_var*index + 7) = factor * input[size_var*index + 8];
		result->coeffRef(size_var*index + 8) = factor * input[size_var*index + 7];
		break;
	}
	case 4:
	{
		result->coeffRef(size_var*index) = -2.0 * factor * input[size_var*index];
		result->coeffRef(size_var*index + 3) = -2.0 * factor * input[size_var*index + 3];
		result->coeffRef(size_var*index + 6) = -2.0 * factor * input[size_var*index + 6];
		break;
	}
	case 5:
	{
		result->coeffRef(size_var*index + 1) = -2.0 * factor * input[size_var*index + 1];
		result->coeffRef(size_var*index + 4) = -2.0 * factor * input[size_var*index + 4];
		result->coeffRef(size_var*index + 7) = -2.0 * factor * input[size_var*index + 7];
		break;
	}
	case 6:
	{
		result->coeffRef(size_var*index + 2) = -2.0 * factor * input[size_var*index + 2];
		result->coeffRef(size_var*index + 6) = -2.0 * factor * input[size_var*index + 6];
		result->coeffRef(size_var*index + 8) = -2.0 * factor * input[size_var*index + 8];
		break;
	}
	}

	/*
	for (int var = 0;
		var != dg->SIZE_VARIABLE_R;
		++var)
	{
		double partial_derivative = partial(input, size_var*index + var);

		double current = result->coeffRef(size_var*index + var);

		std::cout << current << '\t' << partial_derivative << std::endl;
	}*/
	
	return result;
}

nrERigidOneTerm::~nrERigidOneTerm()
{
}

nrESmoothOneTerm::nrESmoothOneTerm(
	std::shared_ptr<nrDeformationGraph> dg,
	const double& alpha,
	const int& starti,
	const int& i, 
	const int& j,
	const int& r,
	const double& weight)
	: nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>(),
	dg(dg),
	alpha(alpha),
	starti(starti),
	i(i),
	j(j),
	r(r),
	weight(weight)
{
}

double nrESmoothOneTerm::operator()(const Eigen::VectorXd& input)
{
	int size_var = dg->size_variable();

	Eigen::Matrix3d rot;
	Eigen::Vector3d trans_i;
	Eigen::Vector3d trans_j;
	calc_rts(input, rot, trans_i, trans_j);

	Eigen::Vector3d difference;
	difference.setZero();

	double factor = std::sqrt(alpha * weight);

	difference += rot * (dg->node(j) - dg->node(i));

	difference += (dg->node(i) + trans_i);

	difference -= (dg->node(j) + trans_j);

	double result = difference[r];

	result *= factor;

	return result;
}

std::unique_ptr<JacobianRow> nrESmoothOneTerm::df(const Eigen::VectorXd& input)
{
	int size_variable = dg->size_variable();
	std::unique_ptr<JacobianRow> result = std::make_unique<JacobianRow>();
	result->resize(input.size());

	double factor = std::sqrt(weight * alpha);
		
	Eigen::Vector3d nodei = dg->node(i);
	Eigen::Vector3d nodej = dg->node(j);

	for (int c = 0; c < 3; c++)
	{
		double der = nodej[c] - nodei[c];
		der *= factor;

		result->coeffRef(size_variable * (starti + i) + 3 * r + c) = der;
	}

	double der = 1.0 * factor;
	result->coeffRef(size_variable * (starti + i) + dg->SIZE_VARIABLE_R + r) = 
		der;

	der = -1.0 * factor;
	result->coeffRef(size_variable * (starti + j) + dg->SIZE_VARIABLE_R + r) = 
		der;

	/*
	for (int var = 0; var != size_variable; ++var)
	{
		double partial_derivative1 = partial(input, size_variable* (starti + i) + var);
		result->coeffRef(size_variable * (starti + i) + var) = partial_derivative1;

		double partial_derivative2 = partial(input, size_variable* (starti + j) + var);
		result->coeffRef(size_variable * (starti + j) + var) = partial_derivative2;
	}*/

	return result;
}

nrESmoothOneTerm::~nrESmoothOneTerm()
{
}

void nrESmoothOneTerm::calc_rts(
	const Eigen::VectorXd& input,
	Eigen::Matrix3d& rot,
	Eigen::Vector3d& ti,
	Eigen::Vector3d& tj) const
{
	int size_var = dg->size_variable();

	for (int k = 0; k < dg->SIZE_VARIABLE_R; k++)
	{
		rot(k / 3, k % 3) = input[size_var * (starti + i) + k];
	}

	for (int k = dg->SIZE_VARIABLE_R; k < size_var; k++)
	{
		ti(k - dg->SIZE_VARIABLE_R) = input[size_var * (starti + i) + k];
		tj(k - dg->SIZE_VARIABLE_R) = input[size_var * (starti + j) + k];
	}
}

nrECorrOneTerm::nrECorrOneTerm(
	std::shared_ptr<nrDeformationGraph> dg1,
	std::shared_ptr<nrDeformationGraph> dg2,
	const double& alpha,
	const Correspondence& correspondence,
	const int& starti_1,
	const int& starti_2,
	const int& r,
	const int& set_size)
	: nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>(),
	d(dg1->distance_function()),
	alpha(alpha),
	r(r),
	set_size(set_size)
{
	dg.push_back(dg1);
	dg.push_back(dg2);

	start.push_back(starti_1);
	start.push_back(starti_2);

	points.push_back(std::get<1>(correspondence));
	points.push_back(std::get<3>(correspondence));
}

double nrECorrOneTerm::operator()(const Eigen::VectorXd& input)
{
	std::vector<Eigen::Vector3d> t_points;

	for (int i = 0; i < dg.size(); i++)
	{
		dg[i]->set_vector(input, start[i]);
		Eigen::Vector3d t_point = dg[i]->transform(points[i]);
		t_points.push_back(t_point);
	}

	double factor = std::sqrt(alpha / set_size);

	double result = std::abs((t_points[0] - t_points[1])[r]);

	result *= factor;

	return result;
}

std::unique_ptr<JacobianRow> nrECorrOneTerm::df(const Eigen::VectorXd& input)
{
	std::vector<int> size;
	std::vector<int> size_variable;
	std::vector<std::vector<double>> weights;

	for (int i = 0; i < dg.size(); i++)
	{
		size.push_back(dg[i]->size_graph());
		size_variable.push_back(dg[i]->size_variable());

		std::vector<double> weights_i;
		dg[i]->calculate_weights(points[i], weights_i);
		weights.push_back(weights_i);
	}
	
	std::unique_ptr<JacobianRow> result = std::make_unique<JacobianRow>();
	result->resize(input.size());

	std::vector<double> factor(2, std::sqrt(alpha/set_size));
	factor[0] *= 1.0;
	factor[1] *= -1.0;

	for (int i = 0; i < dg.size(); i++)
	{
		for (int j = 0; j < size[i]; j++)
		{
			if (weights[i][j] < std::numeric_limits<double>::epsilon())
			{
				continue;
			}

			for (int c = 0; c < 3; c++)
			{
				double der = points[i][c] - dg[i]->node(j)[c];
				der *= factor[i] * weights[i][j];

				result->coeffRef(
					size_variable[i] * (start[i] + j) + 3 * r + c) = der;
			}

			double der = factor[i];
			result->coeffRef(
				size_variable[i] * 
				(start[i] + j) + dg[i]->SIZE_VARIABLE_R + r) = der;			
		}
	}

	return result;
}

nrECorrOneTerm::~nrECorrOneTerm()
{
}

void dummy()
{
	std::unique_ptr<std::vector<std::unique_ptr<nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>>>> x;
	nrLeastSquaresFunction<Eigen::VectorXd> dummy_function(std::move(x));
}

}
