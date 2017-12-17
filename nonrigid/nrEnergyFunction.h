#ifndef NRENERGYFUNCTION_H
#define NRENERGYFUNCTION_H

#include "nrDistanceFunction.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <memory>
#include <vector>

namespace nonrigid
{

//energy function base class
template<class InputType, class DerivativeType>
class nrEnergyFunction
{
public:
	nrEnergyFunction();

	virtual double operator()(const InputType&) = 0;
	
	virtual DerivativeType df(const InputType&) = 0;

	virtual double last_eval_value() const;

	virtual ~nrEnergyFunction();

protected:
	double last_value_cache;
};

class nrDeformationGraph;
typedef Eigen::SparseMatrix<double> Jacobian;
typedef Eigen::SparseVector<double> JacobianRow;

template<class InputType>
class nrLeastSquaresFunction
	: public nrEnergyFunction<InputType, std::shared_ptr<Jacobian>>
{
public:
	nrLeastSquaresFunction(
		std::unique_ptr<std::vector<std::unique_ptr<nrEnergyFunction<InputType, std::unique_ptr<JacobianRow>>>>>);

	virtual std::shared_ptr<Eigen::VectorXd> eval_residuals(const InputType&);

	virtual double operator()(const InputType&);

	virtual std::shared_ptr<Jacobian> df(const InputType&);

	//to be called only after calling eval_residuals() and df() on the same input
	virtual std::shared_ptr<Eigen::RowVectorXd> gradient();

	virtual ~nrLeastSquaresFunction();

private:
	std::unique_ptr<std::vector<std::unique_ptr<nrEnergyFunction<InputType, std::unique_ptr<JacobianRow>>>>>
		function_vector;

	std::shared_ptr<Eigen::VectorXd> residuals;
	std::shared_ptr<Jacobian> jacobian;
	std::shared_ptr<Eigen::RowVectorXd> last_gradient;
};

class nrERigidOneTerm
	: public nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>
{
public:
	/*
	sum_term == 1 : a_1^T a_2
	sum_term == 2 : a_1^T a_3
	sum_term == 3 : a_2^T a_3
	sum_term == 4 : 1 - a_1^T a_1
	sum_term == 5 : 1 - a_2^T a_2
	sum_term == 6 : 1 - a_3^T a_3
	*/
	nrERigidOneTerm(
		std::shared_ptr<nrDeformationGraph> dg, 
		const double& alpha, 
		const int& index,
		const int& sum_term);
	

	virtual double operator()(const Eigen::VectorXd&);

	virtual std::unique_ptr<JacobianRow> df(const Eigen::VectorXd&);

	virtual ~nrERigidOneTerm();

private:
	std::shared_ptr<nrDeformationGraph> dg;

	int index;
	int sum_term;
	double alpha;
};

class nrESmoothOneTerm
	: public nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>
{
public:
	nrESmoothOneTerm(
		std::shared_ptr<nrDeformationGraph> dg,
		const double& alpha, 
		const int& starti, //vector start index for R,t of relevant graph
		const int& i, 
		const int& j,
		const int& r,
		const double& weight);

	virtual double operator()(const Eigen::VectorXd&);

	virtual std::unique_ptr<JacobianRow> df(const Eigen::VectorXd&);

	virtual ~nrESmoothOneTerm();

private:
	virtual void calc_rts(
		const Eigen::VectorXd& input, 
		Eigen::Matrix3d& rot, 
		Eigen::Vector3d& ti, 
		Eigen::Vector3d& tj) const;

	std::shared_ptr<nrDeformationGraph> dg;

	double alpha, weight;
	int starti, i, j, r;
};

typedef std::tuple<int, Eigen::Vector3d, int, Eigen::Vector3d> Correspondence;
class nrECorrOneTerm
	: public nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>>
{
public:
	nrECorrOneTerm(
		std::shared_ptr<nrDeformationGraph> dg1,
		std::shared_ptr<nrDeformationGraph> dg2,
		const double& alpha, 
		const Correspondence& correspondence,
		const int& starti_1, //vector start indices for R,t of relevant graphs
		const int& starti_2, // ^
		const int& r,
		const int& set_size);

	virtual double operator()(const Eigen::VectorXd&);

	virtual std::unique_ptr<JacobianRow> df(const Eigen::VectorXd&);

	virtual ~nrECorrOneTerm();

private:
	std::vector<int> start;	
	std::vector<std::shared_ptr<nrDeformationGraph>> dg;
	nrDistanceFunction d;

	double alpha;
	std::vector<Eigen::Vector3d> points;
	int set_size, r;
};

}

#endif