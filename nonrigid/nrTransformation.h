#ifndef NRTRANSFORMATION_H
#define NRTRANSFORMATION_H

#include "nrDistanceFunction.h"
#include "nrPointCloud.h"

#include<Eigen/Dense>

#include<vector>
#include<memory>

namespace nonrigid
{

class nrDeformationGraph
{
	/*
	Transforms a point cloud using a deformation graph. Check:

	Hao Li, Bart Adams, Leonidas J. Guibas, Mark Pauly
	Robust Single-View Geometry and Motion Reconstruction
	SIGGRAPH Asia 2009

	Robert W. Sunmer, Johannes Schmid, Mark Pauly
	Embedded Deformation for Shape Manipulation
	ACM SIGGRAPH 2007
	*/

public:
	nrDeformationGraph();
	
	virtual void add(
		Eigen::Vector3d node, 
		Eigen::Matrix3d rotation, 
		Eigen::Vector3d translation, 
		double radius);
	virtual void init_vector();

	virtual void set_vector(const Eigen::VectorXd&, const int& start_index);
	virtual Eigen::VectorXd get_vector();
	
	virtual nrDistanceFunction& distance_function();
	
	virtual nrPointCloud operator*(const nrPointCloud&) const;

	virtual int size_graph() const;
	virtual int size_variable() const;

	virtual bool incident(const int& i, const int& j) const;
	
	bool calculate_weights(
		const Eigen::Vector3d& point,
		std::vector<double>& weights,
		const double& radius = 0.0) const;

	virtual const Eigen::Vector3d& node(const int& i) const;
	virtual const Eigen::Matrix3d& rotation(const int& i) const;
	virtual const Eigen::Vector3d& translation(const int& i) const;
	virtual const double radius(const int& i) const;

	virtual Eigen::Vector3d transform(
		const Eigen::Vector3d&) const;

	virtual Eigen::Vector3d transform_normal(
		const Eigen::Vector3d& vertex, 
		const Eigen::Vector3d& normal) const;

	static double weight(
		const Eigen::Vector3d&,
		const Eigen::Vector3d&,
		const double& radius);

	const int SIZE_VARIABLE_R;
	const int SIZE_VARIABLE_T;

private:
	nrDistanceFunction d;

	std::vector<Eigen::Vector3d> nodes;
	std::vector<double> radii;

	Eigen::VectorXd optimization_vector;
	std::vector<Eigen::Matrix3d> rotations;
	std::vector<Eigen::Vector3d> translations;

	std::vector<std::vector<bool>> incidence;

};

}

#endif

