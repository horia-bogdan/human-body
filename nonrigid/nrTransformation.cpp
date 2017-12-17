#include "nrTransformation.h"

#include<cmath>

namespace nonrigid
{

nrDeformationGraph::nrDeformationGraph()
	:SIZE_VARIABLE_R(9),
	SIZE_VARIABLE_T(3)
{
}

void nrDeformationGraph::add(
	Eigen::Vector3d node,
	Eigen::Matrix3d rotation,
	Eigen::Vector3d translation,
	double radius)
{
	nodes.push_back(node);
	rotations.push_back(rotation);
	translations.push_back(translation);
	radii.push_back(radius);
	
	int size = incidence.size();
	incidence.push_back(std::vector<bool>(size + 1));

	for (int i = 0; i < size; i++)
	{
		double distance = d(node, nodes[i]);
		
		incidence[i].resize(size+1);

		incidence[i][size] = distance < radii[i];
		incidence[size][i] = distance < radius;		
	}

	incidence[size][size] = false;
}

void nrDeformationGraph::init_vector()
{
	int size = size_graph();
	int size_var = SIZE_VARIABLE_R + SIZE_VARIABLE_T;

	optimization_vector.resize(size_var * size);

	for (int j = 0; j < size; j++)
	{
		int current_size = size_var * j;

		for (int i = 0; i < SIZE_VARIABLE_R; i++)
		{
			optimization_vector(current_size + i) = rotations[j](i / 3, i % 3);
		}
		for (int i = SIZE_VARIABLE_R; 
			i < SIZE_VARIABLE_R + SIZE_VARIABLE_T; i++)
		{
			optimization_vector(current_size + i) = translations[j](i - 9);
		}
	}
}

void nrDeformationGraph::set_vector(
	const Eigen::VectorXd& vec, 
	const int& start_index)
{
	optimization_vector = vec;

	int size_var = SIZE_VARIABLE_R + SIZE_VARIABLE_T;
	for (int i = 0; i < nodes.size(); i++)
	{
		for (int j = 0; j < SIZE_VARIABLE_R; j++)
		{
			rotations[i](j / 3, j % 3) = vec(start_index + size_var*i + j);
		}
		for (int j = 0; j < SIZE_VARIABLE_T; j++)
		{
			translations[i](j) = 
				vec(start_index + size_var*i + j + SIZE_VARIABLE_R);
		}
	}
}

Eigen::VectorXd nrDeformationGraph::get_vector() 
{
	return optimization_vector;
}

nrDistanceFunction& nrDeformationGraph::distance_function()
{
	return d;
}

nrPointCloud nrDeformationGraph::operator*(const nrPointCloud& point_cloud) const
{
	nrPointCloud transformed_cloud;

	for (int i = 0; i < point_cloud.size(); i++)
	{
		Eigen::Vector3d point = point_cloud(i);
		Eigen::Vector3d transformed_point = this->transform(point);

		Eigen::Vector3d normal = point_cloud.normal(i);
		Eigen::Vector3d transformed_normal = this->transform_normal(point, normal);

		transformed_cloud.append(transformed_point, point_cloud.rgb(i), transformed_normal);
	}

	return transformed_cloud;
}

int nrDeformationGraph::size_graph() const
{
	return nodes.size();
}

int nrDeformationGraph::size_variable() const
{
	return SIZE_VARIABLE_R + SIZE_VARIABLE_T;
}

bool nrDeformationGraph::incident(const int& i, const int& j) const
{
	if (i < 0 || i >= incidence.size() || j < 0 || j >= incidence.size())
	{
		return false;
	}

	return incidence[i][j];
}

bool nrDeformationGraph::calculate_weights(
	const Eigen::Vector3d& point,
	std::vector<double>& weights,
	const double& radius) const
{
	weights.resize(size_graph());

	double sum = 0.0;
	double r = radius;
	for (int i = 0; i < weights.size(); i++)
	{
		if (r < std::numeric_limits<double>::epsilon())
		{
			r = radii[i];
		}

		weights[i] = nrDeformationGraph::weight(point, node(i), r);
		sum += weights[i];
	}

	if (sum < std::numeric_limits<double>::epsilon())
	{
		return false;
	}

	for (int i = 0; i < weights.size(); i++)
	{
		weights[i] /= sum;
	}

	return true;
}

const Eigen::Vector3d& nrDeformationGraph::node(const int& i) const
{
	return nodes[i];
}

const Eigen::Matrix3d& nrDeformationGraph::rotation(const int& i) const
{
	return rotations[i];
}

const Eigen::Vector3d& nrDeformationGraph::translation(const int& i) const
{
	return translations[i];
}

const double nrDeformationGraph::radius(const int& i) const
{
	return radii[i];
}

Eigen::Vector3d nrDeformationGraph::transform(
	const Eigen::Vector3d& point) const
{
	Eigen::Vector3d transformed_point;
	transformed_point.setZero();

	std::vector<double> weights;
	bool relevant = calculate_weights(point, weights);

	if (!relevant)
	{
		return point;
	}

	for (int i = 0; i < size_graph(); i++)
	{
		if (d(point, nodes[i]) > radii[i])
		{
			continue;
		}

		double w = weights[i];

		if (std::abs(w) < std::numeric_limits<double>::epsilon())
		{
			continue;
		}

		Eigen::Vector3d rotated =
			rotation(i) * (point - node(i));

		Eigen::Vector3d translated = node(i) + translation(i);

		transformed_point += w*(rotated + translated);
	}

	return transformed_point;
}

Eigen::Vector3d nrDeformationGraph::transform_normal(
	const Eigen::Vector3d& vertex,
	const Eigen::Vector3d& normal) const
{
	std::vector<double> weights;
	bool relevant = calculate_weights(vertex, weights);

	if (!relevant)
	{
		return normal;
	}

	Eigen::Vector3d transformed_normal;
	transformed_normal.setZero();

	for (int i = 0; i < size_graph(); i++)
	{
		double w = weights[i];

		if (std::abs(w) < std::numeric_limits<double>::epsilon())
		{
			continue;
		}

		transformed_normal +=
			rotation(i).inverse().transpose() * normal;
	}

	return transformed_normal;
}

double nrDeformationGraph::weight(
	const Eigen::Vector3d& v,
	const Eigen::Vector3d& x,
	const double& radius)
{
	nrDistanceFunction d;

	double dist = d(v, x);

	double value = std::pow(1.0 - std::pow(dist, 2) / std::pow(radius, 2), 3);
	double positive = std::max(0.0, value);

	return positive;
}



}
