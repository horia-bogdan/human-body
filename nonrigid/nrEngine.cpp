#include "nrEngine.h"

#include "nrPointCloud.h"
#include "nrOptimization.h"
#include "nrTransformation.h"

#include <fstream>

namespace nonrigid

{

nrEngine::nrEngine(
	const double& leaf_size, 
	const double& radius, 
	const double& alpha_rigid, 
	const double& alpha_smooth, 
	const double& alpha_corr, 
	std::vector<Correspondence>& corrs)
	:node_density(leaf_size),
	radius(radius),
	alpha_rigid(alpha_rigid/*500.0*/),
	alpha_smooth(alpha_smooth/*2.0*/),
	alpha_corr(alpha_corr/*2.5*/),
	correspondences(corrs)
{
}

void nrEngine::add(const nrPointCloud& cloud,
	const Eigen::Matrix3d& init_rigid_r,
	const Eigen::Vector3d& init_rigid_t)
{
	data.push_back(cloud);

	init_r.push_back(init_rigid_r);
	init_t.push_back(init_rigid_t);
}

void nrEngine::apply(nrOptimizationType optimization)
{
	downsample_all();

	set_up_transformation();
	set_up_objective();

	int size = 0;
	for (int i = 0; i < deformation_graph.size(); i++)
	{
		size += 
			deformation_graph[i]->size_graph() * 
				deformation_graph[i]->size_variable();
	}
	Eigen::VectorXd variable(size);
	variable.setZero();
	int vector_index = 0;
	for (int i = 0; i < deformation_graph.size(); i++)
	{
		Eigen::VectorXd graph_vector = 
			deformation_graph[i]->get_vector();

		for (int j = 0; j < graph_vector.size(); j++)
		{
			variable(vector_index) = graph_vector(j);

			++vector_index;
		}
	}

	Eigen::VectorXd result;
	switch (optimization)
	{
	case GAUSS_NEWTON:
	{
		nrGaussNewtonOptimization gn(objective, variable);
		gn.run();
		result = gn.argmin();
		break;
	}
	case LBFGS:
	{
		nrLBFGSOptimization::objective = objective;
		nrLBFGSOptimization::current_point = variable;
		Result r = nrLBFGSOptimization::run();
		result = nrLBFGSOptimization::argmin();
		break;
	}
	}

	for (int i = 0; i < deformation_graph.size(); i++)
	{
		int size_variable = deformation_graph[i]->size_variable();

		deformation_graph[i]->set_vector(
			result,
			size_variable * start_index[i]);
	}

	transform_all();
}

nrPointCloud nrEngine::get_transformed(const int& i)
{
	return transformed[i];
}

void nrEngine::transform_from_file(const std::string& file)
{
	downsample_all();

	set_up_transformation();

	std::vector<double> read_vector;
	std::ifstream f(file);

	double d;
	while (f >> d)
	{
		read_vector.push_back(d);
	}

	Eigen::VectorXd transform(read_vector.size());
	for (int i = 0; i < read_vector.size(); i++)
	{
		transform(i) = read_vector[i];
	}

	for (int i = 0; i < deformation_graph.size(); i++)
	{
		int size_variable = deformation_graph[i]->size_variable();

		deformation_graph[i]->set_vector(
			transform, 
			size_variable * start_index[i]);
	}

	transform_all();
}

nrEngine::~nrEngine()
{

}

void nrEngine::downsample_all()
{
	for (std::vector<nrPointCloud>::iterator it = data.begin();
		it != data.end();
		++it)
	{
		nrPointCloud pc = it->downsample(node_density);
		graph_nodes.push_back(pc);
	}
}

typedef nrEnergyFunction<Eigen::VectorXd, std::unique_ptr<JacobianRow>> Energy;

void nrEngine::set_up_objective()
{
	std::unique_ptr<std::vector<std::unique_ptr<Energy>>> function_vector = 
		std::make_unique<std::vector<std::unique_ptr<Energy>>>();

	//E Corr
	for (int i = 0; i < correspondences.size(); i++)
	{
		int i1 = std::get<0>(correspondences[i]);
		int i2 = std::get<2>(correspondences[i]);

		for (int r = 0; r < 3; r++)
		{
			std::unique_ptr<nrECorrOneTerm> e_corr =
				std::make_unique<nrECorrOneTerm>(
					deformation_graph[i1],
					deformation_graph[i2],
					alpha_corr,
					correspondences[i],
					start_index[i1],
					start_index[i2],
					r,
					correspondences.size());

			function_vector->push_back(std::move(e_corr));
		}
	}

	//E Rigid

	for (int i = 0; i < deformation_graph.size(); i++)
	{
		int size_graph = deformation_graph[i]->size_graph();
		int size_var = deformation_graph[i]->size_variable();

		for (int j = 0; j < size_graph; j++)
		{
			for (int k = 1; k <= 6; k++)
			{
				std::unique_ptr<nrERigidOneTerm> e_rigid =
					std::make_unique<nrERigidOneTerm>(
						deformation_graph[i],
						alpha_rigid,
						start_index[i] + j,
						k);

				function_vector->push_back(std::move(e_rigid));
			}

		}
	}

	//E Smooth
	for (int i = 0; i < deformation_graph.size(); i++)
	{
		int size_graph = deformation_graph[i]->size_graph();
		int size_var = deformation_graph[i]->size_variable();

		for (int j = 0; j < size_graph; j++)
		{
			for (int k = 0; k < size_graph; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					if (deformation_graph[i]->incident(j, k))
					{
						std::unique_ptr<nrESmoothOneTerm> e_smooth =
							std::make_unique<nrESmoothOneTerm>(
								deformation_graph[i],
								alpha_smooth,
								start_index[i],
								j,
								k,
								l,
								1.0);

						function_vector->push_back(std::move(e_smooth));
					}
				}
			}		
		}
	}


	objective = std::make_shared<nrLeastSquaresFunction<Eigen::VectorXd>>(
		std::move(function_vector));
}

void nrEngine::set_up_transformation()
{
	double w_eps = 0.01;

	int size = graph_nodes.size();
	deformation_graph.resize(size);

	std::vector<std::vector<Eigen::Vector3d>> relevant_points;
	relevant_points.resize(size);

	for (int i = 0; i < correspondences.size(); i++)
	{
		relevant_points[std::get<0>(correspondences[i])].push_back(
			std::get<1>(correspondences[i]));

		relevant_points[std::get<2>(correspondences[i])].push_back(
			std::get<3>(correspondences[i]));
	}


	for (int i = 0; i < size; i++)
	{
		deformation_graph[i] = std::make_shared<nrDeformationGraph>();
		
		if (i > 0)
		{
			start_index.push_back(
				start_index[i - 1] + deformation_graph[i - 1]->size_graph());
		}
		else {
			start_index.push_back(0);
		}

		for (int j = 0; j < graph_nodes[i].size(); j++)
		{
			Eigen::Vector3d node = graph_nodes[i](j);

			/*
			bool node_is_relevant = false;

			for (int k = 0; k < relevant_points[i].size(); k++)
			{
				nrDistanceFunction& d = 
					deformation_graph[i]->distance_function();

				double distance = d(node, relevant_points[i][k]);
				double w =
					nrDeformationGraph::weight(relevant_points[i][k], node, radius);

				if (distance < radius && w > w_eps)
				{
					node_is_relevant = true;
					break;
				}
			}

			if (node_is_relevant)
			{*/
				deformation_graph[i]->add(
					node,
					init_r[i],
					//Eigen::Matrix3d::Identity(),
					//Eigen::Matrix3d::Random(),
					init_t[i],
					//Eigen::Vector3d::Zero(),
					//Eigen::Vector3d::Random(),
					radius);
			//}
		}
		deformation_graph[i]->init_vector();
	}
}

void nrEngine::transform_all()
{
	transformed.clear();
	for (int i = 0; i < data.size(); i++)
	{
		nrPointCloud t_cloud = deformation_graph[i]->operator*(data[i]);
		transformed.push_back(t_cloud);
	}
}

}