#ifndef NRENGINE_H
#define NRENGINE_H

#include "nrEnergyFunction.h"
#include "nrTransformation.h"

#include<memory>
#include<vector>

namespace nonrigid 
{

class nrPointCloud;

enum nrOptimizationType
{
	GAUSS_NEWTON,
	LBFGS
};


class nrEngine
{
public:
	nrEngine(
		const double& leaf_size,   //how dense is the deformation graph
		const double& radius,      
		const double& alpha_rigid,
		const double& alpha_smoth,
		const double& alpha_corr,
		std::vector<Correspondence>& corrs);

	virtual void add(
		const nrPointCloud&, 
		const Eigen::Matrix3d& init_rigid_r, 
		const Eigen::Vector3d& init_rigid_t);

	virtual void apply(nrOptimizationType = GAUSS_NEWTON);
	
	virtual nrPointCloud get_transformed(const int& i);
	
	virtual void transform_from_file(const std::string& file);

	virtual ~nrEngine();

private:
	void downsample_all();	
	void set_up_objective();
	void set_up_transformation();
	void transform_all();

	double node_density;
	double alpha_rigid, alpha_smooth, alpha_corr;
	double radius;

	std::vector<Correspondence>& correspondences;

	std::vector<std::shared_ptr<nrDeformationGraph>> deformation_graph;
	std::vector<int> start_index;

	std::shared_ptr<nrLeastSquaresFunction<Eigen::VectorXd>> objective;	
	
	std::vector<nrPointCloud> graph_nodes;
	std::vector<Eigen::Matrix3d> init_r;
	std::vector<Eigen::Vector3d> init_t;

	std::vector<nrPointCloud> data;
	std::vector<nrPointCloud> transformed;
};

}

#endif

