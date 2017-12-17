#ifndef RALGINMENTENGINE_H
#define RALIGNMENTENGINE_H

#include "rTransformationMatrix.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace rec3D 
{

class rAlignmentEngine
{
public:
	typedef std::tuple<int, Eigen::Vector3d, int, Eigen::Vector3d> Correspondence;

	rAlignmentEngine(const float& tol);

	virtual void add_point_cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr);

	virtual bool align_all_icp();
	
	bool empty() const;

	virtual pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr get_cloud(
		const int& i) const;

	virtual void nonrigid_all();

	virtual ~rAlignmentEngine();

	const float tolerance;
	const int OVERLAP_THRESHOLD = 500;

private:
	void build_overlap_matrix();

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr closest_point_overlap(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr pc1, 
		pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr pc2);
	
	void downsample_all();
	
	void downsample(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr input, 
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& container);

	bool find_order_indices(
		std::vector<std::vector<int>>& order) const;

	double icp(
		const int& pc_i1,
		const std::vector<int>& pc_i2,
		Eigen::Matrix<float, 4, 4>& transformation);
	
	//void nonrigid_tps(const int& source_i, const std::vector<int>& target_i);

	void overlap(
		const int& pc_i1,
		const int& pc_i2,
		const float& tolerance);

	void write_nonrigid_config(
		std::string config_file,
		std::string source_file,
		std::string target_file,
		std::string ctrl_points,
		std::string init_affine,
		std::string init_tps,
		std::string init_params,
		std::string transformed_model,
		std::string final_rigid,
		std::string final_affine,
		std::string final_tps,
		std::string final_params) const;

	void write_point_cloud_txt(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud, 
		std::string path) const;

	void calculate_corrs();

	std::vector<Correspondence> corrs;

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> downsampled;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>>  
		downsampled_intersections;

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> patches;

	std::vector<std::vector<int>> overlap_matrix;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> 
		intersections;

	std::vector<Eigen::Matrix<float, 4, 4>> rigid_transform;
};

}

namespace gmmreg 
{
	int ThinPlateSplineTransform(const char* f_config);
}


#endif