#include "rAlignmentEngine.h"

#include "../nonrigid/nrEngine.h"
#include "../nonrigid/nrPointCloud.h"

#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

//#include <gmmreg_api.h>

#include <algorithm>
#include <fstream>
#include <queue>

namespace rec3D
{

rAlignmentEngine::rAlignmentEngine(const float& tol)
	:tolerance(tol)
{
}

void rAlignmentEngine::add_point_cloud(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud)
{
	patches.push_back(point_cloud);

	rigid_transform.push_back(Eigen::Matrix<float, 4, 4>::Identity());
}

bool rAlignmentEngine::align_all_icp()
{
	int size = patches.size();

	if (overlap_matrix.size() != size)
	{
		build_overlap_matrix();
	}

	std::vector<std::vector<int>> order;
	bool result = false;

	result = find_order_indices(order);
	
	if (!result)
	{
		return false;
	}

	/*
	for (int i = order.size()-1; i >= 0; i--)
	{
		for (int j = 1; j < order[i].size(); j++)
		{
			Eigen::Matrix<float, 4, 4> transformation;
			std::vector<int> o;
			o.push_back(order[i][0]);

			icp(order[i][j], o, transformation);

			if (transformation == Eigen::Matrix<float, 4, 4>::Identity())
			{
				return false;
			}

			for (int k = i + 1; k < order.size(); k++)
			{
				for (int l = 1; l < order[k].size(); l++)
				{
					pcl::transformPointCloudWithNormals(
						*patches[order[k][l]], 
						*patches[order[k][l]], 
						transformation);
				}
			}
		}
	}*/

	for (int i = order.size() - 1; i >= 0; i--)
	{
		for (int j = 1; j < order[i].size(); j++)
		{
			Eigen::Matrix<float, 4, 4> transformation;
			std::vector<int> align_to;

			for (int k = 0; k < patches.size(); k++)
			{				
				if (overlap_matrix[order[i][j]][k] > OVERLAP_THRESHOLD)
				{
					align_to.push_back(k);
				}
			}

			if (align_to.empty())
			{
				return false;
			}

			icp(order[i][j], align_to, transformation);

			if (transformation ==
				Eigen::Matrix<float, 4, 4>::Identity())
			{
				return false;
			}

			for (int k = i + 1; k < order.size(); k++)
			{
				for (int l = 1; l < order[k].size(); l++)
				{
					pcl::transformPointCloudWithNormals(
						*patches[order[k][l]],
						*patches[order[k][l]],
						transformation);

					rigid_transform[order[k][l]] *= transformation;
				}
			}
		}
	}

	for (int i = 0; i < rigid_transform.size(); i++)
	{
		std::ofstream f(std::string("rigid") + std::to_string(i));
		f << rigid_transform[i];
	}

	return true;
}

bool rAlignmentEngine::empty() const
{
	return patches.empty();
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rAlignmentEngine::get_cloud(
	const int& i) const
{
	return patches[i];
}

void rAlignmentEngine::nonrigid_all()
{
	if (downsampled.empty())
	{
		for (int i = 0; i < patches.size(); i++)
		{
			downsample(patches[i], downsampled);
		}
	}

	calculate_corrs();

	std::vector<std::unique_ptr<std::ofstream>> files;
	for (int i = 0; i < patches.size(); i++)
	{
		std::string filename = std::string("corrs" + std::to_string(i));
		std::unique_ptr<std::ofstream> f = std::make_unique<std::ofstream>(filename);
		files.push_back(std::move(f));
	}

	for (int i = 0; i < corrs.size(); i++)
	{
		int f1 = std::get<0>(corrs[i]);
		int f2 = std::get<2>(corrs[i]);

		Eigen::Vector3d v1 = std::get<1>(corrs[i]);
		Eigen::Vector3d v2 = std::get<3>(corrs[i]);

		*files[f1] << v1[0] << " " << v1[1] << " " << v1[2] << std::endl;
		*files[f2] << v2[0] << " " << v2[1] << " " << v2[2] << std::endl;
	}

	nonrigid::nrEngine engine(0.05, 0.1, 500.0, 2.0, 100.0, corrs);
	
	for (int i = 0; i < patches.size(); i++)
	{
		std::ifstream f(std::string("rigid") + std::to_string(i));
		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				f >> rigid_transform[i](j,k);
			}
		}

		std::unique_ptr<nonrigid::nrPointCloud> pc = 
			std::make_unique<nonrigid::nrPointCloud>(patches[i]);

		Eigen::Matrix3d rigid_r;
		Eigen::Vector3d rigid_t;

		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				rigid_r(j, k) = rigid_transform[i](j, k);
			}

			rigid_t(j) = rigid_transform[i](j, 3);
		}

		engine.add(*pc, rigid_r, rigid_t);
	}

	engine.apply(nonrigid::LBFGS);
	//engine.transform_from_file("x12");

	for (int i = 0; i < patches.size(); i++)
	{
		patches[i] = engine.get_transformed(i).pointer();
	}
}

rAlignmentEngine::~rAlignmentEngine()
{
}

double rAlignmentEngine::icp(
	const int& source_i, 
	const std::vector<int>& targets_i, 
	Eigen::Matrix<float, 4, 4>& transformation)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_point_cloud(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>
		icp;

	icp.setInputSource(patches[source_i]);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	for (int i = 0; i < targets_i.size(); i++)
	{
		*target += *patches[targets_i[i]];
	}
	
	icp.setInputTarget(target);


	icp.setMaxCorrespondenceDistance(tolerance);

	/*icp.setMaximumIterations(1);

	std::ofstream f(std::string("../errors") + std::to_string(n));

	int iterations = 0;
	for (int iterations = 0; iterations < 100; iterations++)
	{
	icp.align(*new_point_cloud);

	f << iterations << " " << icp.getFitnessScore() << std::endl;
	}

	n++;*/

	icp.align(*final_point_cloud);

	if (icp.hasConverged())
	{
		transformation = icp.getFinalTransformation();

		pcl::transformPointCloudWithNormals(
			*patches[source_i],
			*patches[source_i],
			transformation);

		rigid_transform[source_i] *= transformation;

		return icp.getFitnessScore();
	}
	
	return std::numeric_limits<double>::max();
}

void rAlignmentEngine::build_overlap_matrix()
{
	int size = patches.size();
	overlap_matrix.resize(size);
	intersections.resize(size);
	float tolerance = 0.1;

	for (int i = 0; i < size; i++)
	{
		overlap_matrix[i].resize(size, false);
		intersections[i].resize(size, nullptr);
	}

	for (int i = 0; i < size; i++)
	{
		overlap_matrix[i][i] = 0; 

		for (int j = i + 1; j < size; j++)
		{
			overlap(i, j, tolerance);
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rAlignmentEngine::closest_point_overlap(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr pc1,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr pc2)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	pcl::search::KdTree<pcl::PointXYZRGBNormal> tree1, tree2;
	tree1.setInputCloud(pc1);
	tree2.setInputCloud(pc2);

	std::vector<int> indices1, indices2;
	std::vector<float> sq_distance1, sq_distance2;
	for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator it = 
			pc1->begin(); 
		it != pc1->end(); 
		++it)
	{
		indices1.clear();
		indices1.resize(1);
		sq_distance1.clear();
		sq_distance1.resize(1);
		indices2.clear();
		indices2.resize(1);
		sq_distance2.clear();
		sq_distance2.resize(1);

		tree2.nearestKSearch(*it, 1, indices1, sq_distance1);
		tree1.nearestKSearch(
			pc2->operator[](indices1[0]), 
			1, 
			indices2, 
			sq_distance2);

		float distance = pcl::geometry::distance(
			pc2->operator[](indices1[0]), 
			pc1->operator[](indices2[0]));

		if (distance <= tolerance && std::sqrt(sq_distance1[0]) < tolerance)
		{
			point_cloud->insert(point_cloud->end(), *it);
		}
	}

	return point_cloud;

}

void rAlignmentEngine::downsample(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr input,
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& container)
{
	if (input == nullptr)
	{
		container.push_back(nullptr);
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_grid;
	voxel_grid.setInputCloud(input);
	voxel_grid.setLeafSize(0.03, 0.03, 0.03);
	voxel_grid.filter(*point_cloud);

	container.push_back(point_cloud);
	/*
	std::fstream f(
		std::string("downsampled") + std::to_string(index) + std::string(".txt"), 
		std::ios::out);
	for (int i = 0; i < downsampled[index]->size(); i++)
	{
		f << point_cloud->operator[](i).x << " ";
		f << point_cloud->operator[](i).y << " ";
		f << point_cloud->operator[](i).z << std::endl;
	}*/
}

void rAlignmentEngine::downsample_all()
{
	int size = patches.size();

	if (overlap_matrix.size() != size)
	{
		build_overlap_matrix();
	}

	for (int i = 0; i < size; i++)
	{
		downsample(patches[i], downsampled);
		//control_points(i);
	}

	downsampled_intersections.resize(size);
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			downsample(intersections[i][j], downsampled_intersections[i]);
		}
	}
}

bool rAlignmentEngine::find_order_indices(
	std::vector<std::vector<int>>& order) const
{
	order.clear();
	std::vector<int> reached;
	std::queue<int> to_traverse;
	int size = patches.size();
	int cur_node = -1;

	int max_node = -1;
	int max_value = 0;
	for (int i = 0; i < size; i++)
	{
		int c = 0;
		int value = 0;
		for (int j = 0; j < size; j++)
		{
			if (overlap_matrix[i][j] > OVERLAP_THRESHOLD)
			{
				c++;
			}
			
			value += overlap_matrix[i][j];
		}
		if (c > max_node)
		{
			max_node = i;
			max_value = value;
		}
		else if (c == max_node && value > max_value)
		{
			max_node = i;
			max_value = value;
		}
	}
		
	if (max_node != -1)
	{
		cur_node = max_node;
	}
	else
	{
		return false;
	}
		
	reached.push_back(cur_node);
	to_traverse.push(cur_node);

	while (reached.size() != size)
	{
		if (to_traverse.empty())
		{
			return false;
		}

		int current = to_traverse.front();
		to_traverse.pop();

	
		std::vector<int> new_reached;

		for (int j = 0; j < size; j++)
		{
			if (overlap_matrix[current][j] > OVERLAP_THRESHOLD)
			{
				if (std::find(reached.begin(), reached.end(), j) 
					== reached.end())
				{
					new_reached.push_back(j);
				}
			}
		}

		if (new_reached.size() >= 1)
		{
			std::vector<int> new_order;
			new_order.push_back(current);

			for (int i = 0; i < new_reached.size(); i++)
			{
				new_order.push_back(new_reached[i]);
				to_traverse.push(i);
			}

			order.push_back(new_order);

			reached.insert(
				reached.end(), 
				new_reached.begin(), 
				new_reached.end());
		}
	}

	return true;
}

/*
void rAlignmentEngine::nonrigid_tps(
	const int& source_i, 
	const std::vector<int>& target_i)
{
	const std::string source_file("source.txt"), target_file("target.txt");
	write_point_cloud_txt(downsampled[source_i], source_file);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for (int i = 0; i < target_i.size(); i++)
	{
		*target += *downsampled[target_i[i]];
	}

	write_point_cloud_txt(target, target_file);
	
	const std::string config_file("config.ini");
	const std::string affine("result_affine.txt");
	const std::string tps("result_tps.txt");

	write_nonrigid_config(
		config_file, 
		source_file, 
		target_file, 
		"", 
		"", 
		"",
		"",
		"",
		"",
		affine, 
		tps,
		"");

	gmmreg_api(config_file.c_str(), "TPS_L2");

	const std::string transform_config("transform_config.ini");
	const std::string real_source("real_source.txt");
	const std::string transformed("transformed.txt");

	write_point_cloud_txt(patches[source_i], real_source);

	write_nonrigid_config(
		transform_config, 
		real_source, 
		"",
		source_file, 
		affine, 
		tps,
		"",
		transformed, 
		"",
		"", 
		"",
		"");

	gmmreg::ThinPlateSplineTransform(transform_config.c_str());
}*/

void rAlignmentEngine::overlap(
	const int& pc_i1,
	const int& pc_i2,
	const float& tolerance)
{
	pcl::PointXYZRGBNormal new_min_pt, new_max_pt;
	pcl::PointXYZRGBNormal base_min_pt, base_max_pt;

	pcl::getMinMax3D<pcl::PointXYZRGBNormal>(*patches[pc_i1], new_min_pt, new_max_pt);
	pcl::getMinMax3D<pcl::PointXYZRGBNormal>(*patches[pc_i2], base_min_pt, base_max_pt);

	float max_x, max_y, max_z;
	float min_x, min_y, min_z;

	max_x = std::min<float>(base_max_pt.x, new_max_pt.x) + tolerance;
	max_y = std::min<float>(base_max_pt.y, new_max_pt.y) + tolerance;
	max_z = std::min<float>(base_max_pt.z, new_max_pt.z) + tolerance;

	min_x = std::max<float>(base_min_pt.x, new_min_pt.x) - tolerance;
	min_y = std::max<float>(base_min_pt.y, new_min_pt.y) - tolerance;
	min_z = std::max<float>(base_min_pt.z, new_min_pt.z) - tolerance;

	if (max_x <= min_x || max_y <= min_y || max_z <= min_z)
	{
		overlap_matrix[pc_i1][pc_i2] = 0;
		overlap_matrix[pc_i2][pc_i2] = 0;
		intersections[pc_i1][pc_i2] = nullptr;
		intersections[pc_i2][pc_i1] = nullptr;

		return;
	}

	Eigen::Vector4f max(max_x, max_y, max_z, 1.0);
	Eigen::Vector4f min(min_x, min_y, min_z, 1.0);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cropped1(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cropped2(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	pcl::CropBox<pcl::PointXYZRGBNormal> new_crop_box;
	new_crop_box.setMax(max);
	new_crop_box.setMin(min);
	new_crop_box.setInputCloud(patches[pc_i1]);
	new_crop_box.filter(*cropped1);

	pcl::CropBox<pcl::PointXYZRGBNormal> base_crop_box;
	base_crop_box.setMax(max);
	base_crop_box.setMin(min);
	base_crop_box.setInputCloud(patches[pc_i2]);
	base_crop_box.filter(*cropped2);

	if (cropped1->size() > OVERLAP_THRESHOLD && 
		cropped2->size() > OVERLAP_THRESHOLD)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud12 = 
			closest_point_overlap(cropped1, cropped2);
		overlap_matrix[pc_i1][pc_i2] = point_cloud12->size();
		intersections[pc_i1][pc_i2] = point_cloud12;

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud21 = 
			closest_point_overlap(cropped2, cropped1);
		overlap_matrix[pc_i2][pc_i1] = point_cloud21->size();
		intersections[pc_i2][pc_i1] = point_cloud21;

		return;
	}

	overlap_matrix[pc_i1][pc_i2] = 0;
	overlap_matrix[pc_i2][pc_i2] = 0;
	intersections[pc_i1][pc_i2] = nullptr;
	intersections[pc_i2][pc_i1] = nullptr;

}

void rAlignmentEngine::write_nonrigid_config(
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
	std::string final_params) const 
{
	std::fstream f(config_file, std::ios::out);
	f << "[FILES]" << std::endl;

	f << "model = ";
	if (!source_file.empty()) 
	{
		f << "./" << source_file;
	}
	f << std::endl;

	f << "scene = ";
	if (!target_file.empty())
	{
		f << "./" << target_file;
	}
	f << std::endl;

	f << "ctrl_pts = ";
	if (!ctrl_points.empty())
	{
		f << "./" << ctrl_points;
	}
	f << std::endl;

	f << "init_affine = ";
	if (!init_affine.empty())
	{
		f << "./" << init_affine;
	}
	f << std::endl;

	f << "init_tps = ";
	if (!init_tps.empty())
	{
		f << "./" << init_tps;
	}
	f << std::endl;

	f << "init_params = ";
	if (!init_params.empty())
	{
		f << "./" << init_params;
	}
	f << std::endl;

	f << "final_rigid = ./";
	if (!final_rigid.empty())
	{
		f << final_rigid;
	}
	else {
		f << "final_rigid.txt";
	}
	f << std::endl;

	f << "final_affine = ./";
	if (!final_affine.empty())
	{
		f << final_affine;
	}
	else {
		f << "final_affine.txt";
	}
	f << std::endl;

	f << "final_tps = ./";
	if (!final_tps.empty())
	{
		f << "final_tps = ./";
	}
	else {
		f << "final_tps.txt";
	}
	f << std::endl;

	f << "final_params = ./";
	if (!final_params.empty())
	{
		f << final_params;
	}
	else {
		f << "final_params.txt";
	}
	f << std::endl;

	f << "transformed_model = ./";
	if (!transformed_model.empty())
	{
		f << transformed_model;
	}
	else {
		f << "transformed_model.txt";
	}
	f << std::endl;

	const int normalize = 1;
	const int level = 4;
	float sigma[level] = {1.0};
	int lambda[level] = {1};
	int fix_affine[level] = { 0 };
	int max_function_evals[level] = { 100 };

	f << "[GMMREG_OPT]" << std::endl <<
		std::string("normalize = ") + std::to_string(normalize) << std::endl <<
		std::string("level = ") + std::to_string(level) << std::endl <<
		"sigma =";

	for (int i = 0; i < level; i++)
	{
		f << " " << sigma[i];
	}
	f << std::endl;

	f << "lambda =";
	for (int i = 0; i < level; i++)
	{
		f << " " << lambda[i];
	}
	f << std::endl;

	f << "fix_affine =";
	for (int i = 0; i < level; i++)
	{
		f << " " << fix_affine[i];
	}
	f << std::endl;

	f << "max_function_evals =";
	for (int i = 0; i < level; i++)
	{
		f << " " << max_function_evals[i];
	}
}

void rAlignmentEngine::write_point_cloud_txt(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud, 
	std::string path) const
{
	std::fstream f(path, std::ios::out);

	for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = 
			point_cloud->begin();
		it != point_cloud->end(); 
		++it)
	{
		f << it->x << " " << it->y << " " << it->z << std::endl;
	}
}

void rAlignmentEngine::calculate_corrs()
{
	for (int i = 0; i < downsampled.size(); i++)
	{
		pcl::search::KdTree<pcl::PointXYZRGBNormal> tree1;
		tree1.setInputCloud(downsampled[i]);

		for (int j = i + 1; j < downsampled.size(); j++)
		{
			if (!overlap_matrix.empty() && 
				overlap_matrix[i][j] == 0)
			{
				continue;
			}

			pcl::search::KdTree<pcl::PointXYZRGBNormal> tree2;
			tree2.setInputCloud(downsampled[j]);

			std::vector<int> indices1, indices2;
			std::vector<float> sq_distance1, sq_distance2;

			for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator it =
				downsampled[i]->begin();
				it != downsampled[i]->end();
				++it)
			{
				indices2.clear();
				indices2.resize(1);
				sq_distance2.clear();
				sq_distance2.resize(1);

				tree2.nearestKSearch(*it, 1, indices2, sq_distance2);

				indices1.clear();
				indices1.resize(1);
				sq_distance1.clear();
				sq_distance1.resize(1);

				tree1.nearestKSearch(
					downsampled[j]->operator[](indices2[0]), 
					1, 
					indices1, 
					sq_distance1);
				
				if (pcl::geometry::distance(
						downsampled[i]->operator[](indices1[0]), 
						*it) <= 
							tolerance
					/*std::sqrt(sq_distance2[0]) <= tolerance*/)
				{
					Eigen::Vector3d c1(it->x, it->y, it->z);
					Eigen::Vector3d c2(
						downsampled[j]->operator[](indices2[0]).x,
						downsampled[j]->operator[](indices2[0]).y,
						downsampled[j]->operator[](indices2[0]).z);

					corrs.push_back(
						std::make_tuple(i, c1, j, c2));
				}
			}
		}
	}
}

}


