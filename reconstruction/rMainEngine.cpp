#include "rMainEngine.h"

#include "rCamera.h"
#include "rFreenect2CameraFactory.h"
#include "rFrame.h"
#include "rInfinitamFusionEngine.h"
#include "rOutlierRemoval.h"
#include "rTransformationMatrix.h"

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>

#include <fstream>
#include <iostream>

namespace rec3D
{
rMainEngine::rMainEngine()
	:all_cameras_added(false),
	input_size(0),
	alignment(0.03)
{
	calibration_system = std::make_unique<rCalibration>();
}

bool rMainEngine::aligned() const
{
	return point_clouds.size() > input_size;
}

void rMainEngine::calculate_point_clouds()
{
	camera_iterator c_it = cameras.begin();
	point_cloud_iterator p_it = partial_point_clouds.begin();

	while (c_it != cameras.end() && p_it != partial_point_clouds.end())
	{
		(*c_it)->to_point_cloud(**p_it);

		++c_it;
		++p_it;
	}
}

void rMainEngine::capture_all(const int& n_frames)
{
	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->clear();
		(*it)->capture(n_frames, rFrame::RGBD);
	}
}

void rMainEngine::capture_calibration_frame()
{
	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->clear();
		(*it)->capture(1, rFrame::RGBD);
	}

	register_all_rgbd();

	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->add_to_calibration(*calibration_system);
	}
}

std::vector<bool> rMainEngine::calibrate_all(
	const int& columns, 
	const int& rows, 
	const double& square_size)
{
	/*
	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->add_to_calibration(calibration_system);
	}*/

	std::vector<bool> result = calibration_system->calibrate_all(
		rCalibration::Chessboard(columns, rows, square_size));
	
	calibration_system->clear_frames();

	return result;
}

rMainEngine::camera_iterator rMainEngine::cameras_begin() const
{
	return cameras.begin();
}

rMainEngine::camera_iterator rMainEngine::cameras_end() const
{
	return cameras.end();
}

bool rMainEngine::combine_point_clouds(Alignment align)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combined_point_cloud(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());


	if (alignment.empty())
	{
		for (int i = 0; i < input_size; i++)
		{
			alignment.add_point_cloud(point_clouds[i]);
		}
	}
	
	switch (align)
	{
	case ROUGH: {
		rough_alignment();
		break;
	}

	case ICP: {
		alignment.align_all_icp();

		break;
	}

	case NON_RIGID: {
		alignment.nonrigid_all();

		for (int i = 0; i < input_size; i++)
		{
			point_clouds[i] = alignment.get_cloud(i);
		}

		break;
	}

	default: return false;
	}


	for (int i = 0; i < input_size; i++)
	{
		*combined_point_cloud += *point_clouds[i];
	}

	bool result = !combined_point_cloud->empty();

	if (result)
	{
		point_clouds.push_back(combined_point_cloud);
	}

	return result;
}

void rMainEngine::estimate_normals()
{
	point_clouds.clear();

	for (point_cloud_iterator it = partial_point_clouds.begin(); 
		it != partial_point_clouds.end(); 
		++it)
	{
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
		normal_estimation.setInputCloud(*it);
		normal_estimation.setSearchSurface(*it);

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());

		normal_estimation.setSearchMethod(tree);

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>());

		normal_estimation.setRadiusSearch(0.01);
		normal_estimation.setViewPoint(0.0, 0.0, -1.0);

		normal_estimation.compute(*cloud_normals);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_rgb_nor(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>());

		pcl::concatenateFields(**it, *cloud_normals, *points_rgb_nor);

		point_clouds.push_back(points_rgb_nor);
	}	

	input_size = point_clouds.size();
}

void rMainEngine::filter_depth(
	const double& lower_bound, 
	const double& upper_bound)
{
	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->filter_depth(lower_bound, upper_bound);
	}
}

bool rMainEngine::find(std::unique_ptr<rCameraFactory> factory)
{
	bool success = false;

	std::unique_ptr<rCamera> camera = factory->find_camera();

	while (camera)
	{
		cameras.push_back(std::move(camera));

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>());
		partial_point_clouds.push_back(cloud);
		
		camera = factory->find_camera();

		success = true;
	}

	return success;
}

void rMainEngine::fuse_all()
{
	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->fuse_frames();
	}
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr 
	rMainEngine::get_cloud_xyz_rgb_normal(const int& index) const
{
	if (index >= 0 && index < point_clouds.size())
	{
		return point_clouds[index];
	}

	return nullptr;
}

void rMainEngine::load(std::string file_path)
{
	if (!file_path.empty())
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>());

		pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(file_path, *point_cloud);

		point_clouds.insert(point_clouds.begin() + input_size, point_cloud);
		
		input_size++;
	}
}

rMainEngine::point_cloud_const_iterator rMainEngine::point_clouds_begin() const
{
	return partial_point_clouds.begin();
}

rMainEngine::point_cloud_const_iterator rMainEngine::point_clouds_end() const
{
	return partial_point_clouds.end();
}

bool rMainEngine::read_calibration_from_file(std::string file_path)
{
	return calibration_system->read(file_path);
}

void rMainEngine::reconstruction(const int& index)
{
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	poisson.setDepth(8);
	poisson.setInputCloud(point_clouds[index]);

	poisson.reconstruct(mesh);
}

void rMainEngine::register_all_rgbd()
{
	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->register_rgbd();
	}
}

void rMainEngine::remove_point_cloud(const int& index)
{
	point_clouds.erase(point_clouds.begin() + index);
}

void rMainEngine::remove_outliers()
{
	for (point_cloud_iterator it = partial_point_clouds.begin(); 
		it != partial_point_clouds.end(); 
		++it)
	{
		if ((*it)->size() > 50)
		{
			rOutlierRemoval outlier_removal(*it);
			outlier_removal.apply(*it);

			/*
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_removal;

			outlier_removal.setInputCloud(*it);
			outlier_removal.setMeanK(100);
			outlier_removal.setStddevMulThresh(0.25);

			outlier_removal.filter(**it);
			*/
		}
	}
}

void rMainEngine::reset_calibration()
{
	calibration_system = std::make_unique<rCalibration>();
}

void rMainEngine::save_data()
{
	std::string path = "";

	for (camera_iterator it = cameras.begin(); it != cameras.end(); ++it)
	{
		(*it)->save_data(path, it-cameras.begin());
	}
}

void rMainEngine::save_calibration_to_file(std::string file_path) const
{
	calibration_system->save(file_path);
}

void rMainEngine::save_point_cloud(
	const int& index, 
	const std::string& file_path) const
{
	if (file_path.empty())
	{
		return;
	}

	if (file_path.substr(file_path.size() - 3, 3).compare(
			std::string("txt")) ==
				0 ||
		file_path.substr(file_path.size() - 3, 3).compare(
			std::string("xyz")) ==
				0)
	{
		std::fstream out(file_path, std::fstream::out);

		for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator it = 
				point_clouds[index]->begin();
			it != point_clouds[index]->end(); 
			++it)
		{
			out << it->x << " " << it->y << " " << it->z << std::endl;
		}

		return;
	}

	if (file_path.substr(file_path.size() - 3, 3).compare(
			std::string("ply")) == 
				0)
	{
		pcl::io::savePLYFile(file_path, *point_clouds[index]);

		return;
	}
}

void rMainEngine::save_mesh(std::string file_path) const
{
	pcl::io::saveOBJFile(file_path, mesh);
}

void rMainEngine::set_capture_method(Type type)
{
	this->type = type;
}

rMainEngine::~rMainEngine()
{
}

void rMainEngine::rough_alignment()
{
	rCalibration::const_iterator t_it = calibration_system->begin();

	for(int i = 0; i < input_size; i++)
	{
		point_clouds[i] = (*t_it) * (*point_clouds[i]);
		++t_it;
	}
}

}