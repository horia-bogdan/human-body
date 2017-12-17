#ifndef RMAINENGINE_H
#define RMAINENGINE_H

#include "rAlignmentEngine.h"
#include "rCalibration.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include<vector>
#include<memory>

namespace rec3D
{

class rCamera;
class rCameraFactory;
class rTransformationMatrix;


class rMainEngine
{	
public:
	enum Alignment {
		ROUGH,
		ICP,
		NON_RIGID
	};

	typedef std::vector<std::unique_ptr<rCamera>>::const_iterator 
		camera_iterator;

	typedef std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator
		point_cloud_iterator;

	typedef std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator
		point_cloud_const_iterator;

	typedef 
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>::const_iterator
		points_colors_normals_iterator;
	
	typedef std::vector<pcl::PointCloud<pcl::Normal>::Ptr>::const_iterator 
		normals_iterator;

	enum Type
	{
		LIBFREENECT2
	};

	rMainEngine();

	bool aligned() const;

	void calculate_point_clouds();
	
	void capture_all(const int& n_frames);

	void capture_calibration_frame();

	std::vector<bool> calibrate_all(
		const int& columns, 
		const int& rows, 
		const double& square_size);

	camera_iterator cameras_begin() const;

	camera_iterator cameras_end() const;

	bool combine_point_clouds(Alignment);

	void estimate_normals();

	void filter_depth(
		const double& lower_bound = 0.0, 
		const double& upper_bound = std::numeric_limits<double>::max());
	
	bool find(std::unique_ptr<rCameraFactory>);

	void fuse_all();

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr 
		get_cloud_xyz_rgb_normal(const int& index) const;

	void load(std::string file_path);

	point_cloud_const_iterator point_clouds_begin() const;

	point_cloud_const_iterator point_clouds_end() const;

	bool read_calibration_from_file(std::string file_path);

	void reconstruction(const int& index);
	
	void register_all_rgbd();
	
	void remove_outliers();

	void remove_point_cloud(const int& index);

	void reset_calibration();
	
	void save_data();

	void save_calibration_to_file(std::string file_path) const;

	void save_point_cloud(const int& index, const std::string& file_path) const;

	void save_mesh(std::string file_path) const;

	void set_capture_method(Type);

	~rMainEngine();

private:
	void rough_alignment();

	rAlignmentEngine alignment;

	Type type;

	std::vector<std::unique_ptr<rCamera>> cameras;

	std::unique_ptr<rCalibration> calibration_system;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
		partial_point_clouds;

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
		point_clouds;

	pcl::PolygonMesh mesh;
	
	bool all_cameras_added;

	int input_size;
};

}

#endif