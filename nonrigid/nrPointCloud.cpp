#include "nrPointCloud.h"

#include <pcl/filters/voxel_grid.h>

namespace nonrigid
{

nrPointCloud::nrPointCloud()
	: point_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>())
{
}

nrPointCloud::nrPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
	:point_cloud(cloud)
{
}

void nrPointCloud::append(
	const Eigen::Vector3d& point, 
	const float& rgb, 
	const Eigen::Vector3d& normal)
{
	pcl::PointXYZRGBNormal new_point;
	
	new_point.x = point(0);
	new_point.y = point(1);
	new_point.z = point(2);

	new_point.rgb = rgb;

	new_point.normal_x = normal(0);
	new_point.normal_y = normal(1);
	new_point.normal_z = normal(2);

	point_cloud->insert(point_cloud->end(), new_point);
}

nrPointCloud nrPointCloud::downsample(const double& leaf_size) const
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_point_cloud(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_grid;
	voxel_grid.setInputCloud(point_cloud);
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_grid.filter(*new_point_cloud);

	return nrPointCloud(new_point_cloud);
}

Eigen::Vector3d nrPointCloud::operator()(const int& i) const
{
	Eigen::Vector3d point;
	point.setZero();

	point(0) = point_cloud->operator[](i).x;
	point(1) = point_cloud->operator[](i).y;
	point(2) = point_cloud->operator[](i).z;

	return point;
}

Eigen::Vector3d nrPointCloud::normal(const int& i) const
{
	Eigen::Vector3d normal;
	normal.setZero();

	normal(0) = point_cloud->operator[](i).normal_x;
	normal(1) = point_cloud->operator[](i).normal_y;
	normal(2) = point_cloud->operator[](i).normal_z;

	return normal;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr nrPointCloud::pointer()
{
	return point_cloud;
}

float nrPointCloud::rgb(const int& i) const
{
	return point_cloud->operator[](i).rgb;
}

int nrPointCloud::size() const
{
	return point_cloud->size();
}

nrPointCloud::~nrPointCloud()
{
}

}