#ifndef NRPOINTCLOUD_H
#define NRPOINTCLOUD_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

namespace nonrigid
{

typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator nrPointCloudIterator;

class nrPointCloud
{
public:

	nrPointCloud();
	nrPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr);

	virtual void append(
		const Eigen::Vector3d& point, 
		const float& rgb, 
		const Eigen::Vector3d& normal);
	
	virtual nrPointCloud downsample(const double& leaf_size) const;
	
	virtual Eigen::Vector3d operator()(const int& i) const;

	virtual Eigen::Vector3d normal(const int& i) const;

	virtual pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointer();

	virtual float rgb(const int& i) const;

	virtual int size() const;

	virtual ~nrPointCloud();

private:
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud;
};

}

#endif

