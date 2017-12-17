#ifndef ROUTLIERREMOVAL_H
#define ROUTLIERREMOVAL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rec3D
{

class rOutlierRemoval
{
public:
	rOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

	void apply(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

	virtual ~rOutlierRemoval();

private:
	std::vector<int> labels;
	double epsilon;
	int min_points;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input;
};

}

#endif

