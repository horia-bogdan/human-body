#ifndef RREGISTRATION_H
#define RREGISTRATION_H

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include <opencv2\core\core.hpp>

#include<memory>

namespace rec3D
{

class rFrame;

class rRegistration
{
public:
	rRegistration();

	virtual bool apply() = 0;

	virtual void clear();

	virtual cv::Point3f get_point(const int& r, const int& c) =  0;

	virtual bool to_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>&) const = 0;

	virtual std::shared_ptr<rFrame> registered_rgb();

	virtual std::shared_ptr<rFrame> undistorted_depth();

	virtual ~rRegistration();

protected:
	std::shared_ptr<rFrame> depth, rgb;
};

}

#endif