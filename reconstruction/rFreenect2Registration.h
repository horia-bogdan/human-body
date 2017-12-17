#ifndef RFREENECT2REGISTRATION_H
#define RFREENECT2REGISTRATION_H

#include "rRegistration.h"
#include "rFrame.h"

#include<libfreenect2\frame_listener.hpp>
#include<libfreenect2\frame_listener_impl.h>
#include<libfreenect2\registration.h>

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include <map>

namespace rec3D
{

class rFreenect2Registration :
	public rRegistration
{
public:
	rFreenect2Registration(
		libfreenect2::Freenect2Device::IrCameraParams ir_params,
		libfreenect2::Freenect2Device::ColorCameraParams color_params);
	
	void add_frame(const libfreenect2::Frame& frame, rFrame::Type);

	virtual bool apply();

	virtual cv::Point3f get_point(const int& r, const int& c);

	virtual bool to_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>&) const;

	virtual ~rFreenect2Registration();

private:
	std::map<rFrame::Type, const libfreenect2::Frame*> frames;

	libfreenect2::Registration registration;
};

}

#endif