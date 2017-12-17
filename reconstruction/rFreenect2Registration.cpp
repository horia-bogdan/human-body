#include "rFreenect2Registration.h"

#include "rFreenect2Frame.h"

#include <opencv2\highgui\highgui.hpp>

#include <memory>

namespace rec3D
{

rFreenect2Registration::rFreenect2Registration(
	libfreenect2::Freenect2Device::IrCameraParams ir_params, 
	libfreenect2::Freenect2Device::ColorCameraParams color_params)
	:registration(ir_params, color_params)
{	
}

void rFreenect2Registration::add_frame(
	const libfreenect2::Frame& frame, rFrame::Type type)
{
	frames[type] = &frame;
}

bool rFreenect2Registration::apply()
{
	libfreenect2::Frame undistorted(512, 424, 4);
	libfreenect2::Frame registered(512, 424, 4);

	if (!frames[rFrame::RGB] || !frames[rFrame::DEPTH])
	{
		return false;
	}

	registration.apply(
		frames[rFrame::RGB], 
		frames[rFrame::DEPTH], 
		&undistorted, 
		&registered,
		false);
	
	rgb = std::make_shared<rFreenect2Frame>(rFrame::RGB, registered);
	depth = std::make_shared<rFreenect2Frame>(rFrame::DEPTH, undistorted);

	return true;
}

cv::Point3f rFreenect2Registration::get_point(const int& r, const int& c)
{
	float x, y, z;

	libfreenect2::Frame undistorted(
		depth->columns(),
		depth->rows(),
		depth->bytes_per_pixel(),
		depth->data());

	registration.getPointXYZ(&undistorted, r, c, x, y, z);

	return cv::Point3f(x, y, z);
}

bool rFreenect2Registration::to_point_cloud(
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) const
{
	libfreenect2::Frame undistorted(
		depth->columns(), 
		depth->rows(), 
		depth->bytes_per_pixel(), 
		depth->data());

	libfreenect2::Frame registered(
		rgb->columns(),
		rgb->rows(),
		rgb->bytes_per_pixel(),
		rgb->data());

	for (int i = 0; i < undistorted.height; i++)
	{
		for (int j = 0; j < undistorted.width; j++)
		{
			float x, y, z, rgb;

			registration.getPointXYZRGB(
				&undistorted, 
				&registered, 
				i, 
				j, 
				x, 
				y, 
				z, 
				rgb);

			if (!isnan(x) && !isnan(y) && !isnan(z))
			{
				pcl::PointXYZRGB point;
				point.x = x;
				point.y = y;
				point.z = z;
				point.rgb = rgb;

				point_cloud.push_back(point);
			}
		}
	}

	return true;
}

rFreenect2Registration::~rFreenect2Registration()
{
}

}
