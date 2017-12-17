#ifndef RFREENECT2CAMERA_H
#define RFREENECT2CAMERA_H

#include "rCamera.h"

#include "rFreenect2FrameListener.h"

#include <libfreenect2\libfreenect2.hpp>

namespace rec3D 
{

class rFreenect2Frame;

class rFreenect2Camera :
	public rCamera
{
public:
	rFreenect2Camera(libfreenect2::Freenect2Device*, const int& id);

	virtual bool capture(const int& n_frames, rFrame::Type type);

	virtual void clear();
	
	virtual std::string name() const;

	virtual int register_rgbd();	
		
	virtual bool to_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>&) const;

	virtual ~rFreenect2Camera();

private:
	virtual bool start();

	virtual bool stop();

	libfreenect2::Freenect2Device* f2device;
		
	rFreenect2FrameListener* listener;

	bool running;		
};
}

#endif