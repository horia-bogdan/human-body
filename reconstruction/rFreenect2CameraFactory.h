#ifndef RFREENECT2CAMERAFACTORY_H
#define RFREENECT2CAMERAFACTORY_H

#include "rCameraFactory.h"

#include <libfreenect2\libfreenect2.hpp>

namespace rec3D 
{

class rCamera;

class rFreenect2CameraFactory : public rCameraFactory
{
public:
	rFreenect2CameraFactory();

	virtual std::unique_ptr<rCamera> find_camera();

	virtual ~rFreenect2CameraFactory();
	
private:
	int n_devices;
	int device_index;

	libfreenect2::Freenect2 freenect2;
};
}

#endif