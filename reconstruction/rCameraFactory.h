#ifndef RCAMERAFACTORY_H
#define RCAMERAFACTORY_H

#include<memory>

namespace rec3D
{

class rCamera;

class rCameraFactory
{
public:
	rCameraFactory();

	virtual std::unique_ptr<rCamera> find_camera() = 0;

	~rCameraFactory();
};

}

#endif
