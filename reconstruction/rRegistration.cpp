#include "rRegistration.h"

namespace rec3D
{

rRegistration::rRegistration()
{
}

void rRegistration::clear()
{
	rgb.reset();
	depth.reset();
}

std::shared_ptr<rFrame> rRegistration::registered_rgb()
{
	return rgb;
}

std::shared_ptr<rFrame> rRegistration::undistorted_depth()
{
	return depth;
}

rRegistration::~rRegistration()
{
}

}