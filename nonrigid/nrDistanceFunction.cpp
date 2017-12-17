#include "nrDistanceFunction.h"

#include<cmath>

namespace nonrigid
{

nrDistanceFunction::nrDistanceFunction()
{
}

double nrDistanceFunction::operator()(
	const Eigen::Vector3d& x_1,
	const Eigen::Vector3d& x_2) const
{
	return std::sqrt((x_1 - x_2).squaredNorm());
}

nrDistanceFunction::~nrDistanceFunction()
{
}

}
