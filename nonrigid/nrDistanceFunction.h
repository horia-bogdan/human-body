#ifndef NRDISTANCEFUNCTION_H
#define NRDISTANCEFUNCTION_H

#include<Eigen\Dense>

namespace nonrigid
{

class nrDistanceFunction
{
public:
	nrDistanceFunction();

	virtual double operator()(const Eigen::Vector3d&, const Eigen::Vector3d&) const;

	~nrDistanceFunction();
};

}

#endif

