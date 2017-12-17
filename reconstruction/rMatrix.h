#ifndef RMATRIX_H
#define RMATRIX_H

#include <opencv2/core/core.hpp>

namespace rec3D
{

class rMatrix
{
public:
	rMatrix();
	rMatrix(const cv::Mat&);
	rMatrix(const rMatrix&);
	
	virtual double at(const int& row, const int& column) const;

	virtual cv::Mat get_matrix_opencv() const;

	virtual void operator=(const rMatrix&);

	virtual ~rMatrix();

protected:
	cv::Mat matrix;
};

}

#endif