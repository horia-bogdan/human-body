#include "rMatrix.h"

#include <opencv2/core/core.hpp>

namespace rec3D
{

rMatrix::rMatrix()
	:matrix()
{	
}

rMatrix::rMatrix(const cv::Mat& r_matrix)
	: matrix()
{
	r_matrix.convertTo(matrix, CV_64F);
}

rMatrix::rMatrix(const rMatrix& r_matrix)
	:matrix(r_matrix.get_matrix_opencv())
{
	matrix.convertTo(matrix, CV_64F);
}

double rMatrix::at(const int& row, const int& column) const
{
	return (double)matrix.at<double>(row, column);
}

cv::Mat rMatrix::get_matrix_opencv() const
{
	return matrix;
}

void rMatrix::operator=(const rMatrix& r_matrix)
{
	r_matrix.get_matrix_opencv().copyTo(matrix);
}

rMatrix::~rMatrix()
{
}

}
