#ifndef RCAMERAMATRIX_H
#define RCAMERAMATRIX_H

#include "rMatrix.h"

#include <libfreenect2\libfreenect2.hpp>

namespace rec3D
{

class rCameraMatrix :
	public rMatrix
{
public:
	rCameraMatrix();
	rCameraMatrix(
		const cv::Mat& camera_matrix, 
		const cv::Mat& distortion_coefficients);
	rCameraMatrix(const rCameraMatrix&);
	rCameraMatrix(const libfreenect2::Freenect2Device::ColorCameraParams&);
	rCameraMatrix(const libfreenect2::Freenect2Device::IrCameraParams&);

	cv::Mat get_distortion() const;

	cv::Point3f get_3d(const int& r, const int& c) const;

	virtual ~rCameraMatrix();

private:
	cv::Mat distortion_coefficients;
};

}

#endif