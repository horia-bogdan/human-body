#include "rCameraMatrix.h"

namespace rec3D

{

rCameraMatrix::rCameraMatrix()
	:rMatrix(cv::Mat::eye(3, 3, CV_64F)),
	distortion_coefficients(cv::Mat::zeros(8, 1, CV_64F))
{
}

rCameraMatrix::rCameraMatrix(
	const cv::Mat& matrix, 
	const cv::Mat& distortion_coefficients)
	:rMatrix(matrix),
	distortion_coefficients(distortion_coefficients)
{
}

rCameraMatrix::rCameraMatrix(const rCameraMatrix& matrix)
	: rMatrix(matrix),
	distortion_coefficients(matrix.get_distortion())
{
}

rCameraMatrix::rCameraMatrix(
	const libfreenect2::Freenect2Device::ColorCameraParams& rgb_params)
	: rMatrix()
{
	double camera_parameters[9] = 
	{ rgb_params.fx, 0.0,           rgb_params.cx,
		0.0,         rgb_params.fy, rgb_params.cy,
		0.0,         0.0,           1.0 };

	cv::Mat mat(3, 3, CV_64F, camera_parameters);
	mat.copyTo(matrix);

	double distortion_parameters[5] = { 0.0 };
	cv::Mat dc(5, 1, CV_64F, distortion_parameters);
	dc.copyTo(distortion_coefficients);
}

rCameraMatrix::rCameraMatrix(
	const libfreenect2::Freenect2Device::IrCameraParams& depth_params)
{
	double camera_parameters[9] =
	{ depth_params.fx, 0.0,             depth_params.cx,
		0.0,           depth_params.fy, depth_params.cy,
		0.0,           0.0,             1.0 };
	
	cv::Mat mat(3, 3, CV_64F, camera_parameters);
	mat.copyTo(matrix);

	double distortion_parameters[5] =
	{   depth_params.k1,
		depth_params.k2,
		depth_params.k3,
		depth_params.p1,
		depth_params.p2 };

	cv::Mat dc(5, 1, CV_64F, distortion_parameters);
	dc.copyTo(distortion_coefficients);
}

cv::Point3f rCameraMatrix::get_3d(const int& r, const int& c) const
{
	double fx = matrix.at<double>(0, 0);
	double fy = matrix.at<double>(1, 1);
	double cx = matrix.at<double>(0, 2);
	double cy = matrix.at<double>(1, 2);

	return cv::Point3f(
		((float)c + 0.5 - cx) / fx,
		((float)r + 0.5 - cy) / fy,
		1
	);
}

cv::Mat rCameraMatrix::get_distortion() const
{
	return distortion_coefficients;
}

rCameraMatrix::~rCameraMatrix()
{
}

}
