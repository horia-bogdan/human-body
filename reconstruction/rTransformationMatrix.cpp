#include "rTransformationMatrix.h"

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\common\transforms.h>

#include <opencv2\core\eigen.hpp>

namespace rec3D
{

rTransformationMatrix::rTransformationMatrix()
	:rMatrix(),
	empty_matrix(true)
{
}

rTransformationMatrix::rTransformationMatrix(const cv::Mat& matrix)
	:rMatrix(matrix),
	empty_matrix(false)
{
}

rTransformationMatrix::rTransformationMatrix(
	const cv::Mat& rotation, 
	const cv::Mat& translation)
	:rMatrix(),
	empty_matrix(false)
{
	cv::Mat r;
	rotation.convertTo(r, CV_64F);
	cv::Mat t;
	translation.convertTo(t, CV_64F);

	cv::hconcat(r, t, matrix);

	double row[4] = { 0.0, 0.0, 0.0, 1.0 };
	cv::Mat row_4(1, 4, CV_64F, row);

	cv::vconcat(matrix, row_4, matrix);
}

rTransformationMatrix::rTransformationMatrix(
	const Eigen::Matrix<float, 4, 4>& mat)
	:empty_matrix(false)
{
	cv::eigen2cv(mat, matrix);
}

rTransformationMatrix::rTransformationMatrix(
	const rTransformationMatrix& r_matrix)
	:rMatrix(r_matrix),
	empty_matrix(r_matrix.empty())
{
}

bool rTransformationMatrix::empty() const
{
	return empty_matrix;
}

rTransformationMatrix rTransformationMatrix::inv() const
{
	return matrix.inv();
}

/*
rPoint rTransformationMatrix::operator*(const rPoint& point)
{
	double f_point[4] = { point.x, point.y, point.z, 1.0 };
	cv::Mat cv_point(4, 1, CV_64F, f_point);
	
	cv::Mat res_point = matrix * cv_point;

	return rPoint(
		res_point.at<double>(0, 0), 
		res_point.at<double>(1, 0), 
		res_point.at<double>(2, 0));
}*/

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
	rTransformationMatrix::operator*(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>& point_cloud) const
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	Eigen::Matrix4f transform = get_matrix_eigen();

	pcl::transformPointCloudWithNormals(point_cloud, *temp, transform);

	return temp;
}

rTransformationMatrix rTransformationMatrix::operator*(
	const rTransformationMatrix& other_matrix) const
{
	return matrix * other_matrix.get_matrix_opencv();
}

Eigen::Matrix4f rTransformationMatrix::get_matrix_eigen() const
{
	Eigen::Matrix4f temp;
	cv::cv2eigen(matrix, temp);

	return temp;
}

rTransformationMatrix::~rTransformationMatrix()
{
}

}
