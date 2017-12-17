#ifndef RTRANSFORMATIONMATRIX_H
#define RTRANSFORMATIONMATRIX_H

#include "rMatrix.h"

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include<memory>

namespace rec3D
{

class rTransformationMatrix :
	public rMatrix
{
public:
	rTransformationMatrix();
	rTransformationMatrix(const cv::Mat& matrix);
	rTransformationMatrix(const cv::Mat& rotation, const cv::Mat& translation);
	rTransformationMatrix(const Eigen::Matrix<float, 4, 4>&);
	rTransformationMatrix(const rTransformationMatrix&);

	bool empty() const;

	rTransformationMatrix inv() const;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr operator* (
		const pcl::PointCloud<pcl::PointXYZRGBNormal>&) const;

	rTransformationMatrix operator* (const rTransformationMatrix&) const;
	
	~rTransformationMatrix();

private:
	bool empty_matrix;

	Eigen::Matrix4f get_matrix_eigen() const;
};

}

#endif