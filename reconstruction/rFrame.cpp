#include "rFrame.h"

#include <string>

#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include<opencv2\highgui\highgui.hpp>

namespace rec3D
{

rFrame::rFrame(const Type type)
	: type(type)
{
}

void rFrame::filter(const double& lower_bound, const double& upper_bound)
{
	float* depth_data = reinterpret_cast<float*>(data());

	for (int i = 0; i < rows() * columns(); i++)
	{
		float cur_depth = depth_data[i] / 1000.0;

		if (cur_depth != 0.0 && 
			(cur_depth < lower_bound || cur_depth > upper_bound))
		{
			depth_data[i] = 0.0;
		}
	}
}

void rFrame::save(std::string path, const int& index) const
{
	switch (type)
	{
	case RGB:
	{
		/*cv::Mat matrix (rows(), columns(), CV_8UC4, data());
		cv::cvtColor(matrix, matrix, cv::COLOR_BGRA2BGR);*/
		cv::Mat matrix;
		to_opencv_mat(matrix, true);
		cv::imwrite(path + std::string("rgb") + std::to_string(index) + std::string(".jpg"), matrix);
	}
	break;
	case DEPTH:
	{
		/*cv::Mat matrix(rows(), columns(), CV_32FC1, data());
		cv::Mat depth;
		matrix = matrix / 4096.0f;
		matrix.convertTo(depth, CV_8UC1, 255);*/

		cv::Mat matrix;
		to_opencv_mat(matrix, true);
		cv::imwrite(path + std::string("depth") + std::to_string(index) + std::string(".jpg") , matrix);
	}
	break;
	}
}

void rFrame::to_opencv_mat(cv::Mat& matrix, bool to_image) const
{
	switch (type)
	{
	case RGB: 
	{
		cv::Mat mat;
		cv::Mat(rows(), columns(), CV_8UC4, data()).copyTo(mat);
		cv::cvtColor(mat, matrix, cv::COLOR_BGRA2BGR);
		break;
	}
	case DEPTH:
	{
		cv::Mat mat;
		cv::Mat(rows(), columns(), CV_32FC1, data()).copyTo(mat);
		matrix = mat;
		
		if (to_image)
		{
			matrix /= 4096.0f;
			matrix.convertTo(matrix, CV_8UC1, 255);
		}

		break;
	}
	case IR:
	{
		cv::Mat ir_matrix;
		cv::Mat(
			rows(),
			columns(),
			CV_32FC1,
			data()).copyTo(ir_matrix);
		ir_matrix /= 65535.0;
		ir_matrix.convertTo(matrix, CV_8UC1, 255.0);
		break;
	}
	}
}

rFrame::~rFrame()
{
}

}