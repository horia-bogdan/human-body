#include "rCalibration.h"

#include "rFrame.h"
#include "rRegistration.h"

#include "../calib/c2d_svd_trans_estim.hpp"

#include <opencv2\calib3d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>


namespace rec3D
{

rCalibration::Chessboard::Chessboard(
	const int& points_per_row, 
	const int& points_per_column,
	const double& square_size)
	: points_per_row(points_per_row),
	points_per_column(points_per_column),
	square_size(square_size)
{
}

bool rCalibration::Chessboard::find_calibration_points(
	cv::Mat frame,
	std::vector<cv::Point2f>& points) const
{
	cv::Size pattern_size(points_per_row, points_per_column);

	bool found = cv::findChessboardCorners(
		frame, 
		pattern_size, 
		points, 
		CV_CALIB_CB_ADAPTIVE_THRESH |
		CV_CALIB_CB_NORMALIZE_IMAGE);

	if (!points.empty() &&
		points.front().x < points.back().x &&
		points.front().y < points.back().y)
	{
		std::reverse(points.begin(), points.end());
	}

	/*
	cv::drawChessboardCorners(frame, pattern_size, points, found);
	cv::imshow("corners", frame);
	cv::waitKey(0);
	*/


	return found;
}

bool rCalibration::Chessboard::find_object_points()
{
	object_points.clear();

	for (int i = 0; i < points_per_column; i++)
	{
		for (int j = 0; j < points_per_row; j++)
		{
			object_points.push_back(cv::Point3f(j*square_size, i*square_size, 0));
		}
	}

	return true;
}

rCalibration::rCalibration()
	:image_size(cv::Mat().size())
{
}

void rCalibration::add(
	const std::vector<std::shared_ptr<rFrame>>& rgb, 
	const std::vector<std::shared_ptr<rFrame>>& depth,
	const int& i)
{
	int index = i - 1;

	if (index >= 0 && index < cameras_frames.size())
	{
		cameras_frames[index].insert(
			cameras_frames[index].end(), 
			rgb.begin(), 
			rgb.end());

		cameras_depth[index].insert(
			cameras_depth[index].end(),
			depth.begin(),
			depth.end());
	}
	else {
		cameras_frames.push_back(rgb);
		cameras_depth.push_back(depth);
	}
}

rCalibration::const_iterator rCalibration::begin() const
{
	return transformations.begin();
}

std::vector<bool> rCalibration::calibrate_all(CalibrationMethod& method)
{
	if (cameras_frames.size() < 2)
	{
		return calibrated;
	}

	method.find_object_points();

	//find all pattern points
	int visible_count = 0;
	cameras_images_points.clear();
	has_pattern.clear();
	for (cameras_frames_iterator c_f_it = cameras_frames.begin();
		c_f_it != cameras_frames.end();
		++c_f_it)
	{
		std::vector<std::vector<cv::Point2f>> images_points;
		bool this_has_pattern = find_images(method, *c_f_it, images_points);
		
 		cameras_images_points.push_back(images_points);
		has_pattern.push_back(this_has_pattern);
		
		if (this_has_pattern)
		{
			visible_count++;
		}
	}

	if (visible_count < 2)
	{
		return calibrated;
	}

	//find which camera to calibrate to
	int calibrate_to_index = -1;
	bool first_attempt = calibrated.empty();
	std::vector<bool> old_calibrated = calibrated;
	for(int i = 0; i < cameras_frames.size(); i++)
	{
		if (!first_attempt)
		{
			if (old_calibrated[i] && has_pattern[i])
			{
				calibrate_to_index = i;
				break;
			}
		}
		else {
			if (has_pattern[i])
			{
				calibrate_to_index = i;
				break;
			}
		}
	}
	
	if (calibrate_to_index == -1)
	{
		return calibrated;
	}

	//calibrate each camera with respect to calibrate_to_index
	bool success_initial = true;
	calibrated.assign(cameras_frames.size(), false);
	std::vector<rTransformationMatrix> old_transformations = transformations;
	transformations.clear();
	int calibrated_count = 0;

	for (std::vector<bool>::iterator c_it = calibrated.begin(); 
		c_it != calibrated.end(); 
		++c_it)
	{
		double result = calibrate(
			method, 
			calibrate_to_index, 
			c_it - calibrated.begin());

		if (result <= 1.0)
		{
			*c_it = true;
			calibrated_count++;
		}
		else {
			*c_it = false;
			success_initial = false;
		}
	}

	if (calibrated_count < 2)
	{
		calibrated = old_calibrated;
		transformations = old_transformations;
		return calibrated;
	}

	if (first_attempt)
	{
		return calibrated;
	}

	for (int i = 0; i < cameras_frames.size(); i++)
	{
		if (old_calibrated[i])
		{
			transformations[i] = old_transformations[i];
			calibrated[i] = true;
		} 
		else if (calibrated[i])
		{
			transformations[i] = 
				old_transformations[calibrate_to_index] * transformations[i];
		}
	}

	return calibrated;
}

void rCalibration::clear_frames()
{
	cameras_frames.clear();
	cameras_depth.clear();
}

rCalibration::const_iterator rCalibration::end() const
{
	return transformations.end();
}

bool rCalibration::read(std::string file_path)
{
	transformations.clear();
	cv::FileStorage file(file_path, cv::FileStorage::READ);
	
	int i = 0;
	while(true)
	{
		cv::Mat matrix;
		file[std::string("camera") + std::to_string(i)] >> matrix;

		if (matrix.empty())
		{
			break;
		}

		matrix.convertTo(matrix, CV_64F);
		transformations.push_back(matrix);
		i++;
	}

	file.release();

	return i > 1;
}

void rCalibration::save(std::string file_path) const
{
	cv::FileStorage file(file_path, cv::FileStorage::WRITE);

	int i = 0;
	for (const_iterator it = begin(); it != end(); ++it, i++)
	{
		file << 
			std::string("camera") + std::to_string(i) << 
			(*it).get_matrix_opencv();
	}

	file.release();
}

void rCalibration::set_intrinsics_guess(rCameraMatrix intrinsics)
{
	intrinsics_guess = intrinsics;
}

rCalibration::CameraPair::CameraPair(
	rCameraMatrix camera_matrix,
	std::vector<std::vector<cv::Point2f>>& rgb1,
	std::vector<std::vector<cv::Point2f>>& rgb2,
	std::vector<std::shared_ptr<rFrame>>& depth_frame1,
	std::vector<std::shared_ptr<rFrame>>& depth_frame2)
	: fail(false)
{
	int size = rgb1.size();

	if (size != rgb2.size() && 
		size != depth_frame1.size() && 
		size != depth_frame2.size())
	{
		return;
	}

	frames_points[0].resize(size);
	frames_points[1].resize(size);
	frames_centroids[0].resize(size, cv::Point3f(0.0, 0.0, 0.0));
	frames_centroids[1].resize(size, cv::Point3f(0.0, 0.0, 0.0));

	/*
	for (int i = 0; i < size; i++)
	{
		int points_size = rgb1[i].size();
		
		if (points_size != rgb2[i].size())
		{
			return;
		}

		frames_points[0][i].resize(points_size);
		frames_points[1][i].resize(points_size);

		int n_centroids[2] = { 0, 0 };

		for (int j = 0; j < points_size; j++)
		{
			int r1 = std::floor(rgb1[i][j].x + 0.5);
			int c1 = std::floor(rgb1[i][j].y + 0.5);

			cv::Point3f point1 = camera1[i]->get_point(r1, c1);
			if (!isnan(point1.x) && !isnan(point1.y) && !isnan(point1.z))
			{
				frames_points[0][i][j] = point1;
				frames_centroids[0][i] += point1;
				n_centroids[0]++;
			}


			int r2 = std::floor(rgb2[i][j].x + 0.5);
			int c2 = std::floor(rgb2[i][j].y + 0.5);
			cv::Point3f point2 = camera2[i]->get_point(r2, c2);

			if (!isnan(point2.x) && !isnan(point2.y) && !isnan(point2.z))
			{
				frames_points[1][i][j] = point2;
				frames_centroids[1][i] += point2;
				n_centroids[1]++;
			}
		}

		frames_centroids[0][i] /= n_centroids[0];
		frames_centroids[1][i] /= n_centroids[1];

		centroid[0] += frames_centroids[0][i];
		centroid[1] += frames_centroids[0][i];
	}

	centroid[0] /= size;
	centroid[1] /= size;*/
	
	this->camera_matrix = camera_matrix;

	for (int i = 0; i < size; i++)
	{
		cv::Mat depth1;
		depth_frame1[i]->to_opencv_mat(depth1);
		cv::Mat depth2;
		depth_frame2[i]->to_opencv_mat(depth2);

		restore_3d(
			i, 
			rgb1[i], 
			rgb2[i], 
			depth1,
			depth2);
	}

	centroid[0] /= size;
	centroid[1] /= size;
}

void rCalibration::CameraPair::restore_3d(
	const int& index, 
	std::vector<cv::Point2f>& rgb1, 
	std::vector<cv::Point2f>& rgb2,
	cv::Mat& depth1,
	cv::Mat& depth2)
{
	std::vector<cv::Point2f> rgbPoints[2];
	rgbPoints[0] = rgb1;
	rgbPoints[1] = rgb2;

	int szPoints = rgbPoints[0].size();
	for (int k = 0; k < 2; ++k)
	{
		frames_centroids[k][index] = cv::Point3f(0, 0, 0);
		frames_points[k][index].resize(szPoints);
	}

	cv::Mat dep[2];
	dep[0] = depth1;
	dep[1] = depth2;

	cv::Point3f partial_t[2] = { cv::Point3f(0.0, 0.0, 0.0) };

	for (int k = 0; k < 2; ++k)
	{
		bool badDepth = false;
		cv::Mat depsc;
		dep[k].convertTo(depsc, CV_32FC1);
		for (size_t ci = 0; ci < szPoints; ++ci)
		{
			const float y = rgbPoints[k][ci].x;
			const float x = rgbPoints[k][ci].y;
			const int x0 = std::floor(x + 0.5), y0 = std::floor(y + 0.5);
			if (0 > x0 || 0 > y0 || depsc.rows <= x0 || depsc.cols <= y0)
			{
				std::cout << "C2DSVDTransEstim::FramePair::Restore3d: (" << x0 << ", " << y0 << ") out of range (" << depsc.rows << ", " << depsc.cols << ")!\n";
				return;
			}
			cv::Point3f pos = camera_matrix.get_3d(x0, y0);
			float dd = depsc.at<float>(x0, y0) / 1000.0;
			if (dd < 0.001) // this should not happen!
			{
				fail = true;

				std::cout << x0 << " " << y0 << ": ";

				for (int r = -3; r < 4; ++r)
					for (int c = -3; c < 4; ++c)
					{
						const float v = depsc.at<float>(x0 + r, y0 + c);
						// if (10000 < v || -10000 > v)
						std::cout << v << " ";
					}


				std::cout << " -- " << dd << std::endl;
			}
			pos *= dd;
			frames_points[k][index][ci] = pos;
			frames_centroids[k][index] += pos;
		}
	}
	for (int k = 0; k < 2; ++k)
	{
		frames_centroids[k][index] /= (float)szPoints;
		//partial_t[k] /= (float)szPoints;
		centroid[k] += //partial_t[k];
			frames_centroids[k][index];
	}
}

bool rCalibration::find_images(
	const CalibrationMethod& method,
	const std::vector<std::shared_ptr<rFrame>>& frames,
	std::vector<std::vector<cv::Point2f>>& images_points)
{
	images_points.clear();

	for (std::vector<std::shared_ptr<rFrame>>::const_iterator it = 
			frames.begin();
		it != frames.end();
		++it)
	{
		cv::Mat matrix;
		(*it)->to_opencv_mat(matrix);

		/*
		switch ((*it)->type)
		{
		case rFrame::Type::RGB:
		{
			cv::Mat rgb_matrix(
				(*it)->rows(),
				(*it)->columns(),
				CV_8UC4,
				(*it)->data());
			cv::cvtColor(rgb_matrix, matrix, cv::COLOR_BGRA2BGR);
			break;
		}

		case rFrame::Type::IR:
		{
			cv::Mat ir_matrix(
				(*it)->rows(),
				(*it)->columns(),
				CV_32FC1,
				(*it)->data());
			ir_matrix /= 65535.0;
			ir_matrix.convertTo(matrix, CV_8UC1, 255.0);

			cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.5, cv::Size(32, 32));
			clahe->apply(matrix, matrix);

			break;
		}
		}*/

		std::vector<cv::Point2f> points;

		bool result = method.find_calibration_points(matrix, points);
		
		if (!result)
		{
			return false;
		}
		images_points.push_back(points);

		if (image_size == cv::Mat().size())
		{
			image_size = matrix.size();
		}
		else if (image_size != matrix.size())
		{
			return false;
		}
	}

	if (images_points.empty())
	{
		return false;
	}

	return true;
}

double rCalibration::calibrate(
	const CalibrationMethod& method,
	const int& index_to,
	const int& index_from)
{
	if (index_to == index_from)
	{
		transformations.push_back(
			rTransformationMatrix(
				cv::Mat::eye(3, 3, CV_64F), 
				cv::Mat::zeros(3, 1, CV_64F)));
		return 0.0;
	}

	if (!has_pattern[index_to] || !has_pattern[index_from])
	{
		transformations.push_back(rTransformationMatrix());
		return std::numeric_limits<double>::max();
	}

	
	CameraPair camera_pair(
		intrinsics_guess,
		cameras_images_points[index_from], 
		cameras_images_points[index_to],
		cameras_depth[index_from],
		cameras_depth[index_to]);

	if (camera_pair.fail)
	{
		transformations.push_back(
			rTransformationMatrix(
				cv::Mat::eye(3, 3, CV_64F),
				cv::Mat::zeros(3, 1, CV_64F)));
		return std::numeric_limits<double>::max();
	}

	rTransformationMatrix transform = compute_transformation_SVD(camera_pair);
	if (!transform.empty())
	{
		transformations.push_back(transform);
		return 0.0;
	}

	return std::numeric_limits<double>::max();

	/*
	std::vector<std::vector<cv::Point3f>> object_points;
	object_points.resize(
		cameras_images_points[index_to].size(), 
		method.object_points);
	
	cv::Mat camera_matrix1 = intrinsics_guess.get_matrix_opencv();
	cv::Mat distortion_coeff1 = intrinsics_guess.get_distortion();
	cv::Mat camera_matrix2 = intrinsics_guess.get_matrix_opencv();
	cv::Mat distortion_coeff2 = intrinsics_guess.get_distortion();
	

	cv::Mat rvec, tvec;

	cv::calibrateCamera(
		object_points,
		cameras_images_points[index_from],
		image_size,
		camera_matrix1,
		distortion_coeff1,
		rvec,
		tvec);
//	CV_CALIB_USE_INTRINSIC_GUESS);

	cv::calibrateCamera(
		object_points,
		cameras_images_points[index_to],
		image_size,
		camera_matrix2,
		distortion_coeff2,
		rvec,
		tvec);
//	CV_CALIB_USE_INTRINSIC_GUESS);

	cv::Mat rotation;
	cv::Mat translation;
	cv::Mat essential, fundamental;

	double reprojection_error = cv::stereoCalibrate(
		object_points,
		cameras_images_points[index_from],
		cameras_images_points[index_to],
		camera_matrix1,
		distortion_coeff1,
		camera_matrix2,
		distortion_coeff2,
		image_size,
		rotation,
		translation,
		essential,
		fundamental,
		CV_CALIB_FIX_INTRINSIC);

	transformations.push_back(rTransformationMatrix(rotation, translation));
	
	return reprojection_error; 
	*/
}

rTransformationMatrix rCalibration::compute_transformation_SVD(
	const CameraPair& camera_pair) const
{
	int size = camera_pair.frames_points[0].size();
	if (size != camera_pair.frames_points[1].size() ||
		size != camera_pair.frames_centroids[0].size() || 
		size != camera_pair.frames_centroids[1].size())
	{
		return rTransformationMatrix();
	}

	const float sf = 1e5;
	cv::Matx33f ident = cv::Matx33f::eye();
	cv::Matx33f cov = cv::Matx33f::zeros();
	for (size_t nf = 0; nf < size; ++nf)
	{
		if (camera_pair.frames_points[0][nf].size() != 
			camera_pair.frames_points[1][nf].size())
		{
			return rTransformationMatrix();
		}

		const size_t npairs = camera_pair.frames_points[0][nf].size();
		for (size_t pp = 0; pp < npairs; ++pp)
		{
			const cv::Point3f pl = camera_pair.frames_points[0][nf][pp];
			const cv::Point3f pr = camera_pair.frames_points[1][nf][pp];
			if (0.001 > pl.z || 0.001 > pr.z) continue;
			const cv::Vec3f vl = (pl - camera_pair.frames_centroids[0][nf]) / sf;
			const cv::Vec3f vr = (pr - camera_pair.frames_centroids[1][nf]) / sf;
			for (int ii = 0; ii < 3; ++ii)
				for (int jj = 0; jj < 3; ++jj)
					cov(ii, jj) += vl(ii) * vr(jj);
		}
	}
	cv::Matx33f u, vt;
	cv::Matx31f w;
	cv::SVD::compute(cov, w, u, vt);
	cv::Matx33f ut = u.t(), v = vt.t();
	double det = cv::determinant(v * ut);
	if (0 > det) ident(2, 2) = -1;
	cv::Matx33f rotate_l2r = v * ident * ut;
	cv::Point3f translate_l2r = //cv::Point3f(0.0, 0.0, 0.0);
		camera_pair.centroid[1] - rotate_l2r * camera_pair.centroid[0];
		

	return rTransformationMatrix(cv::Mat(rotate_l2r), cv::Mat(translate_l2r));
}

rCalibration::~rCalibration()
{

}

}
