#ifndef RCALIBRATION_H
#define RCALIBRATION_H

#include "rCameraMatrix.h"
#include "rTransformationMatrix.h"

#include <opencv2\core\core.hpp>

#include <memory>
#include <vector>

namespace rec3D
{

class rFrame;
class rRegistration;

class rCalibration
{
public:
	typedef std::vector<rTransformationMatrix>::const_iterator
		const_iterator;

	//--------------------------------------------------------
	//--------------------------------------------------------

	class CalibrationMethod
	{
	public:
		virtual bool find_calibration_points(
			cv::Mat frame,
			std::vector<cv::Point2f>& points) const = 0;

		virtual bool find_object_points() = 0;

		std::vector<cv::Point3f> object_points;
	};

	class Chessboard : public CalibrationMethod
	{
	public:
		Chessboard(
			const int& points_per_row, 
			const int& points_per_column, 
			const double& square_size = 1.0);

		virtual bool find_calibration_points(
			cv::Mat frame,
			std::vector<cv::Point2f>& points) const;

		virtual bool find_object_points();
		
		const int points_per_row, points_per_column;
		const double square_size;
	};

	//--------------------------------------------------------
	//--------------------------------------------------------
	
	rCalibration();

	virtual void add(
		const std::vector<std::shared_ptr<rFrame>>& rgb, 
		const std::vector<std::shared_ptr<rFrame>>& depth,
		const int& index);

	virtual const_iterator begin() const;
	
	virtual std::vector<bool> calibrate_all(CalibrationMethod&);

	virtual void clear_frames();

	virtual const_iterator end() const;

	virtual bool read(std::string file_path);

	virtual void save(std::string file_path) const;

	virtual void set_intrinsics_guess(rCameraMatrix);

	virtual ~rCalibration();

private:
	class CameraPair {
	public:
		CameraPair(
			/*
			const std::vector<std::vector<cv::Point2f>>& rgb1,
			const std::vector<std::vector<cv::Point2f>>& rgb2,
			const std::vector<std::shared_ptr<rRegistration>>& camera1, 
			const std::vector<std::shared_ptr<rRegistration>>& camera2);
			*/
			rCameraMatrix, 
			std::vector<std::vector<cv::Point2f>>& rgb1, 
			std::vector<std::vector<cv::Point2f>>& rgb2,
			std::vector<std::shared_ptr<rFrame>>& depth_frame1,
			std::vector<std::shared_ptr<rFrame>>& depth_frame2);

		
		rCameraMatrix camera_matrix;
		std::vector<std::vector<cv::Point3f>> frames_points[2];
		std::vector<cv::Point3f> frames_centroids[2];
		cv::Point3f centroid[2];

		bool fail;

	private:
		void restore_3d(
			const int& index,
			std::vector<cv::Point2f>& rgb1,
			std::vector<cv::Point2f>& rgb2,
			cv::Mat& depth1,
			cv::Mat& depth2);

		const int MAX_SEARCH_AREA = 4;
	};

	typedef std::vector<std::vector<std::shared_ptr<rFrame>>>::const_iterator 
		cameras_frames_iterator;

	virtual double calibrate(
		const CalibrationMethod&, 
		const int& index_to,
		const int& index_from);

	virtual rTransformationMatrix compute_transformation_SVD(
		const CameraPair& frame_pair) const;

	virtual bool find_images(
		const CalibrationMethod& method,
		const std::vector<std::shared_ptr<rFrame>>& frames,
		std::vector<std::vector<cv::Point2f>>& images_points);

	std::vector<std::vector<std::shared_ptr<rFrame>>> cameras_frames;
	std::vector<std::vector<std::shared_ptr<rFrame>>> cameras_depth;

	//std::vector<std::vector<std::shared_ptr<rRegistration>>> cameras_registrations;

	std::vector<std::vector<std::vector<cv::Point2f>>> cameras_images_points;

	rCameraMatrix intrinsics_guess;
	
	std::vector<bool> calibrated;

	std::vector<bool> has_pattern;

	std::vector<rTransformationMatrix> transformations;

	cv::Size image_size;
};

}

#endif RCALIBRATION_H