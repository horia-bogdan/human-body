#include "rCamera.h"

#include "rCalibration.h"
#include "rFusionEngine.h"
#include "rInfinitamFusionEngine.h"
#include "rRegistration.h"
#include "rTransformationMatrix.h"

namespace rec3D 
{

rCamera::rCamera(const int& index)
	: calibrated(false),
	registered(false),
	index(index)
{
}

bool rCamera::add_to_calibration(rCalibration& calibration)
{
	if (frames_map[rFrame::Type::RGB].empty())
	{
		return false;
	}

	calibration.set_intrinsics_guess(intrinsics_map[rFrame::Type::DEPTH]);
	calibration.add(
		frames_map[rFrame::Type::RGB], 
		frames_map[rFrame::Type::DEPTH], 
		index);
	
	return true;
}

void rCamera::clear()
{
	raw_frames_map[rFrame::Type::DEPTH].clear();
	raw_frames_map[rFrame::Type::RGB].clear();
	raw_frames_map[rFrame::Type::IR].clear();

	frames_map[rFrame::Type::DEPTH].clear();
	frames_map[rFrame::Type::RGB].clear();

	registrations.clear();

	fused_frame.reset();
}

void rCamera::filter_depth(
	const double& lower_bound, 
	const double& upper_bound)
{
	for (frame_iterator it = frames_map[rFrame::Type::DEPTH].begin();
		it != frames_map[rFrame::Type::DEPTH].end();
		++it)
	{
		(*it)->filter(lower_bound, upper_bound);
	}
}

bool rCamera::fuse_frames()
{
	if (frames_map[rFrame::Type::DEPTH].empty())
	{
		return false;
	}

	rInfinitamFusionEngine fusion(
		intrinsics_map[rFrame::Type::RGB], 
		intrinsics_map[rFrame::Type::DEPTH], 
		frames_map[rFrame::Type::RGB].front()->rows(), 
		frames_map[rFrame::Type::DEPTH].front()->columns());

	bool result = add_to_fusion(fusion);
	
	fused_frame = frames_map[rFrame::Type::DEPTH].front();

	//fusion.fuse_frames(fused_frame);

	/*
	cv::Mat mat(fused_frame->rows(), fused_frame->columns(), CV_32FC1, fused_frame->data());
	cv::imshow("matrix", mat / 4096.0f);
	cv::waitKey(0);
	*/
	return result;
}

rCameraMatrix rCamera::get_intrinsics(rFrame::Type type) const
{
	return intrinsics_map.at(type);
}

void rCamera::save_data(std::string path, const int& camera_index)
{
	path += std::string("camera") + std::to_string(camera_index) + std::string("_");

	frame_iterator raw_rgb_it = raw_frames_map[rFrame::Type::RGB].begin();
	frame_iterator reg_rgb_it = frames_map[rFrame::Type::RGB].begin();
	frame_iterator d_it = frames_map[rFrame::Type::DEPTH].begin();

	while (raw_rgb_it != raw_frames_map[rFrame::Type::RGB].end() && 
		reg_rgb_it != frames_map[rFrame::Type::RGB].end() && 
		d_it != frames_map[rFrame::Type::DEPTH].end())
	{
		(*raw_rgb_it)->save(
			path + std::string("raw_rgb_"), 
				raw_rgb_it - raw_frames_map[rFrame::Type::RGB].begin());

		(*reg_rgb_it)->save(
			path + std::string("rgb_"),
				reg_rgb_it - frames_map[rFrame::Type::RGB].begin());

		(*d_it)->save(
			path + std::string("depth_"),
				d_it - frames_map[rFrame::Type::DEPTH].begin());

		++raw_rgb_it;
		++reg_rgb_it;
		++d_it;
	}
}

bool rCamera::add_to_fusion(rFusionEngine& fusion) const
{
	if (frames_map.at(rFrame::Type::RGB).empty() || frames_map.at(rFrame::Type::DEPTH).empty())
	{
		return false;
	}

	std::vector<std::shared_ptr<rFrame>>::const_iterator r_it =
		frames_map.at(rFrame::Type::RGB).begin();

	std::vector<std::shared_ptr<rFrame>>::const_iterator d_it =
		frames_map.at(rFrame::Type::DEPTH).begin();

	while (r_it != frames_map.at(rFrame::Type::RGB).end() && d_it != frames_map.at(rFrame::Type::DEPTH).end())
	{
		fusion.add_frames(*r_it, *d_it);

		++r_it;
		++d_it;
	}

	return true;
}

void rCamera::set_intrinsics(rCameraMatrix depth, rCameraMatrix rgb)
{
	intrinsics_map[rFrame::Type::DEPTH] = depth;
	intrinsics_map[rFrame::Type::RGB] = rgb;
}

rCamera::~rCamera()
{
}

}