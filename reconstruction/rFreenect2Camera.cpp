#include "rFreenect2Camera.h"

#include "rFreenect2Frame.h"
#include "rFreenect2Registration.h"

//#include <opencv2\core\core.hpp>
//#include <opencv2\imgproc\imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

namespace rec3D
{

const int MAXFRAMES = 20;

rFreenect2Camera::rFreenect2Camera(
	libfreenect2::Freenect2Device* device, 
	const int& id)
	: rCamera(id),
	f2device(device),
	listener(nullptr),
	running(false)
{
}

bool rFreenect2Camera::capture(const int& n_frames, rFrame::Type type)
{
	int framecount = 0;

	if (!running)
	{
		running = start();
	}

	if (type == rFrame::Type::RGBD)
	{
		bool result_rgb = capture(n_frames, rFrame::Type::RGB);
		bool result_depth = capture(n_frames, rFrame::Type::DEPTH);
		bool result_ir = capture(n_frames, rFrame::Type::IR);

		running = !stop();
		
		return result_rgb && result_depth && result_ir;
	}

	if (registrations.size() != n_frames)
	{
		registrations.resize(
			n_frames,
			std::make_unique<rFreenect2Registration>(
				f2device->getIrCameraParams(),
				f2device->getColorCameraParams()));
	}
	
	while (framecount < n_frames)
	{
		std::shared_ptr<rFreenect2Frame> frame = nullptr;

		frame = listener->wait_for_frame(type);

		if (frame == nullptr)
		{
			return false;
		}

		raw_frames_map[type].push_back(frame);

		framecount++;
	}

	return true;
}

void rFreenect2Camera::clear()
{
	rCamera::clear();

	if (listener)
	{
		delete listener;
		listener = nullptr;
	}
}

int rFreenect2Camera::register_rgbd()
{
	int registered_frames = 0;
	int n_frames = raw_frames_map[rFrame::Type::RGB].size();

	if (n_frames != raw_frames_map[rFrame::Type::DEPTH].size())
	{
		return 0;
	}

	while (registered_frames < n_frames)
	{
		(raw_frames_map[rFrame::Type::DEPTH][registered_frames])->
			add_to(*registrations[registered_frames]);
		(raw_frames_map[rFrame::Type::RGB][registered_frames])->
			add_to(*registrations[registered_frames]);

		registrations[registered_frames]->apply();

		frames_map[rFrame::Type::RGB].push_back(
			registrations[registered_frames]->registered_rgb());
		frames_map[rFrame::Type::DEPTH].push_back(
			registrations[registered_frames]->undistorted_depth());

		registered_frames++;
	}

	registered = true;
	
	/*
	int i = 0;
	for (frame_iterator it = frames_map[rFrame::Type::RGB].begin(); it != frames_map[rFrame::Type::RGB].end(); ++it)
	{
		cv::Mat frame((*it)->rows(), (*it)->columns(), CV_8UC4, (*it)->data());
		cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
		cv::imwrite(std::string("../data/rgb_frame") + std::to_string(i) + std::string(".jpg"), frame);
		i++;
	}

	i = 0;
	for (frame_iterator it = frames_map[rFrame::Type::DEPTH].begin(); it != frames_map[rFrame::Type::DEPTH].end(); ++it)
	{
		cv::Mat frame((*it)->rows(), (*it)->columns(), CV_32FC1, (*it)->data());
		cv::Mat frame_8UC1;
		frame = frame / 4096.0f;
		frame.convertTo(frame_8UC1, CV_8UC1, 255);
		cv::imwrite(std::string("../data/depth_frame") + std::to_string(i) + std::string(".jpg"), frame_8UC1);
		i++;
	}*/

	return registered_frames;
}

std::string rFreenect2Camera::name() const
{
	return std::string("camera_") + std::to_string(index);
}

bool rFreenect2Camera::to_point_cloud(
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) const
{	
	point_cloud.clear();
	return registrations.back()->to_point_cloud(point_cloud);
}

rFreenect2Camera::~rFreenect2Camera()
{
	if (listener)
	{
		delete listener;
	}
}

bool rFreenect2Camera::start()
{
	listener = new rFreenect2FrameListener(MAXFRAMES);

	f2device->setColorFrameListener(listener);
	f2device->setIrAndDepthFrameListener(listener);

	bool success = f2device->start();
	
	this->set_intrinsics(
		f2device->getIrCameraParams(),
		f2device->getColorCameraParams());

	return success;
}


bool rFreenect2Camera::stop()
{
	bool success = f2device->stop();

	return success;
}

}