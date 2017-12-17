#include "rFreenect2FrameListener.h"

#include "rFreenect2Frame.h"

namespace rec3D
{

rFreenect2FrameListener::rFreenect2FrameListener(const int& maxframes)
	:framecount_map(
		{{ rFrame::Type::RGB, 0 }, 
		{ rFrame::Type::DEPTH, 0 },
		{ rFrame::Type::IR, 0 } })
{
	this->maxframes = maxframes;
}

bool rFreenect2FrameListener::onNewFrame(
	libfreenect2::Frame::Type type, 
	libfreenect2::Frame *frame)
{
	rFrame::Type our_type = rFreenect2Frame::frame_type_translation.at(type);

	if (framecount_map[our_type] < maxframes)
	{
		std::shared_ptr<rFreenect2Frame> new_frame
			= std::make_shared<rFreenect2Frame>(our_type, *frame);
		delete frame;
		
		std::unique_lock<std::mutex> lock(mutex);
		frames_map[our_type].push(new_frame);
		lock.unlock();
		
		condition_map[our_type].notify_one();

		framecount_map[our_type]++;
	}
	else {
		return false;
	}

	return true;
}

std::shared_ptr<rFreenect2Frame> rFreenect2FrameListener::wait_for_frame(
	rFrame::Type type)
{
	std::unique_lock<std::mutex> lock(mutex);
	
	if (frames_map[type].empty())
	{
		condition_map[type].wait(lock);	
	}

	std::shared_ptr<rFreenect2Frame> current_frame = frames_map[type].front();
	frames_map[type].pop();

	lock.unlock();

	return current_frame;
}


rFreenect2FrameListener::~rFreenect2FrameListener()
{
}

}