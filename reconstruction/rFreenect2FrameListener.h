#ifndef RFREENECT2FRAMELISTENER_H
#define RFREENECT2FRAMELISTENER_H

#include "rFrame.h"

#include <libfreenect2\frame_listener.hpp>

#include <condition_variable>
#include <queue>
#include <map>
#include <mutex>

namespace rec3D
{

class rFreenect2Frame;

class rFreenect2FrameListener :
	public libfreenect2::FrameListener
{
public:
	rFreenect2FrameListener(const int& maxframes);

	virtual bool onNewFrame(
		libfreenect2::Frame::Type type, 
		libfreenect2::Frame *frame);

	std::shared_ptr<rFreenect2Frame> wait_for_frame(
		rFrame::Type type);

	~rFreenect2FrameListener();

private:	
	int maxframes;

	std::map<rFrame::Type, int> framecount_map;

	std::map<rFrame::Type, std::condition_variable> condition_map;

	std::map<
		rFrame::Type, 
		std::queue<std::shared_ptr<rFreenect2Frame>>>
			frames_map;
	
	std::mutex mutex;
};

}

#endif