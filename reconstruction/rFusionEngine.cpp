#include "rFusionEngine.h"

#include "rFrame.h"

namespace rec3D
{

rFusionEngine::rFusionEngine(const int& rows, const int& columns)
	:rows(rows),
	columns(columns)
{
}

void rFusionEngine::add_frames(
	std::shared_ptr<rFrame> rgb_frame,
	std::shared_ptr<rFrame> depth_frame)
{
	rgb_frames.push_back(rgb_frame);
	depth_frames.push_back(depth_frame);
}

rFusionEngine::~rFusionEngine()
{
}

}
