#ifndef RFUSIONENGINE_H
#define RFUSIONENGINE_H

#include <memory>
#include <vector>

namespace rec3D
{

class rFrame;

class rFusionEngine
{
public:
	rFusionEngine(const int& rows, const int& columns);
	
	virtual void add_frames(
		std::shared_ptr<rFrame> depth_frame, 
		std::shared_ptr<rFrame> rgb_frame = 0);

	virtual bool fuse_frames(std::shared_ptr<rFrame> out_frame) = 0;

	virtual ~rFusionEngine();
	
	const int rows, columns;

protected:
	std::vector<std::shared_ptr<rFrame>> depth_frames;
	std::vector<std::shared_ptr<rFrame>> rgb_frames;

	std::shared_ptr<rFrame> fused_frame;
};

}

#endif