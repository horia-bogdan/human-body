#ifndef RINFINITAMFUSIONENGINE_H
#define RINFINITAMFUSIONENGINE_H

#define _CRT_SECURE_NO_DEPRECATE

#include "rFusionEngine.h"

#include <ITMLib\Engine\ITMMainEngine.h>

namespace rec3D
{

class rCameraMatrix;

class rInfinitamFusionEngine : public rFusionEngine
{
public:
	rInfinitamFusionEngine(
		const rCameraMatrix& rgb_intrinsics, 
		const rCameraMatrix& depth_intrinsics, 
		const int& rows,
		const int& columns);

	virtual bool fuse_frames(std::shared_ptr<rFrame> out_frame);

	virtual ~rInfinitamFusionEngine();
	

private:

	void set_intrinsics(
		const rCameraMatrix&,
		ITMIntrinsics& itm_intrinsics);

	template<class T> T* convert_data(
			const unsigned char* raw_data) const;
	
	ITMLibSettings* settings;
	ITMRGBDCalib* calib;
	std::unique_ptr<ITMMainEngine> engine;
};

}

#endif