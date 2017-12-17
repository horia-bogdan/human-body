#include "rInfinitamFusionEngine.h"

#include "rCameraMatrix.h"
#include "rFrame.h"

namespace rec3D
{

rInfinitamFusionEngine::rInfinitamFusionEngine(
	const rCameraMatrix& rgb_intrinsics, 
	const rCameraMatrix& depth_intrinsics,
	const int& rows,
	const int& columns)
	: rFusionEngine(rows, columns)
{	
	settings = new ITMLibSettings();
	calib = new ITMRGBDCalib();
	settings->deviceType = ITMLibSettings::DeviceType::DEVICE_CPU;

	set_intrinsics(
		rgb_intrinsics,
		calib->intrinsics_rgb);

	set_intrinsics(
		depth_intrinsics,
		calib->intrinsics_d);

	Matrix4f mat;
	mat.setIdentity();
	calib->trafo_rgb_to_depth.SetFrom(mat);

	calib->disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
	calib->disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);

	engine = 
		std::make_unique<ITMLib::Engine::ITMMainEngine>(
			settings, 
			calib, 
			Vector2i(rows, columns));
}

bool rInfinitamFusionEngine::fuse_frames(std::shared_ptr<rFrame> out_frame)
{
	if (rgb_frames.empty() || depth_frames.empty())
	{
		return false;
	}

	std::vector<std::shared_ptr<rFrame>>::const_iterator r_it = rgb_frames.begin();
	std::vector<std::shared_ptr<rFrame>>::const_iterator d_it = depth_frames.begin();

	ITMUChar4Image* rgb_image = 0;
	ITMFloatImage* depth_image = 0;
	ITMShortImage* depth_short = 0;

	while (r_it != rgb_frames.end() && d_it != depth_frames.end())
	{
		rgb_image = convert_data<ITMUChar4Image>((*r_it)->data());
		depth_image = convert_data<ITMFloatImage>((*d_it)->data());

		cv::Mat depth;
		cv::Mat(rows, columns, CV_32FC1, (*d_it)->data()).copyTo(depth);
		depth.convertTo(depth, CV_16S);
		depth_short = convert_data<ITMShortImage>(depth.data);

		engine->ProcessFrame(rgb_image, depth_short);
		
		++r_it;
		++d_it;
	}

	ITMFloatImage* fused = engine->GetView()->depth;
	unsigned char* data = 
		reinterpret_cast<unsigned char*>(fused->GetData(MEMORYDEVICE_CPU));
	
	if (fused_frame)
	{
		//fused_frame->set_from(
		//	rFrame::DEPTH,
		//	(fused->noDims).width,
		//	(fused->noDims).height,
		//	data);
	}
	
	if(rgb_image) delete rgb_image;
	if(depth_image) delete depth_image;
	if(depth_short) delete depth_short;

	return true;
}

rInfinitamFusionEngine::~rInfinitamFusionEngine()
{
}

void rInfinitamFusionEngine::set_intrinsics(
	const rCameraMatrix& r_intrinsics,
	ITMLib::Objects::ITMIntrinsics& itm_intrinsics)
{
	itm_intrinsics.SetFrom(
		r_intrinsics.at(0, 0),
		r_intrinsics.at(1, 1),
		r_intrinsics.at(0, 2),
		r_intrinsics.at(1, 2),
		rows,
		columns);
}

template<class T> T* 
	rInfinitamFusionEngine::convert_data(
		const unsigned char* raw_data) const
{
	T* image = 
		new T(Vector2i(rows, columns), true, false);

	unsigned char* new_data = 
		reinterpret_cast<unsigned char*>(image->GetData(MEMORYDEVICE_CPU));

	for (int i = 0; i < rows * columns; i++)
	{
		new_data[i] = raw_data[i];
	}

	return image;
}

}