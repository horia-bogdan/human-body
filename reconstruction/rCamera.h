//a camera class that can capture and store color and depth frames

#ifndef RCAMERA_H
#define RCAMERA_H

#include "rCameraMatrix.h"
#include "rFrame.h"

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>

#include <map>
#include <memory>
#include <vector>

namespace rec3D 
{
class rCalibration;
class rFusionEngine;
class rRegistration;
class rTransformationMatrix;

class rCamera
{
public:
	typedef std::vector<std::shared_ptr<rFrame>>::iterator frame_iterator;

	virtual bool add_to_calibration(rCalibration&);

	virtual bool capture(const int& n_frames, rFrame::Type type) = 0;

	virtual void clear();

	virtual void filter_depth(
		const double& lower_bound, 
		const double& upper_bound);

	virtual bool fuse_frames();

	virtual rCameraMatrix get_intrinsics(rFrame::Type type) const;
	
	virtual std::string name() const = 0;

	virtual void save_data(std::string path, const int& camera_index);

	virtual int register_rgbd() = 0;

	virtual bool to_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>&) const = 0;
		
	virtual ~rCamera();

protected:
	virtual bool add_to_fusion(rFusionEngine&) const;

	virtual void set_intrinsics(rCameraMatrix rgb, rCameraMatrix depth);
	
	virtual bool start() = 0;

	virtual bool stop() = 0;

	rCamera(const int& index);
	
	std::map <rFrame::Type, std::vector<std::shared_ptr<rFrame>>> frames_map;

	std::map <rFrame::Type, std::vector<std::shared_ptr<rFrame>>> raw_frames_map;
	
	std::map <rFrame::Type, rCameraMatrix> intrinsics_map;

	std::shared_ptr<rFrame> fused_frame;
	
	std::vector<std::shared_ptr<rRegistration>> registrations;

	bool calibrated, registered;

	int index;
};
}

#endif
