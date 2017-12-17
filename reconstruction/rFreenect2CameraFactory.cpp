#include "rFreenect2CameraFactory.h"

#include "rFreenect2Camera.h"

namespace rec3D
{

rFreenect2CameraFactory::rFreenect2CameraFactory()
	: device_index(0)
{
	n_devices = freenect2.enumerateDevices();
}

std::unique_ptr<rCamera> rFreenect2CameraFactory::find_camera()
{	
	if (device_index < n_devices)
	{
		libfreenect2::PacketPipeline* packet_pipeline =
			new libfreenect2::CudaPacketPipeline();

		libfreenect2::Freenect2Device *device =
			freenect2.openDevice(device_index, packet_pipeline);

		device_index++;

		return std::make_unique<rFreenect2Camera> (device, device_index);
	}

	return nullptr;
}

rFreenect2CameraFactory::~rFreenect2CameraFactory()
{
}

}
