#include "rFreenect2Frame.h"

#include "rFreenect2Registration.h"

namespace rec3D
{
rFreenect2Frame::rFreenect2Frame(
	const Type type, 
	const libfreenect2::Frame& frame)
	: rFrame(type)
{
	copy_from(frame);
}

rFreenect2Frame::rFreenect2Frame(
	const libfreenect2::Frame::Type libfreenect2_type, 
	const libfreenect2::Frame& frame)
	: rFrame(frame_type_translation.at(libfreenect2_type))
{
	copy_from(frame);
}

void rFreenect2Frame::add_to(rRegistration& registration)
{
	rFreenect2Registration& this_registration = 
		dynamic_cast<rFreenect2Registration&>(registration);

	this_registration.add_frame(*frame, type);
}

int rFreenect2Frame::rows() const
{
	return frame ? frame->height : 0;
}

int rFreenect2Frame::columns() const
{
	return frame ? frame->width : 0;
}

void rFreenect2Frame::copy_from(const libfreenect2::Frame& frame)
{
	unsigned char* new_data = 
		new unsigned char[
			(frame.bytes_per_pixel) * (frame.width) * (frame.height)];

	std::memcpy(
		new_data, 
		frame.data, 
		(frame.bytes_per_pixel) * (frame.width) * (frame.height));

	this->frame = std::make_unique<libfreenect2::Frame>(
		frame.width,
		frame.height,
		frame.bytes_per_pixel,
		new_data);
}

int rFreenect2Frame::bytes_per_pixel() const
{
	return frame ? frame->bytes_per_pixel : 0;
}

unsigned char* rFreenect2Frame::data() const
{
	return frame ? frame->data : 0;
}

rFreenect2Frame::~rFreenect2Frame()
{
}

const std::map<libfreenect2::Frame::Type, rFrame::Type> 
	rFreenect2Frame::frame_type_translation =
		{ { libfreenect2::Frame::Type::Color, Type::RGB },
		{ libfreenect2::Frame::Type::Depth, Type::DEPTH },
		{ libfreenect2::Frame::Type::Ir, Type::IR } };

}