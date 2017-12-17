#ifndef RFREENECT2FRAME_H
#define RFREENECT2FRAME_H

#include "rFrame.h"

#include <libfreenect2\frame_listener.hpp>

#include<map>
#include<memory>

namespace rec3D 
{
class rFreenect2Frame :
	public rFrame
{
public:
	rFreenect2Frame(const Type type, const libfreenect2::Frame&);

	rFreenect2Frame(const libfreenect2::Frame::Type, const libfreenect2::Frame&);

	virtual void add_to(rRegistration&);

	virtual int rows() const;
	
	virtual int columns() const;

	virtual void copy_from(const libfreenect2::Frame&);

	virtual int bytes_per_pixel() const;

	virtual unsigned char* data() const;

	virtual ~rFreenect2Frame();
	
	static const std::map<libfreenect2::Frame::Type, Type> frame_type_translation;
		
private:
	std::unique_ptr<libfreenect2::Frame> frame;
};
}

#endif