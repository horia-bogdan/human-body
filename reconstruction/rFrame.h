#ifndef RFRAME_H
#define RFRAME_H

#include <string>

#include <opencv2\core\core.hpp>

namespace rec3D
{

class rRegistration;

class rFrame 
{
public:
	enum Type
	{
		RGB,
		DEPTH,
		IR,
		RGBD,
		NONE
	};

	rFrame(const Type);
	
	virtual void add_to(rRegistration&) = 0;

	virtual int bytes_per_pixel() const = 0;
	
	virtual int columns() const = 0;
	
	virtual unsigned char* data() const = 0;

	virtual void filter(const double& lower_bound, const double& upper_bound);

	virtual int rows() const = 0;

	virtual void save(std::string path, const int& index) const;

	virtual void to_opencv_mat(cv::Mat& matrix, bool to_image = false) const;

	virtual ~rFrame();

	Type type;
};

}

#endif

