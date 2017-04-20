/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file camera_info_publisher.cc
 * @author Milo Knowles
 * @date 2017-04-19 14:11:19 (Wed)
 */

#pragma once
#include <sophus/se3.hpp>

#include <string>

namespace posest 
{

class OutputWrapper() {
	
public:
	virtual OutputWrapper() {};
	virtual ~OutputWrapper() {};

	virtual void publishPose(SE3 pose) {};
};

} // namespace posest
