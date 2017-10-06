/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file camera_info_publisher.cc
 * @author Milo Knowles
 * @date 2017-04-19 14:11:19 (Wed)
 */

#pragma once

#include <posest/OutputWrapper.h>

#include <lcm/lcm-cpp.hpp>
#include <posest/pose_t.hpp>
#include <string>

namespace posest
{

class LCMOutputWrapper : public OutputWrapper
{
public:
	LCMOutputWrapper() {};
	virtual ~LCMOutputWrapper() {};

	virtual void publishPose(SE3 pose) {};

private:
	lcm::LCM lcm_;

	// TODO: should this be a pose_t or a rigid_transform_t.lcm?
	posest::pose_t pose_msg_;
	std::string pose_channel_;
};

} // namespace posest
