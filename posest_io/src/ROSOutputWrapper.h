/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file camera_info_publisher.cc
 * @author Milo Knowles
 * @date 2017-04-19 14:11:19 (Wed)
 */

#pragma once

#include <posest/OutputWrapper.h>

#include <ros/ros.h>
#include <tf/tf_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"


namespace posest
{

class ROSOutputWrapper : public OutputWrapper
{

public:

	ROSOutputWrapper() {};
	~ROSOutputWrapper() {};

	// publishes the pose on /tf and as a PoseStamped msg
	virtual void publishPose(SE3 pose) {};

private:
	std::string pose_topic_;
	geometry_msgs::PoseStamped pose_msg_;
	ros::Publisher pose_publisher_;

	std::string pose_frame_;
	std::string world_frame_;
	geometry_msgs::TransformStamped tf_msg_;
	tf::TransformBroadcaster pose_tf_broadcaster_;

	ros::NodeHandle nh_;
}

} // namespace posest