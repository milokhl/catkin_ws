/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file camera_info_publisher.cc
 * @author Milo Knowles
 * @date 2017-04-19 14:11:19 (Wed)
 */
#include <ROSOutputWrapper.h>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <sophus/se3.hpp>

ROSOutputWrapper::ROSOutputWrapper() :
	pose_publisher_(),
	pose_tf_broadcaster_() {

	pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
}

ROSOutputWrapper::~ROSOutputWrapper() {
}

ROSOutputWrapper::publishPose(SE3 pose) {

	// fill in and publish the pose_msg
	pose_msg_.pose.position.x = pose.translation()[0];
	pose_msg_.pose.position.y = pose.translation()[1];
	pose_msg_.pose.position.z = pose.translation()[2];
	pose_msg_.pose.orientation.x = pose.so3().unit_quaternion().x();
	pose_msg_.pose.orientation.y = pose.so3().unit_quaternion().y();
	pose_msg_.pose.orientation.z = pose.so3().unit_quaternion().z();
	pose_msg_.pose.orientation.w = pose.so3().unit_quaternion().w();

	if (pose_msg_.pose.orientation.w < 0) {
		pose_msg_.pose.orientation.x *= -1;
		pose_msg_.pose.orientation.y *= -1;
		pose_msg_.pose.orientation.z *= -1;
		pose_msg_.pose.orientation.w *= -1;
	}

	pose_msg_.header.stamp = ros::Time(pose->timestamp());
	pose_msg_.header.frame_id = world_frame_;
	pose_publisher_.publish(pose_msg_);


	// fill in and publish the tf_msg
	tf_msg_.transform.translation.x = pose.translation()[0];
	tf_msg_.transform.translation.y = pose.translation()[1];
	tf_msg_.transform.translation.z = pose.translation()[2];
	tf_msg_.transform.rotation.x = pose.so3().unit_quaternion().x();
	tf_msg_.transform.rotation.y = pose.so3().unit_quaternion().y();
	tf_msg_.transform.rotation.z = pose.so3().unit_quaternion().z();
	tf_msg_.transform.rotation.w = pose.so3().unit_quaternion().w();

	tf_msg_.header.stamp = ros::Time(pose->timestamp());
	tf_msg_.header.frame_id = world_frame_;
	tf_msg_.child_frame_id = pose_frame_;
	tf_publisher_.sendTransform(tf_msg_);

}