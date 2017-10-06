/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file camera_info_publisher.cc
 * @author Milo Knowles
 * @date 2017-04-19 14:11:19 (Wed)
 */

LCMOutputWrapper::LCMOutputWrapper() : 
	lcm_(),
	pose_msg_(),
	pose_channel_() {
}

LCMOutputWrapper::~LCMOutputWrapper() {
}

LCMOutputWrapper::publishPose(SE3 pose) {

	// publish the pose as a pose_t message through LCM
	pose_msg_.pos[0] = pose.translation()[0];
	pose_msg_.pos[1] = pose.translation()[1];
	pose_msg_.pos[2] = pose.translation()[2];
	pose_msg_.orientation[0] = pose.so3().unit_quaternion().x();
	pose_msg_.orientation[1] = pose.so3().unit_quaternion().y();
	pose_msg_.orientation[2] = pose.so3().unit_quaternion().z();
	pose_msg_.orientation[3] = pose.so3().unit_quaternion().w();

	if (pose_msg_.orientation[3] < 0) {
		pose_msg_.orientation[0] *= -1;
		pose_msg_.orientation[1] *= -1;
		pose_msg_.orientation[2] *= -1;
		pose_msg_.orientation[3] *= -1;
	}

	pose_msg_.utime = 0; // TODO: change this to the current time
	lcm.publish(pose_channel_, pose_msg_);
}