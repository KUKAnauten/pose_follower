/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Marcus Ebner */

#include <iimoveit/robot_interface.h>

//TODO not only use positions, use speed and accelerations too
//TODO try out using moveIt! planning live

namespace pose_follower {


class PoseFollower : public iimoveit::RobotInterface {
public:
  PoseFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame, double scale_factor, double max_radius)
      : RobotInterface(node_handle, planning_group, base_frame),
        scale_factor_(scale_factor),
        max_radius_(max_radius),
        max_radius2_(max_radius*max_radius) {
    base_pose_.position.x = 0.5;
    base_pose_.position.y = 0.0;
    base_pose_.position.z = 0.6;
    base_pose_.orientation.x = 0.0;
    base_pose_.orientation.y = 1.0;
    base_pose_.orientation.z = 0.0;
    base_pose_.orientation.w = 0.0;
  }

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }

  void registerSubscriberRelative(const std::string& topic) {
    pose_subscriber_ = node_handle_->subscribe(topic, 1, &PoseFollower::poseCallbackRelative, this);
  }

  void registerSubscriberAbsolute(const std::string& topic) {
    pose_subscriber_ = node_handle_->subscribe(topic, 1, &PoseFollower::poseCallbackAbsolute, this);
  }

  void setBasePose(const geometry_msgs::Pose& pose) {
    base_pose_ = pose;
  }

  geometry_msgs::Pose getBasePose() {
    return base_pose_;
  }


private:
  ros::Subscriber pose_subscriber_;
  geometry_msgs::Pose base_pose_;
  double scale_factor_;
  double max_radius_;
  double max_radius2_;

  void poseCallbackRelative(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double x = msg->pose.position.x * scale_factor_;
    double y = msg->pose.position.y * scale_factor_;
    double z = msg->pose.position.z * scale_factor_;
    if (x*x + y*y + z*z <= max_radius2_) {
      geometry_msgs::Pose target_pose = base_pose_;
      target_pose.position.x += x;
      target_pose.position.y += y;
      target_pose.position.z += z;
      publishPoseGoal(target_pose, 0.01);
    }
  }

  void poseCallbackAbsolute(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    publishPoseGoal(msg->pose, 0.01);
  }
};
} // namespace pose_follower

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  pose_follower::PoseFollower pose_follower(&node_handle, "manipulator", "world", 1, 0.15);
  pose_follower.moveToBasePose();
  pose_follower.waitForApproval();
  pose_follower.registerSubscriberRelative(std::string("/poseFromFile/PoseStampedRelative"));
  //pose_follower.registerSubscriberAbsolute(std::string("/poseFromFile/PoseStampedAbsolute"));
  ROS_INFO_NAMED("pose_follower", "Subscribed to pose!");


  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}