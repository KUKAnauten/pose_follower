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
 *   * Neither the name of the author nor the names of its
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
#include <tf/LinearMath/Quaternion.h>

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
    base_pose_.header.frame_id = "iiwa_link_0";
    base_pose_.pose.position.x = 0.5;
    base_pose_.pose.position.y = 0.0;
    base_pose_.pose.position.z = 0.6;
    base_pose_.pose.orientation.x = 0.0;
    base_pose_.pose.orientation.y = 1.0;
    base_pose_.pose.orientation.z = 0.0;
    base_pose_.pose.orientation.w = 0.0;
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

  void setBasePose(const geometry_msgs::PoseStamped& pose) {
    base_pose_ = pose;
  }

  geometry_msgs::PoseStamped getBasePose() {
    return base_pose_;
  }


private:
  ros::Subscriber pose_subscriber_;
  geometry_msgs::PoseStamped base_pose_;
  double scale_factor_;
  double max_radius_;
  double max_radius2_;

  void poseCallbackRelative(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double x = msg->pose.position.x * scale_factor_;
    double y = msg->pose.position.y * scale_factor_;
    double z = msg->pose.position.z * scale_factor_;
    if (x*x + y*y + z*z <= max_radius2_) {
      tf::Quaternion base_quaternion(base_pose_.pose.orientation.x, base_pose_.pose.orientation.y, base_pose_.pose.orientation.z, base_pose_.pose.orientation.w);
      tf::Quaternion next_quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf::Quaternion result_quaternion = next_quaternion * base_quaternion;

      geometry_msgs::PoseStamped target_pose = base_pose_;
      target_pose.header.stamp = ros::Time::now();
      target_pose.pose.position.x += x;
      target_pose.pose.position.y += y;
      target_pose.pose.position.z += z;
      target_pose.pose.orientation.x = result_quaternion.getX();
      target_pose.pose.orientation.y = result_quaternion.getY();
      target_pose.pose.orientation.z = result_quaternion.getZ();
      target_pose.pose.orientation.w = result_quaternion.getW();
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

  pose_follower::PoseFollower pose_follower(&node_handle, "manipulator", "world", 0.05, 0.5);
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
