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

#include <std_msgs/Float32.h>
#include <iimoveit/robot_interface.h>

//TODO not only use positions, use speed and accelerations too

namespace sinus_follower {


class SinFollower : public iimoveit::RobotInterface {
public:
  SinFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      : RobotInterface(node_handle, planning_group, base_frame) {
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

  void registerSubscriber() {
    sinus_subscriber_ = node_handle_->subscribe("/sinus", 1, &SinFollower::sinusCallback, this);
  }

  void publishYGoal(double addition, double duration) {
    geometry_msgs::Pose target_pose = base_pose_;
    target_pose.position.y += addition;
    robot_state_.setFromIK(joint_model_group_, target_pose);
    trajectory_msgs::JointTrajectoryPoint  trajectory_point;
    robot_state_.copyJointGroupPositions(joint_model_group_, trajectory_point.positions);
    trajectory_point.time_from_start = ros::Duration(duration);

    trajectory_msgs::JointTrajectory single_point_trajectory;
    single_point_trajectory.joint_names = joint_names_;
    single_point_trajectory.points.push_back(trajectory_point);

    trajectory_publisher_.publish(single_point_trajectory);
  }

private:
  ros::Subscriber sinus_subscriber_;
  geometry_msgs::Pose base_pose_;

  void sinusCallback(const std_msgs::Float32::ConstPtr& msg) {
    publishYGoal(0.1 * msg->data, 0.01);
  }
};
} // namespace sinus_follower

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_sintest");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sinus_follower::SinFollower sinus_follower(&node_handle, "manipulator", "world");
  sinus_follower.moveToBasePose();
  sinus_follower.waitForApproval();
  sinus_follower.registerSubscriber();
  ROS_INFO_NAMED("iiwa_test", "Subscribed to sinus!");


  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}