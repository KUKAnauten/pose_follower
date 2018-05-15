// Node to publish all the relevant data of the pick and place experiment so it can be recorded synchronized via rosbag

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <sstream>
#include <iimoveit/robot_interface.h>
#include <iiwa_msgs/JointPosition.h> 
#include <iiwa_msgs/JointVelocity.h> 

// std_msgs::UInt32 msg_status_word;
// std_msgs::Float64 msg_time;
//std_msgs::Float64 msg_last_time;
//std_msgs::Float64 msg_time_delta;
//double joint_velocity[7];
trajectory_msgs::JointTrajectory msg_joint_command;
//std::vector<double> position_command;
//iiwa_msgs::JointVelocity msg_calculated_velocity;
//iiwa_msgs::JointPosition msg_joint_position;

sensor_msgs::JointState msg_joint_states;
//iiwa_msgs::JointPosition msg_joint_mcs;
geometry_msgs::PoseStamped msg_pose_mcs;
//std_msgs::Float64 msg_gripper_command;
//std_msgs::Float64 msg_uncoupling_command;


// void statusCallback (const std_msgs::UInt32::ConstPtr& msg) {
//   msg_status_word = *msg;  
// }

void jointStatesCallback (const sensor_msgs::JointState::ConstPtr& msg) {
  msg_joint_states = *msg;
}

void jointCommandCallback (const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  msg_joint_command = *msg;
}

// void jointMcsCallback (const iiwa_msgs::JointPosition::ConstPtr& msg) {
//   msg_joint_mcs = *msg;
// }

void poseMcsCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
  msg_pose_mcs = *msg;  
}

// void gripperCommandCallback (const std_msgs::Float64::ConstPtr& msg) {
//   msg_gripper_command = *msg;  
// }

// void uncouplingCommandCallback (const std_msgs::Float64::ConstPtr& msg) {
//   msg_uncoupling_command = *msg;  
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Subscriber	
  // ros::Subscriber status_sub = node_handle.subscribe("/iiwa/status_word", 1, statusCallback);
  ros::Subscriber joint_command_sub = node_handle.subscribe("/iiwa/PositionJointInterface_trajectory_controller/command", 1, jointCommandCallback);
  ros::Subscriber joint_states_sub = node_handle.subscribe("/iiwa/joint_states", 1, jointStatesCallback);
  //ros::Subscriber joint_mcs_sub = node_handle.subscribe("/iiwa/jointAnglesFromUDP/JointPosition", 1, jointMcsCallback);
  //ros::Subscriber pose_mcs_sub = node_handle.subscribe("/iiwa/poseFromUDP/PoseStamped", 1, poseMcsCallback);
  ros::Subscriber pose_mcs_sub = node_handle.subscribe("/iiwa/poseFromFile/PoseStampedRelative", 1, poseMcsCallback);
  //ros::Subscriber gripper_sub = node_handle.subscribe("/iiwa/gripperCommandFromUDP/GripperCommand", 1, gripperCommandCallback);
  //ros::Subscriber uncoupling_sub = node_handle.subscribe("/iiwa/uncouplingCommandFromUDP/UncouplingCommand", 1, uncouplingCommandCallback);

  // Publisher
  // ros::Publisher status_pub = node_handle.advertise<std_msgs::UInt32>("published_status", 10);
  ros::Publisher joint_command_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("published_joint_command", 10);
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("published_pose", 10);
  ros::Publisher pose_mcs_pub = node_handle.advertise<geometry_msgs::PoseStamped>("published_pose_mcs", 10);  
  //ros::Publisher joint_mcs_pub = node_handle.advertise<iiwa_msgs::JointPosition>("published_joints_mcs", 10);
  ros::Publisher joint_states_pub = node_handle.advertise<sensor_msgs::JointState>("published_joint_states", 10);
  //ros::Publisher gripper_command_pub = node_handle.advertise<std_msgs::Float64>("published_gripper_command", 10);
  //ros::Publisher uncoupling_command_pub = node_handle.advertise<std_msgs::Float64>("published_uncoupling_command", 10);

  iimoveit::RobotInterface ri_object(&node_handle, "manipulator", "world");

  ros::Rate loop_rate(50);
  geometry_msgs::PoseStamped msg_pose;
  
  while (ros::ok())
  {
    // Alternatively subscriber to the topic /iiwa/state/CartesianPose (only when ROSSmartServo Application is running)
    msg_pose = ri_object.getPose(std::string("iiwa_link_ee"));

    pose_pub.publish(msg_pose);
    pose_mcs_pub.publish(msg_pose_mcs);
    joint_states_pub.publish(msg_joint_states);
    joint_command_pub.publish(msg_joint_command);
    // joint_mcs_pub.publish(msg_joint_mcs);
    // gripper_command_pub.publish(msg_gripper_command);
    // uncoupling_command_pub.publish(msg_uncoupling_command);

    loop_rate.sleep();
  }
	ros::shutdown();
  return 0;
}
