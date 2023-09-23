// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>

#include <mutex>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

//#include <controller_interface/multi_interface_controller.h>

namespace franka_ros_controllers {

class JointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  std::array<double, 7> initial_vel_{};
  std::array<double, 7> prev_d_{};
  std::array<double, 7> vel_d_target_{};
  std::array<double, 7> vel_d_;

  // joint_cmd subscriber
  ros::Subscriber desired_joints_subscriber_;

  double filter_joint_vel_{0.3};
  double target_filter_joint_vel_{0.3};
  double filter_factor_{0.01};

  franka_core_msgs::JointLimits joint_limits_;
  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;

  bool checkVelocityLimits(std::vector<double> velocities);
  void jointVelCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);

};

}  // namespace franka_ros_controllers
