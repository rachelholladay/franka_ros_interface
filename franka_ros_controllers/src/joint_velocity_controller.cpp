// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_ros_controllers/joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_ros_controllers {

bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/joint_commands", 20, &JointVelocityController::jointVelCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }

  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names)) {
    ROS_ERROR("JointVelocityController: Could not parse joint names");
  }
  if (joint_limits_.joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                     << joint_limits_.joint_names.size() << " instead of 7 names!");
    return false;
  }
  std::map<std::string, double> vel_limit_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_velocity_limit", vel_limit_map) ) {
  ROS_ERROR(
      "JointVelocityController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (vel_limit_map.find(joint_limits_.joint_names[i]) != vel_limit_map.end())
      {
        joint_limits_.velocity.push_back(vel_limit_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("JointVelocityController: Unable to find lower velocity limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_limits_.joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("JointVelocityController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);


  /*auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityController: Could not get state interface from hardware");
    return false;
  }

  /*try {
    auto state_handle = state_interface->getHandle("panda_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "JointVelocityController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityController: Exception getting state handle: " << e.what());
    return false;
  }*/

  return true;
}

void JointVelocityController::starting(const ros::Time& /* time */) {

  for (size_t i = 0; i < 7; ++i) {
    initial_vel_[i] = velocity_joint_handles_[i].getVelocity();
  }
  vel_d_ = initial_vel_;
  prev_d_ = vel_d_;

}

void JointVelocityController::update(const ros::Time& /* time */,
                                     const ros::Duration& period) {

  for (size_t i = 0; i < 7; ++i) {
    velocity_joint_handles_[i].setCommand(vel_d_[i]);
  }
  double filter_val = filter_joint_vel_ * filter_factor_;
  for (size_t i = 0; i < 7; ++i) {
    prev_d_[i] = velocity_joint_handles_[i].getVelocity();
    vel_d_[i] = filter_val * vel_d_target_[i] + (1.0 - filter_val) * vel_d_[i];
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  //filter_joint_vel_ = param_change_filter_ * target_filter_joint_vel_ + (1.0 - param_change_filter_) * filter_joint_vel_;
}

bool JointVelocityController::checkVelocityLimits(std::vector<double> velocities)
{
  // bool retval = true;
  for (size_t i = 0;  i < 7; ++i){
    if (!(abs(velocities[i]) <= joint_limits_.velocity[i])){
      return true;
    }
  }

  return false;
}

void JointVelocityController::jointVelCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

    if (msg->mode == franka_core_msgs::JointCommand::VELOCITY_MODE){
      if (msg->velocity.size() != 7) {
        ROS_ERROR_STREAM(
            "JointVelocityController: Published Commands are not of size 7");
        vel_d_ = prev_d_;
        vel_d_target_ = prev_d_;
      }
      else if (checkVelocityLimits(msg->velocity)) {
         ROS_ERROR_STREAM(
            "JointVelocityController: Commanded velocities are beyond allowed velocity limits.");
        vel_d_ = prev_d_;
        vel_d_target_ = prev_d_;

      }
      else
      {
        std::copy_n(msg->velocity.begin(), 7, vel_d_target_.begin());
      }

    }
    // else ROS_ERROR_STREAM("JointVelocityController: Published Command msg are not of JointCommand::Velocity! Dropping message");
}

void JointVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::JointVelocityController,
                       controller_interface::ControllerBase)
