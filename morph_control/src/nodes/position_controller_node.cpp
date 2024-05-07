/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "position_controller_node.h"

#include "morph_control/parameters_ros.h"

namespace morph_control{

PositionControllerNode::PositionControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &PositionControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &PositionControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &PositionControllerNode::TimedCommandCallback, this,
                                  true, false);
}

PositionControllerNode::~PositionControllerNode() { }

void PositionControllerNode::InitializeParams() {
  
  GetRosParameter(private_nh_,"lambda_1/x",
                  smc_position_controller_.controller_parameters_.lambda_1(0,0),
                  &smc_position_controller_.controller_parameters_.lambda_1(0,0));
  GetRosParameter(private_nh_,"lambda_1/y",
                  smc_position_controller_.controller_parameters_.lambda_1(1,1),
                  &smc_position_controller_.controller_parameters_.lambda_1(1,1));
  GetRosParameter(private_nh_,"lambda_1/z",
                  smc_position_controller_.controller_parameters_.lambda_1(2,2),
                  &smc_position_controller_.controller_parameters_.lambda_1(2,2));
  GetRosParameter(private_nh_,"lambda_2/x",
                  smc_position_controller_.controller_parameters_.lambda_2(0,0),
                  &smc_position_controller_.controller_parameters_.lambda_2(0,0));
  GetRosParameter(private_nh_,"lambda_2/y",
                  smc_position_controller_.controller_parameters_.lambda_2(1,1),
                  &smc_position_controller_.controller_parameters_.lambda_2(1,1));
  GetRosParameter(private_nh_,"lambda_2/z",
                  smc_position_controller_.controller_parameters_.lambda_2(2,2),
                  &smc_position_controller_.controller_parameters_.lambda_2(2,2));
  GetRosParameter(private_nh_,"Kp_1/x",
                  smc_position_controller_.controller_parameters_.Kp_1(0,0),
                  &smc_position_controller_.controller_parameters_.Kp_1(0,0));
  GetRosParameter(private_nh_,"Kp_1/y",
                  smc_position_controller_.controller_parameters_.Kp_1(1,1),
                  &smc_position_controller_.controller_parameters_.Kp_1(1,1));
  GetRosParameter(private_nh_,"Kp_1/z",
                  smc_position_controller_.controller_parameters_.Kp_1(2,2),
                  &smc_position_controller_.controller_parameters_.Kp_1(2,2));
  GetRosParameter(private_nh_,"Kp_2/x",
                  smc_position_controller_.controller_parameters_.Kp_2(0,0),
                  &smc_position_controller_.controller_parameters_.Kp_2(0,0));
  GetRosParameter(private_nh_,"Kp_2/y",
                  smc_position_controller_.controller_parameters_.Kp_2(1,1),
                  &smc_position_controller_.controller_parameters_.Kp_2(1,1));
  GetRosParameter(private_nh_,"Kp_2/z",
                  smc_position_controller_.controller_parameters_.Kp_2(2,2),
                  &smc_position_controller_.controller_parameters_.Kp_2(2,2));  
  GetRosParameter(private_nh_,"Kphi_1/x",
                  smc_position_controller_.controller_parameters_.Kphi_1(0,0),
                  &smc_position_controller_.controller_parameters_.Kphi_1(0,0));
  GetRosParameter(private_nh_,"Kphi_1/y",
                  smc_position_controller_.controller_parameters_.Kphi_1(1,1),
                  &smc_position_controller_.controller_parameters_.Kphi_1(1,1));
  GetRosParameter(private_nh_,"Kphi_1/z",
                  smc_position_controller_.controller_parameters_.Kphi_1(2,2),
                  &smc_position_controller_.controller_parameters_.Kphi_1(2,2));
  GetRosParameter(private_nh_,"Kphi_2/x",
                  smc_position_controller_.controller_parameters_.Kphi_2(0,0),
                  &smc_position_controller_.controller_parameters_.Kphi_2(0,0));
  GetRosParameter(private_nh_,"Kphi_2/y",
                  smc_position_controller_.controller_parameters_.Kphi_2(1,1),
                  &smc_position_controller_.controller_parameters_.Kphi_2(1,1));
  GetRosParameter(private_nh_,"Kphi_2/z",
                  smc_position_controller_.controller_parameters_.Kphi_2(2,2),
                  &smc_position_controller_.controller_parameters_.Kphi_2(2,2));  
  GetRosParameter(private_nh_,"sigma_1/val",
                  smc_position_controller_.controller_parameters_.sigma_1,
                  &smc_position_controller_.controller_parameters_.sigma_1); 
  GetRosParameter(private_nh_,"sigma_2/val",
                  smc_position_controller_.controller_parameters_.sigma_2,
                  &smc_position_controller_.controller_parameters_.sigma_2); 
  GetVehicleParameters(private_nh_, &smc_position_controller_.vehicle_parameters_);
  smc_position_controller_.InitializeParameters();
}
void PositionControllerNode::Publish() {
}

void PositionControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  smc_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void PositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  smc_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  smc_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("PositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  smc_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  smc_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "smc_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  morph_control::PositionControllerNode position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
