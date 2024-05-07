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

#include "morph_control/smc_position_controller.h"
#include "cmath"
#include "Tools/UavMath.h"

namespace morph_control {

double sq(double num)
{
    return num * num;
}

Eigen::Matrix<double,3,1> sat(Eigen::Matrix<double,3,1> x)
    {
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (x(i,0) > 1) x(i,0) = 1;
        else if (x(i,0) < -1) x(i,0) = -1;
    }
    return x;
    }

Eigen::Matrix<double,3,1> sign(Eigen::Matrix<double,3,1> x)
    {
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (x(i,0) > 0) x(i,0) = 1;
        else if (x(i,0) < 0) x(i,0) = -1;
    }
    return x;
    }

SMCPositionController::SMCPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

SMCPositionController::~SMCPositionController() {}

void SMCPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  // normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
  //     * vehicle_parameters_.inertia_.inverse();
  // // To make the tuning independent of the inertia matrix we divide here.
  // normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
  //     * vehicle_parameters_.inertia_.inverse();

  // Eigen::Matrix4d I;
  // I.setZero();
  // I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  // I(3, 3) = 1;
  // angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  // 在本项目中，前面只得到了期望的角速度和期望力，是最后在算分配矩阵求逆的同时，才把inertia考虑进去的。如果套用我们的smc算法直接得到期望力和力矩，这里应该得改，应该是直接改成不乘吧？
  // 另外，这边求逆有考虑不是方阵的情况，比如六轴，所以用的是广义解，大概
  // 分配矩阵的顺序：xyz力矩，最后是升力T
  // angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
  //     * (controller_parameters_.allocation_matrix_
  //     * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
    angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();
  initialized_params_ = true;
}

void SMCPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  // 计算期望加速度->直接计算期望力和机身朝向
  Eigen::Vector3d force;
  Eigen::Vector3d body_z;
  double thrust;
  ComputeDesiredForce(&force,&body_z,&thrust);

  // 计算期望角加速度->直接计算期望力矩
  Eigen::Vector3d torque;
  ComputeDesiredTorque(force, body_z, thrust, &torque);
  
  // 组合角加速度和油门输出
  Eigen::Vector4d torque_thrust;
  torque_thrust.block<3, 1>(0, 0) = torque;
  torque_thrust(3) = thrust;

  // angular_acc_to_rotor_velocities_： 控制分配矩阵
  *rotor_velocities = angular_acc_to_rotor_velocities_ * torque_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void SMCPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void SMCPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void SMCPositionController::ComputeDesiredForce(Eigen::Vector3d* force,Eigen::Vector3d* body_z,double* thrust) const {
  assert(force);

  // Calculate e and edot
  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;
  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  // Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  Eigen::Matrix<double,3,1> g_vector(0,0,vehicle_parameters_.gravity_);

  // *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
  //     + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
  //     - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;

  // Calculate sliding items
  Eigen::Matrix<double,3,1> s_1 = velocity_error + controller_parameters_.lambda_1 * position_error;
  Eigen::Matrix<double,3,1> delta_1 = s_1 - controller_parameters_.sigma_1 * sat(s_1/controller_parameters_.sigma_1);

  // Other items
  Eigen::Matrix<double,3,1> p_r_diff2 = command_trajectory_.acceleration_W - controller_parameters_.lambda_1*velocity_error;

  // Calculate desired force
  if (controller_parameters_.sigma_1 == 0)
    *force = vehicle_parameters_.mass_*(g_vector+p_r_diff2) - controller_parameters_.Kp_1*delta_1 - controller_parameters_.Kp_2*sign(s_1);
  else
    *force = vehicle_parameters_.mass_*(g_vector+p_r_diff2) - controller_parameters_.Kp_1*delta_1 - controller_parameters_.Kp_2*sat(s_1/controller_parameters_.sigma_1);

  // Calculate desired body_z
  Eigen::Vector3d force_copy = *force; // just in case
  Eigen::Vector3d body_z_copy;
  body_z_copy = force_copy.normalized();
  Eigen::Matrix<double,3,1> world_z = create_vector_3d(0.0,0.0,1.0);
  double cos_angle = body_z_copy.dot(world_z);
  double angle = _min(acos(cos_angle), deg2rad(double(45)));
  Eigen::Matrix<double,3,1> rejection = body_z_copy - (cos_angle * world_z);
  body_z_copy = cos(angle)*world_z + sin(angle)*rejection.normalized();
  body_z_copy.normalize();
  *body_z = body_z_copy;

  // Calculate thrust
  *thrust = force_copy(2) / (body_z->dot(world_z));
}


void SMCPositionController::ComputeDesiredTorque(const Eigen::Vector3d& force,
                                                  const Eigen::Vector3d& body_z,
                                                  const double& thrust, 
                                                  Eigen::Vector3d* torque) const {
  assert(torque);

  Eigen::Matrix3d R_IB = odometry_.orientation.toRotationMatrix();
  Eigen::Matrix<double,3,1> rpy_cache = R2rpy(R_IB);
  double yaw_d = command_trajectory_.getYaw();
  
  Eigen::Matrix<double,3,1> world_z = create_vector_3d(0.0,0.0,1.0);

  Eigen::Matrix<double,3,1> tmp_y(-sin(yaw_d), cos(yaw_d), 0.0);
  Eigen::Matrix<double,3,1> body_x = tmp_y.cross(body_z);
  body_x.normalize();
  Eigen::Matrix<double,3,1> body_y = body_z.cross(body_x);
  body_y.normalize();

  Eigen::Matrix<double,3,3> R_IB_des;
  R_IB_des << body_x, body_y, body_z;

  Eigen::Matrix<double,3,1> rpy_sp; // 记得把uav math包加入引用路径
  rpy_sp = R2rpy(R_IB_des);

  double dot_f = vehicle_parameters_.mass_ * body_z.dot(command_trajectory_.jerk_W);
  Eigen::Matrix<double,3,1> h_omega = (vehicle_parameters_.mass_*command_trajectory_.jerk_W - dot_f*body_z) / thrust;
  Eigen::Matrix<double,3,1> w_IB_sp(-h_omega.dot(body_y), h_omega.dot(body_x), command_trajectory_.getYawRate()*world_z.dot(body_z));
  // Calculate euler angle speed set point in present body coordinate
  Eigen::Matrix<double,3,1> rpy_speed_sp;
  rpy_speed_sp = compute_W(rpy_cache) * R_IB.transpose() * w_IB_sp; // 记得把uav math包加入引用路径
  
  double ddot_f = vehicle_parameters_.mass_ * body_z.dot(command_trajectory_.snap_W);
  Eigen::Matrix<double,3,1> h_alpha = (vehicle_parameters_.mass_*command_trajectory_.snap_W 
                                        - ddot_f*body_z - dot_f*w_IB_sp.cross(body_z) 
                                        - vehicle_parameters_.mass_*w_IB_sp.cross(command_trajectory_.jerk_W)) / thrust;
  Eigen::Matrix<double,3,1> ang_acc_IB_sp(-h_alpha.dot(body_y), h_alpha.dot(body_x), command_trajectory_.getYawAcc()*world_z.dot(body_z));
  Eigen::Matrix<double,3,1> ang_acc_BB_sp = R_IB.transpose() * ang_acc_IB_sp;
  // Calculate euler angle acceleration set point in present body coordinate
  Eigen::Matrix<double,3,1> rpy_acc_sp;
  Eigen::Matrix<double,3,1> w_cache =  odometry_.angular_velocity;
  rpy_acc_sp = compute_rpy_acc(rpy_cache, w_cache, ang_acc_BB_sp);

  // rpy_cache,rpy_sp
  // w_cache,rpy_speed_sp
  // ,rpy_acc_sp


  Eigen::Matrix<double,3,1> rpy_speed_cache = compute_W(rpy_cache)*w_cache;
  double r = rpy_cache(0,0), p = rpy_cache(1,0), y = rpy_cache(2,0);
  double r_diff = rpy_speed_cache(0,0), p_diff = rpy_speed_cache(1,0), y_diff = rpy_speed_cache(2,0);

  // Calculate error
  Eigen::Matrix<double,3,1> e_2 = rpy_cache - rpy_sp;
  Eigen::Matrix<double,3,1> e_2_diff = rpy_speed_cache - rpy_speed_sp; 
  // Calculate sliding items
  Eigen::Matrix<double,3,1> s_2 = e_2_diff + controller_parameters_.lambda_2 * e_2;
  Eigen::Matrix<double,3,1> delta_2 = s_2 - controller_parameters_.sigma_2 * sat(s_2/controller_parameters_.sigma_2);
  // Other items
  Eigen::Matrix<double,3,1> phi_r_diff = rpy_speed_sp - controller_parameters_.lambda_2*e_2;
  Eigen::Matrix<double,3,1> phi_r_diff2 = rpy_acc_sp - controller_parameters_.lambda_2*e_2_diff;
  Eigen::Matrix<double,3,3> Y;

  Y << (phi_r_diff2(0,0) - phi_r_diff2(2,0)*sin(p) - phi_r_diff(1,0)*y_diff*cos(p)), 
    (phi_r_diff(1,0)*(p_diff*cos(r)*sin(r) - y_diff*cos(p)*sq(cos(r)) + y_diff*cos(p)*sq(sin(r))) - phi_r_diff(2,0)*y_diff*cos(r)*sq(cos(p))*sin(r)), 
    (phi_r_diff(2,0)*y_diff*cos(r)*sq(cos(p))*sin(r) - phi_r_diff(1,0)*(p_diff*cos(r)*sin(r) - y_diff*cos(p)*sq(cos(r)) + y_diff*cos(p)*sq(sin(r)))),
    (phi_r_diff(0,0)*y_diff*cos(p) - phi_r_diff(2,0)*y_diff*cos(p)*sin(p)), 
    (phi_r_diff2(1,0)*sq(cos(r)) - phi_r_diff(0,0)*(p_diff*cos(r)*sin(r) - y_diff*cos(p)*sq(cos(r)) + y_diff*cos(p)*sq(sin(r))) + phi_r_diff2(2,0)*cos(r)*cos(p)*sin(r) - phi_r_diff(1,0)*r_diff*cos(r)*sin(r) + phi_r_diff(2,0)*y_diff*cos(p)*sq(sin(r))*sin(p)), 
    (phi_r_diff(0,0)*(p_diff*cos(r)*sin(r) - y_diff*cos(p)*sq(cos(r)) + y_diff*cos(p)*sq(sin(r))) - phi_r_diff2(2,0)*cos(r)*cos(p)*sin(r) + phi_r_diff(1,0)*r_diff*cos(r)*sin(r) + phi_r_diff(2,0)*y_diff*sq(cos(r))*cos(p)*sin(p)),
    (phi_r_diff2(2,0)*sq(sin(p))- phi_r_diff2(0,0)*sin(p) - phi_r_diff(0,0)*p_diff*cos(p) + phi_r_diff(1,0)*y_diff*cos(p)*sin(p) + phi_r_diff(2,0)*p_diff*cos(p)*sin(p)), 
    (phi_r_diff(2,0)*(r_diff*cos(r)*sq(cos(p))*sin(r) - p_diff*cos(p)*sq(sin(r))*sin(p)) - phi_r_diff(1,0)*(r_diff*cos(p)*sq(sin(r)) - r_diff*sq(cos(r))*cos(p) + p_diff*cos(r)*sin(r)*sin(p) + y_diff*cos(p)*sq(sin(r))*sin(p)) + phi_r_diff2(2,0)*sq(cos(p))*sq(sin(r)) + phi_r_diff2(1,0)*cos(r)*cos(p)*sin(r) - phi_r_diff(0,0)*y_diff*cos(r)*sin(r)*sin(p)), 
    (phi_r_diff2(2,0)*sq(cos(r))*sq(cos(p)) - phi_r_diff(2,0)*(p_diff*sin(p)*sq(cos(r))*cos(p) + r_diff*sin(r)*cos(r)*sq(cos(p))) - phi_r_diff(1,0)*(r_diff*cos(p)*sq(cos(r)) - p_diff*sin(p)*cos(r)*sin(r) + y_diff*cos(p)*sin(p)*cos(r) - r_diff*cos(p)*sq(sin(r))) - phi_r_diff2(1,0)*cos(r)*cos(p)*sin(r) + phi_r_diff(0,0)*y_diff*cos(r)*sin(r)*sin(p));

  Eigen::Matrix<double,3,3> Q = compute_Q(rpy_cache);
  Eigen::Matrix<double,3,1> bb;
  bb(0,0)=vehicle_parameters_.inertia_(0,0);
  bb(1,1)=vehicle_parameters_.inertia_(1,1);
  bb(2,2)=vehicle_parameters_.inertia_(2,2);

  if (controller_parameters_.sigma_2 == 0)
    *torque = Q.transpose().inverse() * (Y*bb - controller_parameters_.Kphi_1*delta_2 - controller_parameters_.Kphi_2*sign(s_2));
  else
    *torque = Q.transpose().inverse() * (Y*bb - controller_parameters_.Kphi_1*delta_2 - controller_parameters_.Kphi_2*sat(s_2/controller_parameters_.sigma_2));

} 

}
