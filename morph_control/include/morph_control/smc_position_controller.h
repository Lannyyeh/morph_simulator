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

#ifndef MORPH_CONTROL_LEE_POSITION_CONTROLLER_H
#define MORPH_CONTROL_LEE_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "morph_control/common.h"
#include "morph_control/parameters.h"

namespace morph_control {

// 是控制器的默认值，一般情况下都会被yaml的配置替换

class SMCPositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SMCPositionControllerParameters()
    :sigma_1(0.04),
    sigma_2(0.01)
    {
    this->lambda_1 << 0.5, 0.0, 0.0,0.0, 0.5, 0.0, 0.0, 0.0, 2.5;
    this->lambda_2 << 0.5, 0.0, 0, 0, 0.5, 0, 0, 0, 2.5;
    this->Kp_1 << 3, 0, 0, 0, 3, 0, 0, 0, 5;
    this->Kp_2 << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.2;
    this->Kphi_1 << 0.6, 0, 0, 0, 0.6, 0, 0, 0, 1;
    this->Kphi_2 << 0.06, 0, 0, 0, 0.06, 0, 0, 0, 0.1;
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  //这里是为了自适应多种无人机构型，比如4轴6轴，x型+型，螺旋桨的角度和转向在yaml配置
  Eigen::Matrix4Xd allocation_matrix_;  
  RotorConfiguration rotor_configuration_; 
  
  //SMC的参数
  Eigen::Matrix<double,3,3> lambda_1, lambda_2, Kp_1, Kp_2, Kphi_1, Kphi_2;
  double sigma_1, sigma_2; 
};

class SMCPositionController {
  public:
    SMCPositionController();
    ~SMCPositionController();
    void InitializeParameters();
    void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

    void SetOdometry(const EigenOdometry& odometry);
    void SetTrajectoryPoint(
      const mav_msgs::EigenTrajectoryPoint& command_trajectory);

    SMCPositionControllerParameters controller_parameters_;
    VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    bool initialized_params_;
    bool controller_active_;

    Eigen::Vector3d normalized_attitude_gain_;
    Eigen::Vector3d normalized_angular_rate_gain_;
    Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

    mav_msgs::EigenTrajectoryPoint command_trajectory_;
    EigenOdometry odometry_;

    void ComputeDesiredForce(Eigen::Vector3d* force,Eigen::Vector3d* body_z,double* thrust) const;
    void ComputeDesiredTorque(const Eigen::Vector3d& force,
                              const Eigen::Vector3d& body_z,
                              const double& thrust,
                              Eigen::Vector3d* torque) const;

    // void ComputeDesiredAngularAcc_Q(const Eigen::Vector3d& acceleration,
    //                               Eigen::Vector3d* angular_acceleration) const;
  };
}

#endif // MORPH_CONTROL_LEE_POSITION_CONTROLLER_H
