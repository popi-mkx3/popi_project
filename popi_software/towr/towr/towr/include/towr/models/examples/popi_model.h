/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_POPI_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_POPI_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot Popi.
 */
class PopiKinematicModel : public KinematicModel {
public:
  PopiKinematicModel () : KinematicModel(4)
  {
    //valeurs nominales dans le repère aile
    const double x_nominal_b = -0.0402;
    const double y_nominal_b = 0.1077;
    const double z_nominal_b = -0.4875;

    const double x_ecart_base_aile = 0.3305;
    const double y_ecart_base_aile = 0.175;
    const double z_ecart_base_aile = 0.051;

    nominal_stance_.at(LF) <<  x_nominal_b + x_ecart_base_aile,   y_nominal_b + y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;
    nominal_stance_.at(RF) <<  x_nominal_b + x_ecart_base_aile,  - y_nominal_b - y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;
    nominal_stance_.at(LH) <<  x_nominal_b - x_ecart_base_aile,   y_nominal_b + y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;
    nominal_stance_.at(RH) <<  x_nominal_b - x_ecart_base_aile,  - y_nominal_b - y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;

    max_dev_from_nominal_ << 0.25, 0.20, 0.10;
  }
};

/**
 * @brief The Dynamics of the quadruped robot Popi.
 */
class PopiDynamicModel : public SingleRigidBodyDynamics {
public: // double mass, double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz, int ee_count
  PopiDynamicModel() : SingleRigidBodyDynamics(50.033,
                      0.97314, 3.88974, 4.74716, 0.0, 0.0, 0.0,
                      4) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_POPI_MODEL_H_ */
