/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Popi
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <popi_hw_interface/popi_hw_interface.h>
#include <ros/console.h>
namespace popi_control
{

PopiHWInterface::PopiHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : popi_control::GenericHWInterface(nh, urdf_model)
{
  client_droite = nh.serviceClient<popi_robot::Multi_sensor_read>("right_multi_sensor_server");
  client_gauche = nh.serviceClient<popi_robot::Multi_sensor_read>("left_multi_sensor_server");
  pub = nh.advertise<popi_robot::CmdPosArray>("cmd_mot", 1);
  ros::service::waitForService("right_multi_sensor_server");
  ros::service::waitForService("left_multi_sensor_server");
  //cmd_array.aile_avd=0.0;
  ROS_INFO_NAMED("popi_hw_interface", "PopiHWInterface Ready.");
}

void PopiHWInterface::getJoint()
{
// associe id_joint a nom de joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    
    if (joint_names_[joint_id] == "epauleAVD") epauleAVD=joint_id;
    else if (joint_names_[joint_id] == "epauleAVG") epauleAVG=joint_id;
    else if (joint_names_[joint_id] == "epauleARD") epauleARD=joint_id;
    else if (joint_names_[joint_id] == "epauleARG") epauleARG=joint_id;
     

    else if (joint_names_[joint_id] == "aileAVD") aileAVD=joint_id;
    else if (joint_names_[joint_id] == "aileAVG") aileAVG=joint_id;
    else if (joint_names_[joint_id] == "aileARD") aileARD=joint_id;
    else if (joint_names_[joint_id] == "aileARG") aileARG=joint_id;
     
    else if (joint_names_[joint_id] == "coudeAVD") coudeAVD=joint_id;
    else if (joint_names_[joint_id] == "coudeAVG") coudeAVG=joint_id;
    else if (joint_names_[joint_id] == "coudeARD") coudeARD=joint_id;
    else if (joint_names_[joint_id] == "coudeARG") coudeARG=joint_id;

    else if (joint_names_[joint_id] == "piedAVD" || joint_names_[joint_id] == "piedAVG" || joint_names_[joint_id] == "piedARD" || joint_names_[joint_id] == "piedARG" || joint_names_[joint_id] == "base_flottante");
    else ROS_ERROR("Problemes de nommage voir popi_hw_interface.cpp");

  }//for
}

void PopiHWInterface::read(ros::Duration &elapsed_time)
{
  // ----------------------------------------------------
  //      joint_position_= ROBOT.read_all_positions();
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //cote gauche
  if (client_gauche.call(srv))
  {
    joint_position_[aileAVG]=srv.response.mesure.aile_av;
    joint_position_[aileARG]=srv.response.mesure.aile_ar;
    joint_position_[epauleAVG]=srv.response.mesure.epaule_av;
    joint_position_[epauleARG]=srv.response.mesure.epaule_ar;
    joint_position_[coudeAVG]=srv.response.mesure.coude_av;
    joint_position_[coudeARG]=srv.response.mesure.coude_ar;
  }
  else
  {
    ROS_ERROR("Failed to call left_multi_sensor service");
  }

  //cote droit
  if (client_droite.call(srv))
  {
    joint_position_[aileAVD]=srv.response.mesure.aile_av;
    joint_position_[aileARD]=srv.response.mesure.aile_ar;
    joint_position_[epauleAVD]=srv.response.mesure.epaule_av;
    joint_position_[epauleARD]=srv.response.mesure.epaule_ar;
    joint_position_[coudeAVD]=srv.response.mesure.coude_av;
    joint_position_[coudeARD]=srv.response.mesure.coude_ar;
  }
  else
  {
    ROS_ERROR("Failed to call right_multi_sensor service");
  }

}
  


void PopiHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  
  cmd_array.aile_avd = joint_effort_command_[aileAVD];
  cmd_array.aile_avg = joint_effort_command_[aileAVG];
  cmd_array.aile_ard = joint_effort_command_[aileARD];
  cmd_array.aile_arg = joint_effort_command_[aileARG];

  cmd_array.epaule_avd = joint_effort_command_[epauleAVD];
  cmd_array.epaule_avg = joint_effort_command_[epauleAVG];
  cmd_array.epaule_ard = joint_effort_command_[epauleARD];
  cmd_array.epaule_arg = joint_effort_command_[epauleARG];

  cmd_array.coude_avd = joint_effort_command_[coudeAVD];
  cmd_array.coude_avg = joint_effort_command_[coudeAVG];
  cmd_array.coude_ard = joint_effort_command_[coudeARD];
  cmd_array.coude_arg = joint_effort_command_[coudeARG];

  pub.publish(cmd_array);
  
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
//  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
//    joint_position_[joint_id] += joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void PopiHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
