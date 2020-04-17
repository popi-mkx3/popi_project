/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

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

#include <xpp_popi/popileg_inverse_kinematics.h>
#include "ros/ros.h"

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>


namespace xpp {


PopilegInverseKinematics::Vector3d
PopilegInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B) const
{
  double q_aile, q_epaule, q_coude; //aile, épaule, coude
  double A, B, C, H; //variables intermédiaires pour plus de facilité

  double x_12 = aile_epaule[X];
  double y_12 = aile_epaule[Y];
  double z_12 = aile_epaule[Z];

  double x_23 = epaule_coude[X];
  double y_23 = epaule_coude[Y];
  double z_23 = epaule_coude[Z];

  double x_34 = coude_pied[X];
  double y_34 = coude_pied[Y];
  double z_34 = coude_pied[Z];

  Eigen::Vector3d P; //vecteur position
  Eigen::Matrix3d R;
 
  int erreur = 0 ;
  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  P = ee_pos_B;

  this->compteur = this->compteur + 1 ;
  // compute the HAA angle
  if (P[Y] == -y_12 || P[Y] == y_12) //gestion des singularités
  {
    q_aile = 0 ;
    erreur = 1 ;
  }
  else if (pow(P[Z],2)+pow(P[Y],2)-pow(y_12,2) < 0) //gestion des singularités, dans le repère de l'aile, cylindre interdit d'axe de révolution x et de rayon y_12
    erreur = 2 ;
  else
    q_aile = 2*atan((P[Z]+sqrt(pow(P[Z],2)+pow(P[Y],2)-pow(y_12,2)))/(P[Y]+y_12)) ;


  // compute the HFE angle
  // variables intermédiaires pour plus de compréhension dans les équations
  H = -sin(q_aile)*P[Y] + cos(q_aile)*P[Z] ;
  
  A = 2*(P[X]*z_23 - H*x_23) ;
  B = 2*(-P[X]*x_23 - H*z_23) ;
  C = pow(x_34,2) - pow(P[X],2) - pow(H,2) - pow(x_23,2) - pow(z_23,2) ;

  if (B+C == 0 || pow(A,2) + pow(B,2) - pow(C,2) < 0) //gestion des singularités : si le dénominateur est nul ou le terme sous la racine est négatif, on n'a pas de solution. Implications mathématiques dans le dossier de conception.
    erreur = 3 ;
  else
    q_epaule = 2*atan((A + sqrt(pow(A,2) + pow(B,2) - pow(C,2))) / (B + C)) ;

  // compute the KFE angle
  if (P[X]*cos(q_epaule) + H*sin(q_epaule) - x_23 == 0) // singularité : implications mathématiques dans le dossier de conception.
    erreur = 4 ;
  else
    q_coude = atan2(P[X]*sin(q_epaule) - H*cos(q_epaule) + z_23, P[X]*cos(q_epaule) + H*sin(q_epaule) - x_23) ;

  //vérification des limites géométriques
  EnforceLimits(q_aile, HAA);
  EnforceLimits(q_epaule, HFE);
  EnforceLimits(q_coude, KFE);

  if (erreur > 1)
  {
    ROS_INFO(" erreur : [%i] - compteur : [%i]\n Px : [%f] - Py : [%f] - Pz : [%f]\n q_aile : [%f] - q_epaule : [%f] - q_coude : [%f]\n", erreur, this->compteur, P[X], P[Y], P[Z], q_aile, q_epaule, q_coude);
    q_aile = this->q_aile_4;
    q_epaule = this->q_epaule_4;
    q_coude = this->q_coude_4;
  }

  //rotation 4<-3<-2<-1<-0 car on utilise la même fonction pour les quatre pattes
  this->q_aile_4 = this->q_aile_3;
  this->q_epaule_4 = this->q_epaule_3;
  this->q_coude_4 = this->q_coude_3;

  this->q_aile_3 = this->q_aile_2;
  this->q_epaule_3 = this->q_epaule_2;
  this->q_coude_3 = this->q_coude_2;

  this->q_aile_2 = this->q_aile_1;
  this->q_epaule_2 = this->q_epaule_1;
  this->q_coude_2 = this->q_coude_1;

  this->q_aile_1 = q_aile;
  this->q_epaule_1 = q_epaule;
  this->q_coude_1 = q_coude;

  if(isnan(q_aile) || isnan(q_epaule) || isnan(q_coude)) // précaution supplémentaire si l'on a oublié des singularités
    ROS_INFO("q_aile : [%g]\n q_epaule : [%g]\n q_coude : [%g]\n", q_aile, q_epaule, q_coude);
  else
  {
    //ROS_INFO(" erreur : [%i] - compteur : [%i]\n Px : [%f] - Py : [%f] - Pz : [%f]\n q_aile : [%f] - q_epaule : [%f] - q_coude : [%f]\n", erreur, this->compteur, P[X], P[Y], P[Z], q_aile, q_epaule, q_coude);
    //erreur = 0 ;
    return Vector3d(q_aile, q_epaule, q_coude);
  }

}

void
PopilegInverseKinematics::EnforceLimits (double& val, PopiJointID joint) const
{
  // totally exaggerated joint angle limits
  const static double aile_min = -0.4; // -22.92°
  const static double aile_max =  0.4; // 22.92°

  const static double epaule_min = 0;
  const static double epaule_max = M_PI; // 180°

  const static double coude_min = 0.26; // 15 °
  const static double coude_max = 1.92; // 110 °

  // reduced joint angles for optimization
  static const std::map<PopiJointID, double> max_range {
    {HAA, aile_max},
    {HFE, epaule_max},
    {KFE, coude_max}
  };

  // reduced joint angles for optimization
  static const std::map<PopiJointID, double> min_range {
    {HAA, aile_min},
    {HFE, epaule_min},
    {KFE, coude_min}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

} /* namespace xpp */
