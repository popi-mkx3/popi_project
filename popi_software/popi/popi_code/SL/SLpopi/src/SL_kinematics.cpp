#include <sl/rcg/eigen_conversion.h>
#include <sl/rcg/rbd_conversion.h>
#include <sl/rcg/joint_status_conversion.h>

#include <iit/robots/popi/declarations.h>
#include <iit/robots/popi/transforms.h>
#include <iit/robots/popi/jacobians.h>
#include <iit/robots/popi/traits.h>
#include <iit/robots/popi/inertia_properties.h>

#include <iit/rbd/utils.h>

#include "robcogen_globals.h"

#include <iostream>

// the system headers
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "mdefs.h"
#include "SL_kinematics.h"
#include "utility_macros.h"

using namespace iit;
using namespace iit::popi;


static JointState q;
static dyn::InertiaProperties inertias;

typedef typename sl::SLtoRobogen<Traits> SLtoRGen;

/*!
 * Original documentation:
 *
 *        computes the m*cog, rotation axis, and local coord.sys. orgin for
 *        every link. This information can be used to compute the COG and
 *        COG jacobians, assuming the mass and center of mass parameters are
 *        properly identified.
 *
 * \param[in]     state   : the state containing th, thd, thdd
 * \param[in]     basec   : the position state of the base
 * \param[in]     baseo   : the orientational state of the base
 * \param[in]     endeff  : the endeffector parameters
 * \param[out]    Xmcog   : array of mass*cog vectors
 * \param[out]    Xaxis   : array of rotation axes
 * \param[out]    Xorigin : array of coord.sys. origin vectors
 * \param[out]    Xlink   : array of link position
 * \param[out]    Ahmat   : homogeneous transformation matrices of each link
 *
 *
 * Marco's notes:
 * I am not really sure what is the difference between Xorigin and Xlink
 */
void linkInformation(
        SL_Jstate *state,
        SL_Cstate *basec,
        SL_quat *baseo,
        SL_endeff *eff,
        double **Xmcog, double **Xaxis, double **Xorigin, double **Xlink,
        double ***Ahmat, double ***Ahmatdof)
{
    // Convenient alias of the global variable
    iit::popi::HomogeneousTransforms& HT = * iit::popi::SL::homogeneousTransforms;
    // Copy the joint status
    SLtoRGen::pos(state, q);
    // Support vector
    Eigen::Matrix<double,4,1> tmp_vec;

    iit::popi::HomogeneousTransforms::MatrixType tmpX_1;
    iit::popi::HomogeneousTransforms::MatrixType tmpX_4;
    iit::popi::HomogeneousTransforms::MatrixType tmpX_7;
    iit::popi::HomogeneousTransforms::MatrixType tmpX_10;
    
    
    // The transform from EpauleAVD to world
    tmpX_1 = SL::world_X_base * HT.fr_base_X_fr_EpauleAVD(q);
    
    sl::copy(Xaxis  [::RF_HAA_JOINT], iit::rbd::Utils::zAxis(tmpX_1) );
    sl::copy(Xorigin[::RF_HAA_JOINT], iit::rbd::Utils::positionVector(tmpX_1) );
    
    sl::copy(Xlink[::EPAULEAVD], iit::rbd::Utils::positionVector(tmpX_1) );
    sl::copy(Ahmat[::EPAULEAVD], tmpX_1);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_EpauleAVD() * inertias.getMass_EpauleAVD();
    tmp_vec(3) = inertias.getMass_EpauleAVD();
    sl::copy(Xmcog[::RF_HAA_JOINT], tmpX_1 * tmp_vec);
    
    // The transform from HJambeAVD to world
    tmpX_1 = tmpX_1 * HT.fr_EpauleAVD_X_fr_HJambeAVD(q);
    
    sl::copy(Xaxis  [::RF_HFE_JOINT], iit::rbd::Utils::zAxis(tmpX_1) );
    sl::copy(Xorigin[::RF_HFE_JOINT], iit::rbd::Utils::positionVector(tmpX_1) );
    
    sl::copy(Xlink[::HJAMBEAVD], iit::rbd::Utils::positionVector(tmpX_1) );
    sl::copy(Ahmat[::HJAMBEAVD], tmpX_1);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_HJambeAVD() * inertias.getMass_HJambeAVD();
    tmp_vec(3) = inertias.getMass_HJambeAVD();
    sl::copy(Xmcog[::RF_HFE_JOINT], tmpX_1 * tmp_vec);
    
    // The transform from BJambeAVD to world
    tmpX_1 = tmpX_1 * HT.fr_HJambeAVD_X_fr_BJambeAVD(q);
    
    sl::copy(Xaxis  [::RF_KFE_JOINT], iit::rbd::Utils::zAxis(tmpX_1) );
    sl::copy(Xorigin[::RF_KFE_JOINT], iit::rbd::Utils::positionVector(tmpX_1) );
    
    sl::copy(Xlink[::BJAMBEAVD], iit::rbd::Utils::positionVector(tmpX_1) );
    sl::copy(Ahmat[::BJAMBEAVD], tmpX_1);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_BJambeAVD() * inertias.getMass_BJambeAVD();
    tmp_vec(3) = inertias.getMass_BJambeAVD();
    sl::copy(Xmcog[::RF_KFE_JOINT], tmpX_1 * tmp_vec);
    
    //TODO  add the code for the endeffector links!!// The transform from EpauleAVG to world
    tmpX_4 = SL::world_X_base * HT.fr_base_X_fr_EpauleAVG(q);
    
    sl::copy(Xaxis  [::LF_HAA_JOINT], iit::rbd::Utils::zAxis(tmpX_4) );
    sl::copy(Xorigin[::LF_HAA_JOINT], iit::rbd::Utils::positionVector(tmpX_4) );
    
    sl::copy(Xlink[::EPAULEAVG], iit::rbd::Utils::positionVector(tmpX_4) );
    sl::copy(Ahmat[::EPAULEAVG], tmpX_4);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_EpauleAVG() * inertias.getMass_EpauleAVG();
    tmp_vec(3) = inertias.getMass_EpauleAVG();
    sl::copy(Xmcog[::LF_HAA_JOINT], tmpX_4 * tmp_vec);
    
    // The transform from HJambeAVG to world
    tmpX_4 = tmpX_4 * HT.fr_EpauleAVG_X_fr_HJambeAVG(q);
    
    sl::copy(Xaxis  [::LF_HFE_JOINT], iit::rbd::Utils::zAxis(tmpX_4) );
    sl::copy(Xorigin[::LF_HFE_JOINT], iit::rbd::Utils::positionVector(tmpX_4) );
    
    sl::copy(Xlink[::HJAMBEAVG], iit::rbd::Utils::positionVector(tmpX_4) );
    sl::copy(Ahmat[::HJAMBEAVG], tmpX_4);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_HJambeAVG() * inertias.getMass_HJambeAVG();
    tmp_vec(3) = inertias.getMass_HJambeAVG();
    sl::copy(Xmcog[::LF_HFE_JOINT], tmpX_4 * tmp_vec);
    
    // The transform from BJambeAVG to world
    tmpX_4 = tmpX_4 * HT.fr_HJambeAVG_X_fr_BJambeAVG(q);
    
    sl::copy(Xaxis  [::LF_KFE_JOINT], iit::rbd::Utils::zAxis(tmpX_4) );
    sl::copy(Xorigin[::LF_KFE_JOINT], iit::rbd::Utils::positionVector(tmpX_4) );
    
    sl::copy(Xlink[::BJAMBEAVG], iit::rbd::Utils::positionVector(tmpX_4) );
    sl::copy(Ahmat[::BJAMBEAVG], tmpX_4);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_BJambeAVG() * inertias.getMass_BJambeAVG();
    tmp_vec(3) = inertias.getMass_BJambeAVG();
    sl::copy(Xmcog[::LF_KFE_JOINT], tmpX_4 * tmp_vec);
    
    //TODO  add the code for the endeffector links!!// The transform from EpauleARD to world
    tmpX_7 = SL::world_X_base * HT.fr_base_X_fr_EpauleARD(q);
    
    sl::copy(Xaxis  [::RH_HAA_JOINT], iit::rbd::Utils::zAxis(tmpX_7) );
    sl::copy(Xorigin[::RH_HAA_JOINT], iit::rbd::Utils::positionVector(tmpX_7) );
    
    sl::copy(Xlink[::EPAULEARD], iit::rbd::Utils::positionVector(tmpX_7) );
    sl::copy(Ahmat[::EPAULEARD], tmpX_7);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_EpauleARD() * inertias.getMass_EpauleARD();
    tmp_vec(3) = inertias.getMass_EpauleARD();
    sl::copy(Xmcog[::RH_HAA_JOINT], tmpX_7 * tmp_vec);
    
    // The transform from HJambeARD to world
    tmpX_7 = tmpX_7 * HT.fr_EpauleARD_X_fr_HJambeARD(q);
    
    sl::copy(Xaxis  [::RH_HFE_JOINT], iit::rbd::Utils::zAxis(tmpX_7) );
    sl::copy(Xorigin[::RH_HFE_JOINT], iit::rbd::Utils::positionVector(tmpX_7) );
    
    sl::copy(Xlink[::HJAMBEARD], iit::rbd::Utils::positionVector(tmpX_7) );
    sl::copy(Ahmat[::HJAMBEARD], tmpX_7);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_HJambeARD() * inertias.getMass_HJambeARD();
    tmp_vec(3) = inertias.getMass_HJambeARD();
    sl::copy(Xmcog[::RH_HFE_JOINT], tmpX_7 * tmp_vec);
    
    // The transform from BJambeARD to world
    tmpX_7 = tmpX_7 * HT.fr_HJambeARD_X_fr_BJambeARD(q);
    
    sl::copy(Xaxis  [::RH_KFE_JOINT], iit::rbd::Utils::zAxis(tmpX_7) );
    sl::copy(Xorigin[::RH_KFE_JOINT], iit::rbd::Utils::positionVector(tmpX_7) );
    
    sl::copy(Xlink[::BJAMBEARD], iit::rbd::Utils::positionVector(tmpX_7) );
    sl::copy(Ahmat[::BJAMBEARD], tmpX_7);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_BJambeARD() * inertias.getMass_BJambeARD();
    tmp_vec(3) = inertias.getMass_BJambeARD();
    sl::copy(Xmcog[::RH_KFE_JOINT], tmpX_7 * tmp_vec);
    
    //TODO  add the code for the endeffector links!!// The transform from EpauleARG to world
    tmpX_10 = SL::world_X_base * HT.fr_base_X_fr_EpauleARG(q);
    
    sl::copy(Xaxis  [::LH_HAA_JOINT], iit::rbd::Utils::zAxis(tmpX_10) );
    sl::copy(Xorigin[::LH_HAA_JOINT], iit::rbd::Utils::positionVector(tmpX_10) );
    
    sl::copy(Xlink[::EPAULEARG], iit::rbd::Utils::positionVector(tmpX_10) );
    sl::copy(Ahmat[::EPAULEARG], tmpX_10);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_EpauleARG() * inertias.getMass_EpauleARG();
    tmp_vec(3) = inertias.getMass_EpauleARG();
    sl::copy(Xmcog[::LH_HAA_JOINT], tmpX_10 * tmp_vec);
    
    // The transform from HJambeARG to world
    tmpX_10 = tmpX_10 * HT.fr_EpauleARG_X_fr_HJambeARG(q);
    
    sl::copy(Xaxis  [::LH_HFE_JOINT], iit::rbd::Utils::zAxis(tmpX_10) );
    sl::copy(Xorigin[::LH_HFE_JOINT], iit::rbd::Utils::positionVector(tmpX_10) );
    
    sl::copy(Xlink[::HJAMBEARG], iit::rbd::Utils::positionVector(tmpX_10) );
    sl::copy(Ahmat[::HJAMBEARG], tmpX_10);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_HJambeARG() * inertias.getMass_HJambeARG();
    tmp_vec(3) = inertias.getMass_HJambeARG();
    sl::copy(Xmcog[::LH_HFE_JOINT], tmpX_10 * tmp_vec);
    
    // The transform from BJambeARG to world
    tmpX_10 = tmpX_10 * HT.fr_HJambeARG_X_fr_BJambeARG(q);
    
    sl::copy(Xaxis  [::LH_KFE_JOINT], iit::rbd::Utils::zAxis(tmpX_10) );
    sl::copy(Xorigin[::LH_KFE_JOINT], iit::rbd::Utils::positionVector(tmpX_10) );
    
    sl::copy(Xlink[::BJAMBEARG], iit::rbd::Utils::positionVector(tmpX_10) );
    sl::copy(Ahmat[::BJAMBEARG], tmpX_10);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_BJambeARG() * inertias.getMass_BJambeARG();
    tmp_vec(3) = inertias.getMass_BJambeARG();
    sl::copy(Xmcog[::LH_KFE_JOINT], tmpX_10 * tmp_vec);
    
    //TODO  add the code for the endeffector links!!
}
