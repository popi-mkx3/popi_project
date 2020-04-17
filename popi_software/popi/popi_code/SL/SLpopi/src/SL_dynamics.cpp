/************************************************************* STRT CPYHDR
*
* SL - realtime robot control and simulation framework
* (c) 2010 Stefan Schaal, all rights reserved
*
* Copy, use and distribution of SL, both in source and binary form is
* not permitted without explicit permission by the copyright holder.
*
* Please contact Stefan Schaal <sschaal@usc.edu>
* for licensing information.
*
*
************************************************************* EOF CPYHDR */

#include <iit/robots/popi/declarations.h>
#include <iit/robots/popi/forward_dynamics.h>
#include <iit/robots/popi/traits.h>

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>

#include <sl/rcg/generic_dynamics.h>
#include <sl/rcg/rbd_conversion.h>
#include <sl/rcg/joint_status_conversion.h>

#include "robcogen_globals.h"

#include "SL_system_headers.h"
#include "SL.h"
#include "SL_dynamics.h"
#include "SL_common.h"


// global variables
int    freeze_base               = FALSE;
double freeze_base_pos[N_CART+1] = {0.0,0.0,0.0,0.0};
double freeze_base_quat[N_QUAT+1] = {0.0,1.0,0.0,0.0,0.0};


using namespace iit;
using namespace iit::popi;


int init_dynamics( void )
{
    int i;
    double quat[N_QUAT+1];
    double pos[N_CART+1];
    double euler[N_CART+1];
    double aux;

    // read link parameters
    if (!read_link_parameters(config_files[LINKPARAMETERS]))
        return FALSE;

    // the the default endeffector parameters
    setDefaultEndeffector();

    // initialize the base variables
    bzero((void *)&base_state,sizeof(base_state));
    bzero((void *)&base_orient,sizeof(base_orient));
    base_orient.q[_Q0_] = 1.0;

    if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_pos",N_CART,pos)) {
        for (i=1; i<=N_CART; ++i)
            freeze_base_pos[i] = base_state.x[i] = pos[i];
    }

    if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_quat",N_QUAT,quat)) {
        aux = 0.0;
        for (i=1; i<=N_QUAT; ++i)
            aux += sqr(quat[i]);
        aux = sqrt(aux);

        for (i=1; i<=N_QUAT; ++i)
            freeze_base_quat[i] = base_orient.q[i] = quat[i]/(aux + 1.e-10);
    } else if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_euler",N_CART,euler)) {
        SL_quat qtmp;

        bzero((void *)&qtmp,sizeof(qtmp));
        eulerToQuat(euler, &qtmp);
        for (i=1; i<=N_QUAT; ++i)
            freeze_base_quat[i] = base_orient.q[i] = qtmp.q[i];
    }
    return TRUE;
}

/*!
 * Inverse dynamics
 *
 * Original documentation (Sept 2010):
 *
 * Standard Newton Euler inverse dynamics, which switches automatically between
 * floating base and fixed base robots
 *
 * \param[in]     cstate  : the current state (pass NULL to use only desired state)
 * \param[in,out] lstate  : the desired state
 * \param[in]     endeff  : the endeffector parameters
 * \param[in]     cbase   : the position state of the base
 * \param[in]     obase   : the orientational state of the base
 *
 * Returns:
 * The appropriate feedforward torques are added in the uff component of the lstate
 * structure.
 *
 */
void SL_InvDyn(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
      SL_Cstate *cbase, SL_quat *obase)
{
    sl::inverse_dynamics<Traits>::
    floating_base(*SL::invDynEngine, SL::world_X_base.block<3,3>(0,0),
            cstate, lstate, cbase, obase);
}


/*!
 * Forward Dynamics
 *
 * Original documentation (date June 1999)
 *
 *         computes the forward dynamics accelerations
 *
 *
 *  \param[in,out] lstate  : the state containing th, thd, thdd, and receiving the
 *                           appropriate u
 *  \param[in,out] cbase   : the position state of the base
 *  \param[in,out] obase   : the orientational state of the base
 *  \param[in]     ux      : the external forces acting on each joint,
 *                           in world coordinates, e.g., as computed from contact
 *                           forces
 *  \param[in]     endeff  : the endeffector parameters
 *
 */
void SL_ForDyn(
    SL_Jstate *lstate,
    SL_Cstate *cbase, SL_quat *obase,
    SL_uext *ux, SL_endeff *leff)
{
    static dyn::ForwardDynamics::ExtForces extForces(iit::rbd::ForceVector::Zero());
    // TODO convert the external forces!

    sl::forward_dynamics<Traits>::
            floating_base(*SL::fwdDynEngine, extForces,
            SL::world_X_base.block<3,3>(0,0), lstate, cbase, obase);

    if(freeze_base) {
        //trunk_a.setZero();
        //trunk_v.setZero();
        sl::baseVelToSL(iit::rbd::VelocityVector::Zero(), *cbase, *obase);
        sl::baseAccelToSL(iit::rbd::VelocityVector::Zero(), *cbase, *obase);
    }
}


/*!
 *  Original documentation (date  Sept 2010)
 *
 *
 * computes the generalized joint forces due to friction and spring terms, i.e.,
 * the sum of all forces that act per joint independently of all others. The sign
 * of the terms is as if they were on the LEFT side of the RBD equations:
 *
 * M qdd + C qd + G + f = u
 *
 *
 *  \param[in] state       : the joint state of the robot
 *  \param[in] li          : the link parameters for this joint
 *
 *  returns the generalized joint force for this joint due friction and spring terms
 *
 */
double compute_independent_joint_forces(SL_Jstate state, SL_link li)
{
  double f=0;

  f = state.thd*li.vis +
    COULOMB_FUNCTION(state.thd)*li.coul +
    state.th*li.stiff +
    li.cons;

  return f;
}
