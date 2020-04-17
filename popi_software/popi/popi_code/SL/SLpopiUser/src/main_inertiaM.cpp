#include <iostream>
#include <fstream>
#include <ctime>

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/robots/popi/jsim.h>

#include "SL.h"
#include "SL_user.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"

using namespace std;
using namespace [iit];

static SL_Jstate currentState[N_ROBOT_DOFS];
static SL_endeff endeffector[N_ROBOT_ENDEFFECTORS];
static SL_Cstate basePosition;
static SL_quat baseOrient;
static SL_uext   ux[N_DOFS+1];
static Matrix rbdM;
static Vector rbdCG;

static void fillState(popi::JointState& q, SL_Jstate* SLState);
static void SL_init();

/* This main is supposed to be used to test the joint space inertia matrix routines */
int main(int argc, char**argv)
{
    popi::JointState q;

    SL_init();
    std::srand(std::time(NULL)); // initialize random number generator
    fillState(q, currentState);
    SL_ForDynComp(currentState, &basePosition, &baseOrient, ux, endeff, rbdM, rbdCG);
    popi::dyn::jsim(q);

    popi::dyn::JSIM SLM;

    // Copies the matrix of SL into an Eigen matrix, to make it easier to print, compare, etc.
    // Copy the joint-space part of the matrix.
    // Cannot use for loops because the joint ordering might be different in SL
    SLM(popi::RF_HAA_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[rf_haa_joint][rf_haa_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[rf_haa_joint][rf_hfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[rf_haa_joint][rf_kfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[rf_haa_joint][lf_haa_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[rf_haa_joint][lf_hfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[rf_haa_joint][lf_kfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[rf_haa_joint][rh_haa_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[rf_haa_joint][rh_hfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[rf_haa_joint][rh_kfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[rf_haa_joint][lh_haa_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[rf_haa_joint][lh_hfe_joint];
    SLM(popi::RF_HAA_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[rf_haa_joint][lh_kfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[rf_hfe_joint][rf_haa_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[rf_hfe_joint][rf_hfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[rf_hfe_joint][rf_kfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[rf_hfe_joint][lf_haa_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[rf_hfe_joint][lf_hfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[rf_hfe_joint][lf_kfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[rf_hfe_joint][rh_haa_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[rf_hfe_joint][rh_hfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[rf_hfe_joint][rh_kfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[rf_hfe_joint][lh_haa_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[rf_hfe_joint][lh_hfe_joint];
    SLM(popi::RF_HFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[rf_hfe_joint][lh_kfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[rf_kfe_joint][rf_haa_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[rf_kfe_joint][rf_hfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[rf_kfe_joint][rf_kfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[rf_kfe_joint][lf_haa_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[rf_kfe_joint][lf_hfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[rf_kfe_joint][lf_kfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[rf_kfe_joint][rh_haa_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[rf_kfe_joint][rh_hfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[rf_kfe_joint][rh_kfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[rf_kfe_joint][lh_haa_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[rf_kfe_joint][lh_hfe_joint];
    SLM(popi::RF_KFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[rf_kfe_joint][lh_kfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[lf_haa_joint][rf_haa_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[lf_haa_joint][rf_hfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[lf_haa_joint][rf_kfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[lf_haa_joint][lf_haa_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[lf_haa_joint][lf_hfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[lf_haa_joint][lf_kfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[lf_haa_joint][rh_haa_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[lf_haa_joint][rh_hfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[lf_haa_joint][rh_kfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[lf_haa_joint][lh_haa_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[lf_haa_joint][lh_hfe_joint];
    SLM(popi::LF_HAA_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[lf_haa_joint][lh_kfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[lf_hfe_joint][rf_haa_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[lf_hfe_joint][rf_hfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[lf_hfe_joint][rf_kfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[lf_hfe_joint][lf_haa_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[lf_hfe_joint][lf_hfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[lf_hfe_joint][lf_kfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[lf_hfe_joint][rh_haa_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[lf_hfe_joint][rh_hfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[lf_hfe_joint][rh_kfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[lf_hfe_joint][lh_haa_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[lf_hfe_joint][lh_hfe_joint];
    SLM(popi::LF_HFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[lf_hfe_joint][lh_kfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[lf_kfe_joint][rf_haa_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[lf_kfe_joint][rf_hfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[lf_kfe_joint][rf_kfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[lf_kfe_joint][lf_haa_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[lf_kfe_joint][lf_hfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[lf_kfe_joint][lf_kfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[lf_kfe_joint][rh_haa_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[lf_kfe_joint][rh_hfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[lf_kfe_joint][rh_kfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[lf_kfe_joint][lh_haa_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[lf_kfe_joint][lh_hfe_joint];
    SLM(popi::LF_KFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[lf_kfe_joint][lh_kfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[rh_haa_joint][rf_haa_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[rh_haa_joint][rf_hfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[rh_haa_joint][rf_kfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[rh_haa_joint][lf_haa_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[rh_haa_joint][lf_hfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[rh_haa_joint][lf_kfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[rh_haa_joint][rh_haa_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[rh_haa_joint][rh_hfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[rh_haa_joint][rh_kfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[rh_haa_joint][lh_haa_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[rh_haa_joint][lh_hfe_joint];
    SLM(popi::RH_HAA_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[rh_haa_joint][lh_kfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[rh_hfe_joint][rf_haa_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[rh_hfe_joint][rf_hfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[rh_hfe_joint][rf_kfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[rh_hfe_joint][lf_haa_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[rh_hfe_joint][lf_hfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[rh_hfe_joint][lf_kfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[rh_hfe_joint][rh_haa_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[rh_hfe_joint][rh_hfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[rh_hfe_joint][rh_kfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[rh_hfe_joint][lh_haa_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[rh_hfe_joint][lh_hfe_joint];
    SLM(popi::RH_HFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[rh_hfe_joint][lh_kfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[rh_kfe_joint][rf_haa_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[rh_kfe_joint][rf_hfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[rh_kfe_joint][rf_kfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[rh_kfe_joint][lf_haa_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[rh_kfe_joint][lf_hfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[rh_kfe_joint][lf_kfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[rh_kfe_joint][rh_haa_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[rh_kfe_joint][rh_hfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[rh_kfe_joint][rh_kfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[rh_kfe_joint][lh_haa_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[rh_kfe_joint][lh_hfe_joint];
    SLM(popi::RH_KFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[rh_kfe_joint][lh_kfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[lh_haa_joint][rf_haa_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[lh_haa_joint][rf_hfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[lh_haa_joint][rf_kfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[lh_haa_joint][lf_haa_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[lh_haa_joint][lf_hfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[lh_haa_joint][lf_kfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[lh_haa_joint][rh_haa_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[lh_haa_joint][rh_hfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[lh_haa_joint][rh_kfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[lh_haa_joint][lh_haa_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[lh_haa_joint][lh_hfe_joint];
    SLM(popi::LH_HAA_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[lh_haa_joint][lh_kfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[lh_hfe_joint][rf_haa_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[lh_hfe_joint][rf_hfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[lh_hfe_joint][rf_kfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[lh_hfe_joint][lf_haa_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[lh_hfe_joint][lf_hfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[lh_hfe_joint][lf_kfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[lh_hfe_joint][rh_haa_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[lh_hfe_joint][rh_hfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[lh_hfe_joint][rh_kfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[lh_hfe_joint][lh_haa_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[lh_hfe_joint][lh_hfe_joint];
    SLM(popi::LH_HFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[lh_hfe_joint][lh_kfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::RF_HAA_JOINT+6) = rbdM[lh_kfe_joint][rf_haa_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::RF_HFE_JOINT+6) = rbdM[lh_kfe_joint][rf_hfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::RF_KFE_JOINT+6) = rbdM[lh_kfe_joint][rf_kfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::LF_HAA_JOINT+6) = rbdM[lh_kfe_joint][lf_haa_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::LF_HFE_JOINT+6) = rbdM[lh_kfe_joint][lf_hfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::LF_KFE_JOINT+6) = rbdM[lh_kfe_joint][lf_kfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::RH_HAA_JOINT+6) = rbdM[lh_kfe_joint][rh_haa_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::RH_HFE_JOINT+6) = rbdM[lh_kfe_joint][rh_hfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::RH_KFE_JOINT+6) = rbdM[lh_kfe_joint][rh_kfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::LH_HAA_JOINT+6) = rbdM[lh_kfe_joint][lh_haa_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::LH_HFE_JOINT+6) = rbdM[lh_kfe_joint][lh_hfe_joint];
    SLM(popi::LH_KFE_JOINT+6,popi::LH_KFE_JOINT+6) = rbdM[lh_kfe_joint][lh_kfe_joint];
    // Copy the 6x6 block with the composite inertia of the whole robot:
    int r,c;
    for(r=0; r<6; r++) {
        for(c=0; c<6; c++) {
            SLM(r,c) = rbdM[r+1+12][c+1+12];
        }
    }
    // re-arrange blocks to match the convention of the generated dynamics code
    iit::rbd::Matrix33d temp;
    temp = SLM.block<3,3>(0,0);
    SLM.block<3,3>(0,0) = SLM.block<3,3>(3,3);
    SLM.block<3,3>(3,3) = temp;

    SLM.block<3,3>(0,3) = SLM.block<3,3>(3,0);
    SLM.block<3,3>(3,0) = SLM.block<3,3>(0,3).transpose();

    // Copy the remaining blocks:
    for(r=0; r<6; r++) {
        for(c=6; c<12; c++) {
            SLM(r,c) = rbdM[r+1+12][c+1-6];
        }
    }
    for(r=0; r<6; r++) {
        SLM(r,popi::RF_HAA_JOINT+6) = rbdM[r+1+12][rf_haa_joint];
        SLM(r,popi::RF_HFE_JOINT+6) = rbdM[r+1+12][rf_hfe_joint];
        SLM(r,popi::RF_KFE_JOINT+6) = rbdM[r+1+12][rf_kfe_joint];
        SLM(r,popi::LF_HAA_JOINT+6) = rbdM[r+1+12][lf_haa_joint];
        SLM(r,popi::LF_HFE_JOINT+6) = rbdM[r+1+12][lf_hfe_joint];
        SLM(r,popi::LF_KFE_JOINT+6) = rbdM[r+1+12][lf_kfe_joint];
        SLM(r,popi::RH_HAA_JOINT+6) = rbdM[r+1+12][rh_haa_joint];
        SLM(r,popi::RH_HFE_JOINT+6) = rbdM[r+1+12][rh_hfe_joint];
        SLM(r,popi::RH_KFE_JOINT+6) = rbdM[r+1+12][rh_kfe_joint];
        SLM(r,popi::LH_HAA_JOINT+6) = rbdM[r+1+12][lh_haa_joint];
        SLM(r,popi::LH_HFE_JOINT+6) = rbdM[r+1+12][lh_hfe_joint];
        SLM(r,popi::LH_KFE_JOINT+6) = rbdM[r+1+12][lh_kfe_joint];
    }
    // Deal with different convention about spatial vectors (linear/angular part)
    Eigen::Matrix<double,3,12> tempF;
    tempF = SLM.block<3,12>(0,6);
    SLM.block<3,12>(0,6) = SLM.block<3,12>(3,6);
    SLM.block<3,12>(3,6) = tempF;
    // F and F^T blocks
    SLM.block<12,6>(6,0) = SLM.block<6,12>(0,6).transpose();

    rbd::Utils::CwiseAlmostZeroOp<popi::dyn::JSIM::Scalar> almostZero(1E-4);

    cout << "SL:" << endl << SLM.unaryExpr(almostZero) << endl;
    cout << "Me:" << endl << popi::dyn::jsim.unaryExpr(almostZero)  << endl;

    //popi::dyn::JSIM::MatrixType diff = SLM - popi::dyn::jsim;
    //cout << "difference:" << endl << diff.unaryExpr(almostZero) << endl;

    //cout << SLM.block<6,6>(0,0).unaryExpr(almostZero) << endl;
    //cout << popi::dyn::jsim.block<6,6>(0,0).unaryExpr(almostZero) << endl;
    return TRUE;
}


void fillState(popi::JointState& q, SL_Jstate* SLState) {
    static const double max = 12.3;
    q(0) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[rf_haa_joint].th = q(0);
    q(1) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[rf_hfe_joint].th = q(1);
    q(2) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[rf_kfe_joint].th = q(2);
    q(3) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[lf_haa_joint].th = q(3);
    q(4) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[lf_hfe_joint].th = q(4);
    q(5) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[lf_kfe_joint].th = q(5);
    q(6) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[rh_haa_joint].th = q(6);
    q(7) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[rh_hfe_joint].th = q(7);
    q(8) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[rh_kfe_joint].th = q(8);
    q(9) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[lh_haa_joint].th = q(9);
    q(10) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[lh_hfe_joint].th = q(10);
    q(11) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[lh_kfe_joint].th = q(11);
}

static void SL_init() {
    init_kinematics();
    init_dynamics();

    bzero((void *)&basePosition,sizeof(basePosition));
    bzero((void *)&baseOrient,sizeof(baseOrient));
    bzero((void *)ux,sizeof(SL_uext)*N_DOFS+1);
    setDefaultEndeffector(); // the the default end-effector parameters

    baseOrient.q[_Q0_] = 1;
    baseOrient.q[_Q1_] = 0;
    baseOrient.q[_Q2_] = 0;
    baseOrient.q[_Q3_] = 0;

    rbdM = my_matrix(1,N_DOFS+6,1,N_DOFS+6);
    rbdCG = my_vector(1,N_DOFS+6);
    mat_zero(rbdM);
    vec_zero(rbdCG);

    freeze_base = TRUE;//
}
