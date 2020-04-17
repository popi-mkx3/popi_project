#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace iit::popi::dyn;

// Initialization of static-const data
const iit::popi::dyn::InverseDynamics::ExtForces
iit::popi::dyn::InverseDynamics::zeroExtForces(Force::Zero());

iit::popi::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    EpauleAVD_I(inertiaProps->getTensor_EpauleAVD() ),
    HJambeAVD_I(inertiaProps->getTensor_HJambeAVD() ),
    BJambeAVD_I(inertiaProps->getTensor_BJambeAVD() ),
    EpauleAVG_I(inertiaProps->getTensor_EpauleAVG() ),
    HJambeAVG_I(inertiaProps->getTensor_HJambeAVG() ),
    BJambeAVG_I(inertiaProps->getTensor_BJambeAVG() ),
    EpauleARD_I(inertiaProps->getTensor_EpauleARD() ),
    HJambeARD_I(inertiaProps->getTensor_HJambeARD() ),
    BJambeARD_I(inertiaProps->getTensor_BJambeARD() ),
    EpauleARG_I(inertiaProps->getTensor_EpauleARG() ),
    HJambeARG_I(inertiaProps->getTensor_HJambeARG() ),
    BJambeARG_I(inertiaProps->getTensor_BJambeARG() )
    ,
        base_I( inertiaProps->getTensor_base() ),
        BJambeAVD_Ic(BJambeAVD_I),
        BJambeAVG_Ic(BJambeAVG_I),
        BJambeARD_Ic(BJambeARD_I),
        BJambeARG_Ic(BJambeARG_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot popi, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    EpauleAVD_v.setZero();
    HJambeAVD_v.setZero();
    BJambeAVD_v.setZero();
    EpauleAVG_v.setZero();
    HJambeAVG_v.setZero();
    BJambeAVG_v.setZero();
    EpauleARD_v.setZero();
    HJambeARD_v.setZero();
    BJambeARD_v.setZero();
    EpauleARG_v.setZero();
    HJambeARG_v.setZero();
    BJambeARG_v.setZero();

    vcross.setZero();
}

void iit::popi::dyn::InverseDynamics::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_Ic = base_I;
    EpauleAVD_Ic = EpauleAVD_I;
    HJambeAVD_Ic = HJambeAVD_I;
    EpauleAVG_Ic = EpauleAVG_I;
    HJambeAVG_Ic = HJambeAVG_I;
    EpauleARD_Ic = EpauleARD_I;
    HJambeARD_Ic = HJambeARD_I;
    EpauleARG_Ic = EpauleARG_I;
    HJambeARG_Ic = HJambeARG_I;

    // First pass, link 'EpauleAVD'
    EpauleAVD_v = ((xm->fr_EpauleAVD_X_fr_base) * base_v);
    EpauleAVD_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleAVD_v, vcross);
    
    EpauleAVD_a = (vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT));
    EpauleAVD_a(iit::rbd::AZ) += qdd(RF_HAA_JOINT);
    
    EpauleAVD_f = EpauleAVD_I * EpauleAVD_a + vxIv(EpauleAVD_v, EpauleAVD_I);
    
    // First pass, link 'HJambeAVD'
    HJambeAVD_v = ((xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_v);
    HJambeAVD_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeAVD_v, vcross);
    
    HJambeAVD_a = (xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    HJambeAVD_a(iit::rbd::AZ) += qdd(RF_HFE_JOINT);
    
    HJambeAVD_f = HJambeAVD_I * HJambeAVD_a + vxIv(HJambeAVD_v, HJambeAVD_I);
    
    // First pass, link 'BJambeAVD'
    BJambeAVD_v = ((xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_v);
    BJambeAVD_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeAVD_v, vcross);
    
    BJambeAVD_a = (xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    BJambeAVD_a(iit::rbd::AZ) += qdd(RF_KFE_JOINT);
    
    BJambeAVD_f = BJambeAVD_I * BJambeAVD_a + vxIv(BJambeAVD_v, BJambeAVD_I);
    
    // First pass, link 'EpauleAVG'
    EpauleAVG_v = ((xm->fr_EpauleAVG_X_fr_base) * base_v);
    EpauleAVG_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleAVG_v, vcross);
    
    EpauleAVG_a = (vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT));
    EpauleAVG_a(iit::rbd::AZ) += qdd(LF_HAA_JOINT);
    
    EpauleAVG_f = EpauleAVG_I * EpauleAVG_a + vxIv(EpauleAVG_v, EpauleAVG_I);
    
    // First pass, link 'HJambeAVG'
    HJambeAVG_v = ((xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_v);
    HJambeAVG_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeAVG_v, vcross);
    
    HJambeAVG_a = (xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    HJambeAVG_a(iit::rbd::AZ) += qdd(LF_HFE_JOINT);
    
    HJambeAVG_f = HJambeAVG_I * HJambeAVG_a + vxIv(HJambeAVG_v, HJambeAVG_I);
    
    // First pass, link 'BJambeAVG'
    BJambeAVG_v = ((xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_v);
    BJambeAVG_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeAVG_v, vcross);
    
    BJambeAVG_a = (xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    BJambeAVG_a(iit::rbd::AZ) += qdd(LF_KFE_JOINT);
    
    BJambeAVG_f = BJambeAVG_I * BJambeAVG_a + vxIv(BJambeAVG_v, BJambeAVG_I);
    
    // First pass, link 'EpauleARD'
    EpauleARD_v = ((xm->fr_EpauleARD_X_fr_base) * base_v);
    EpauleARD_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleARD_v, vcross);
    
    EpauleARD_a = (vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT));
    EpauleARD_a(iit::rbd::AZ) += qdd(RH_HAA_JOINT);
    
    EpauleARD_f = EpauleARD_I * EpauleARD_a + vxIv(EpauleARD_v, EpauleARD_I);
    
    // First pass, link 'HJambeARD'
    HJambeARD_v = ((xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_v);
    HJambeARD_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeARD_v, vcross);
    
    HJambeARD_a = (xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    HJambeARD_a(iit::rbd::AZ) += qdd(RH_HFE_JOINT);
    
    HJambeARD_f = HJambeARD_I * HJambeARD_a + vxIv(HJambeARD_v, HJambeARD_I);
    
    // First pass, link 'BJambeARD'
    BJambeARD_v = ((xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_v);
    BJambeARD_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeARD_v, vcross);
    
    BJambeARD_a = (xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    BJambeARD_a(iit::rbd::AZ) += qdd(RH_KFE_JOINT);
    
    BJambeARD_f = BJambeARD_I * BJambeARD_a + vxIv(BJambeARD_v, BJambeARD_I);
    
    // First pass, link 'EpauleARG'
    EpauleARG_v = ((xm->fr_EpauleARG_X_fr_base) * base_v);
    EpauleARG_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleARG_v, vcross);
    
    EpauleARG_a = (vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT));
    EpauleARG_a(iit::rbd::AZ) += qdd(LH_HAA_JOINT);
    
    EpauleARG_f = EpauleARG_I * EpauleARG_a + vxIv(EpauleARG_v, EpauleARG_I);
    
    // First pass, link 'HJambeARG'
    HJambeARG_v = ((xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_v);
    HJambeARG_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeARG_v, vcross);
    
    HJambeARG_a = (xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    HJambeARG_a(iit::rbd::AZ) += qdd(LH_HFE_JOINT);
    
    HJambeARG_f = HJambeARG_I * HJambeARG_a + vxIv(HJambeARG_v, HJambeARG_I);
    
    // First pass, link 'BJambeARG'
    BJambeARG_v = ((xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_v);
    BJambeARG_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeARG_v, vcross);
    
    BJambeARG_a = (xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    BJambeARG_a(iit::rbd::AZ) += qdd(LH_KFE_JOINT);
    
    BJambeARG_f = BJambeARG_I * BJambeARG_a + vxIv(BJambeARG_v, BJambeARG_I);
    
    // The force exerted on the floating base by the links
    base_f = vxIv(base_v, base_I);
    

    // Add the external forces:
    base_f -= fext[BASE];
    EpauleAVD_f -= fext[EPAULEAVD];
    HJambeAVD_f -= fext[HJAMBEAVD];
    BJambeAVD_f -= fext[BJAMBEAVD];
    EpauleAVG_f -= fext[EPAULEAVG];
    HJambeAVG_f -= fext[HJAMBEAVG];
    BJambeAVG_f -= fext[BJAMBEAVG];
    EpauleARD_f -= fext[EPAULEARD];
    HJambeARD_f -= fext[HJAMBEARD];
    BJambeARD_f -= fext[BJAMBEARD];
    EpauleARG_f -= fext[EPAULEARG];
    HJambeARG_f -= fext[HJAMBEARG];
    BJambeARG_f -= fext[BJAMBEARG];

    HJambeARG_Ic = HJambeARG_Ic + (xm->fr_BJambeARG_X_fr_HJambeARG).transpose() * BJambeARG_Ic * (xm->fr_BJambeARG_X_fr_HJambeARG);
    HJambeARG_f = HJambeARG_f + (xm->fr_BJambeARG_X_fr_HJambeARG).transpose() * BJambeARG_f;
    
    EpauleARG_Ic = EpauleARG_Ic + (xm->fr_HJambeARG_X_fr_EpauleARG).transpose() * HJambeARG_Ic * (xm->fr_HJambeARG_X_fr_EpauleARG);
    EpauleARG_f = EpauleARG_f + (xm->fr_HJambeARG_X_fr_EpauleARG).transpose() * HJambeARG_f;
    
    base_Ic = base_Ic + (xm->fr_EpauleARG_X_fr_base).transpose() * EpauleARG_Ic * (xm->fr_EpauleARG_X_fr_base);
    base_f = base_f + (xm->fr_EpauleARG_X_fr_base).transpose() * EpauleARG_f;
    
    HJambeARD_Ic = HJambeARD_Ic + (xm->fr_BJambeARD_X_fr_HJambeARD).transpose() * BJambeARD_Ic * (xm->fr_BJambeARD_X_fr_HJambeARD);
    HJambeARD_f = HJambeARD_f + (xm->fr_BJambeARD_X_fr_HJambeARD).transpose() * BJambeARD_f;
    
    EpauleARD_Ic = EpauleARD_Ic + (xm->fr_HJambeARD_X_fr_EpauleARD).transpose() * HJambeARD_Ic * (xm->fr_HJambeARD_X_fr_EpauleARD);
    EpauleARD_f = EpauleARD_f + (xm->fr_HJambeARD_X_fr_EpauleARD).transpose() * HJambeARD_f;
    
    base_Ic = base_Ic + (xm->fr_EpauleARD_X_fr_base).transpose() * EpauleARD_Ic * (xm->fr_EpauleARD_X_fr_base);
    base_f = base_f + (xm->fr_EpauleARD_X_fr_base).transpose() * EpauleARD_f;
    
    HJambeAVG_Ic = HJambeAVG_Ic + (xm->fr_BJambeAVG_X_fr_HJambeAVG).transpose() * BJambeAVG_Ic * (xm->fr_BJambeAVG_X_fr_HJambeAVG);
    HJambeAVG_f = HJambeAVG_f + (xm->fr_BJambeAVG_X_fr_HJambeAVG).transpose() * BJambeAVG_f;
    
    EpauleAVG_Ic = EpauleAVG_Ic + (xm->fr_HJambeAVG_X_fr_EpauleAVG).transpose() * HJambeAVG_Ic * (xm->fr_HJambeAVG_X_fr_EpauleAVG);
    EpauleAVG_f = EpauleAVG_f + (xm->fr_HJambeAVG_X_fr_EpauleAVG).transpose() * HJambeAVG_f;
    
    base_Ic = base_Ic + (xm->fr_EpauleAVG_X_fr_base).transpose() * EpauleAVG_Ic * (xm->fr_EpauleAVG_X_fr_base);
    base_f = base_f + (xm->fr_EpauleAVG_X_fr_base).transpose() * EpauleAVG_f;
    
    HJambeAVD_Ic = HJambeAVD_Ic + (xm->fr_BJambeAVD_X_fr_HJambeAVD).transpose() * BJambeAVD_Ic * (xm->fr_BJambeAVD_X_fr_HJambeAVD);
    HJambeAVD_f = HJambeAVD_f + (xm->fr_BJambeAVD_X_fr_HJambeAVD).transpose() * BJambeAVD_f;
    
    EpauleAVD_Ic = EpauleAVD_Ic + (xm->fr_HJambeAVD_X_fr_EpauleAVD).transpose() * HJambeAVD_Ic * (xm->fr_HJambeAVD_X_fr_EpauleAVD);
    EpauleAVD_f = EpauleAVD_f + (xm->fr_HJambeAVD_X_fr_EpauleAVD).transpose() * HJambeAVD_f;
    
    base_Ic = base_Ic + (xm->fr_EpauleAVD_X_fr_base).transpose() * EpauleAVD_Ic * (xm->fr_EpauleAVD_X_fr_base);
    base_f = base_f + (xm->fr_EpauleAVD_X_fr_base).transpose() * EpauleAVD_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_a = - base_Ic.inverse() * base_f;
    
    EpauleAVD_a = xm->fr_EpauleAVD_X_fr_base * base_a;
    jForces(RF_HAA_JOINT) = (EpauleAVD_Ic.row(iit::rbd::AZ) * EpauleAVD_a + EpauleAVD_f(iit::rbd::AZ));
    
    HJambeAVD_a = xm->fr_HJambeAVD_X_fr_EpauleAVD * EpauleAVD_a;
    jForces(RF_HFE_JOINT) = (HJambeAVD_Ic.row(iit::rbd::AZ) * HJambeAVD_a + HJambeAVD_f(iit::rbd::AZ));
    
    BJambeAVD_a = xm->fr_BJambeAVD_X_fr_HJambeAVD * HJambeAVD_a;
    jForces(RF_KFE_JOINT) = (BJambeAVD_Ic.row(iit::rbd::AZ) * BJambeAVD_a + BJambeAVD_f(iit::rbd::AZ));
    
    EpauleAVG_a = xm->fr_EpauleAVG_X_fr_base * base_a;
    jForces(LF_HAA_JOINT) = (EpauleAVG_Ic.row(iit::rbd::AZ) * EpauleAVG_a + EpauleAVG_f(iit::rbd::AZ));
    
    HJambeAVG_a = xm->fr_HJambeAVG_X_fr_EpauleAVG * EpauleAVG_a;
    jForces(LF_HFE_JOINT) = (HJambeAVG_Ic.row(iit::rbd::AZ) * HJambeAVG_a + HJambeAVG_f(iit::rbd::AZ));
    
    BJambeAVG_a = xm->fr_BJambeAVG_X_fr_HJambeAVG * HJambeAVG_a;
    jForces(LF_KFE_JOINT) = (BJambeAVG_Ic.row(iit::rbd::AZ) * BJambeAVG_a + BJambeAVG_f(iit::rbd::AZ));
    
    EpauleARD_a = xm->fr_EpauleARD_X_fr_base * base_a;
    jForces(RH_HAA_JOINT) = (EpauleARD_Ic.row(iit::rbd::AZ) * EpauleARD_a + EpauleARD_f(iit::rbd::AZ));
    
    HJambeARD_a = xm->fr_HJambeARD_X_fr_EpauleARD * EpauleARD_a;
    jForces(RH_HFE_JOINT) = (HJambeARD_Ic.row(iit::rbd::AZ) * HJambeARD_a + HJambeARD_f(iit::rbd::AZ));
    
    BJambeARD_a = xm->fr_BJambeARD_X_fr_HJambeARD * HJambeARD_a;
    jForces(RH_KFE_JOINT) = (BJambeARD_Ic.row(iit::rbd::AZ) * BJambeARD_a + BJambeARD_f(iit::rbd::AZ));
    
    EpauleARG_a = xm->fr_EpauleARG_X_fr_base * base_a;
    jForces(LH_HAA_JOINT) = (EpauleARG_Ic.row(iit::rbd::AZ) * EpauleARG_a + EpauleARG_f(iit::rbd::AZ));
    
    HJambeARG_a = xm->fr_HJambeARG_X_fr_EpauleARG * EpauleARG_a;
    jForces(LH_HFE_JOINT) = (HJambeARG_Ic.row(iit::rbd::AZ) * HJambeARG_a + HJambeARG_f(iit::rbd::AZ));
    
    BJambeARG_a = xm->fr_BJambeARG_X_fr_HJambeARG * HJambeARG_a;
    jForces(LH_KFE_JOINT) = (BJambeARG_Ic.row(iit::rbd::AZ) * BJambeARG_a + BJambeARG_f(iit::rbd::AZ));
    

    base_a += g;
}


void iit::popi::dyn::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_a = -g;

    // Link 'EpauleAVD'
    EpauleAVD_a = (xm->fr_EpauleAVD_X_fr_base) * base_a;
    EpauleAVD_f = EpauleAVD_I * EpauleAVD_a;
    // Link 'HJambeAVD'
    HJambeAVD_a = (xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_a;
    HJambeAVD_f = HJambeAVD_I * HJambeAVD_a;
    // Link 'BJambeAVD'
    BJambeAVD_a = (xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_a;
    BJambeAVD_f = BJambeAVD_I * BJambeAVD_a;
    // Link 'EpauleAVG'
    EpauleAVG_a = (xm->fr_EpauleAVG_X_fr_base) * base_a;
    EpauleAVG_f = EpauleAVG_I * EpauleAVG_a;
    // Link 'HJambeAVG'
    HJambeAVG_a = (xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_a;
    HJambeAVG_f = HJambeAVG_I * HJambeAVG_a;
    // Link 'BJambeAVG'
    BJambeAVG_a = (xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_a;
    BJambeAVG_f = BJambeAVG_I * BJambeAVG_a;
    // Link 'EpauleARD'
    EpauleARD_a = (xm->fr_EpauleARD_X_fr_base) * base_a;
    EpauleARD_f = EpauleARD_I * EpauleARD_a;
    // Link 'HJambeARD'
    HJambeARD_a = (xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_a;
    HJambeARD_f = HJambeARD_I * HJambeARD_a;
    // Link 'BJambeARD'
    BJambeARD_a = (xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_a;
    BJambeARD_f = BJambeARD_I * BJambeARD_a;
    // Link 'EpauleARG'
    EpauleARG_a = (xm->fr_EpauleARG_X_fr_base) * base_a;
    EpauleARG_f = EpauleARG_I * EpauleARG_a;
    // Link 'HJambeARG'
    HJambeARG_a = (xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_a;
    HJambeARG_f = HJambeARG_I * HJambeARG_a;
    // Link 'BJambeARG'
    BJambeARG_a = (xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_a;
    BJambeARG_f = BJambeARG_I * BJambeARG_a;

    base_f = base_I * base_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

void iit::popi::dyn::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& qd)
{
    // Link 'EpauleAVD'
    EpauleAVD_v = ((xm->fr_EpauleAVD_X_fr_base) * base_v);
    EpauleAVD_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    motionCrossProductMx<Scalar>(EpauleAVD_v, vcross);
    EpauleAVD_a = (vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT));
    EpauleAVD_f = EpauleAVD_I * EpauleAVD_a + vxIv(EpauleAVD_v, EpauleAVD_I);
    
    // Link 'HJambeAVD'
    HJambeAVD_v = ((xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_v);
    HJambeAVD_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    motionCrossProductMx<Scalar>(HJambeAVD_v, vcross);
    HJambeAVD_a = (xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    HJambeAVD_f = HJambeAVD_I * HJambeAVD_a + vxIv(HJambeAVD_v, HJambeAVD_I);
    
    // Link 'BJambeAVD'
    BJambeAVD_v = ((xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_v);
    BJambeAVD_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    motionCrossProductMx<Scalar>(BJambeAVD_v, vcross);
    BJambeAVD_a = (xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    BJambeAVD_f = BJambeAVD_I * BJambeAVD_a + vxIv(BJambeAVD_v, BJambeAVD_I);
    
    // Link 'EpauleAVG'
    EpauleAVG_v = ((xm->fr_EpauleAVG_X_fr_base) * base_v);
    EpauleAVG_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    motionCrossProductMx<Scalar>(EpauleAVG_v, vcross);
    EpauleAVG_a = (vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT));
    EpauleAVG_f = EpauleAVG_I * EpauleAVG_a + vxIv(EpauleAVG_v, EpauleAVG_I);
    
    // Link 'HJambeAVG'
    HJambeAVG_v = ((xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_v);
    HJambeAVG_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    motionCrossProductMx<Scalar>(HJambeAVG_v, vcross);
    HJambeAVG_a = (xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    HJambeAVG_f = HJambeAVG_I * HJambeAVG_a + vxIv(HJambeAVG_v, HJambeAVG_I);
    
    // Link 'BJambeAVG'
    BJambeAVG_v = ((xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_v);
    BJambeAVG_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    motionCrossProductMx<Scalar>(BJambeAVG_v, vcross);
    BJambeAVG_a = (xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    BJambeAVG_f = BJambeAVG_I * BJambeAVG_a + vxIv(BJambeAVG_v, BJambeAVG_I);
    
    // Link 'EpauleARD'
    EpauleARD_v = ((xm->fr_EpauleARD_X_fr_base) * base_v);
    EpauleARD_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    motionCrossProductMx<Scalar>(EpauleARD_v, vcross);
    EpauleARD_a = (vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT));
    EpauleARD_f = EpauleARD_I * EpauleARD_a + vxIv(EpauleARD_v, EpauleARD_I);
    
    // Link 'HJambeARD'
    HJambeARD_v = ((xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_v);
    HJambeARD_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    motionCrossProductMx<Scalar>(HJambeARD_v, vcross);
    HJambeARD_a = (xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    HJambeARD_f = HJambeARD_I * HJambeARD_a + vxIv(HJambeARD_v, HJambeARD_I);
    
    // Link 'BJambeARD'
    BJambeARD_v = ((xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_v);
    BJambeARD_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    motionCrossProductMx<Scalar>(BJambeARD_v, vcross);
    BJambeARD_a = (xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    BJambeARD_f = BJambeARD_I * BJambeARD_a + vxIv(BJambeARD_v, BJambeARD_I);
    
    // Link 'EpauleARG'
    EpauleARG_v = ((xm->fr_EpauleARG_X_fr_base) * base_v);
    EpauleARG_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    motionCrossProductMx<Scalar>(EpauleARG_v, vcross);
    EpauleARG_a = (vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT));
    EpauleARG_f = EpauleARG_I * EpauleARG_a + vxIv(EpauleARG_v, EpauleARG_I);
    
    // Link 'HJambeARG'
    HJambeARG_v = ((xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_v);
    HJambeARG_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    motionCrossProductMx<Scalar>(HJambeARG_v, vcross);
    HJambeARG_a = (xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    HJambeARG_f = HJambeARG_I * HJambeARG_a + vxIv(HJambeARG_v, HJambeARG_I);
    
    // Link 'BJambeARG'
    BJambeARG_v = ((xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_v);
    BJambeARG_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    motionCrossProductMx<Scalar>(BJambeARG_v, vcross);
    BJambeARG_a = (xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    BJambeARG_f = BJambeARG_I * BJambeARG_a + vxIv(BJambeARG_v, BJambeARG_I);
    

    base_f = vxIv(base_v, base_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

void iit::popi::dyn::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_a = baseAccel -g;

    // First pass, link 'EpauleAVD'
    EpauleAVD_v = ((xm->fr_EpauleAVD_X_fr_base) * base_v);
    EpauleAVD_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleAVD_v, vcross);
    
    EpauleAVD_a = (xm->fr_EpauleAVD_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT);
    EpauleAVD_a(iit::rbd::AZ) += qdd(RF_HAA_JOINT);
    
    EpauleAVD_f = EpauleAVD_I * EpauleAVD_a + vxIv(EpauleAVD_v, EpauleAVD_I) - fext[EPAULEAVD];
    
    // First pass, link 'HJambeAVD'
    HJambeAVD_v = ((xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_v);
    HJambeAVD_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeAVD_v, vcross);
    
    HJambeAVD_a = (xm->fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    HJambeAVD_a(iit::rbd::AZ) += qdd(RF_HFE_JOINT);
    
    HJambeAVD_f = HJambeAVD_I * HJambeAVD_a + vxIv(HJambeAVD_v, HJambeAVD_I) - fext[HJAMBEAVD];
    
    // First pass, link 'BJambeAVD'
    BJambeAVD_v = ((xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_v);
    BJambeAVD_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeAVD_v, vcross);
    
    BJambeAVD_a = (xm->fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    BJambeAVD_a(iit::rbd::AZ) += qdd(RF_KFE_JOINT);
    
    BJambeAVD_f = BJambeAVD_I * BJambeAVD_a + vxIv(BJambeAVD_v, BJambeAVD_I) - fext[BJAMBEAVD];
    
    // First pass, link 'EpauleAVG'
    EpauleAVG_v = ((xm->fr_EpauleAVG_X_fr_base) * base_v);
    EpauleAVG_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleAVG_v, vcross);
    
    EpauleAVG_a = (xm->fr_EpauleAVG_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT);
    EpauleAVG_a(iit::rbd::AZ) += qdd(LF_HAA_JOINT);
    
    EpauleAVG_f = EpauleAVG_I * EpauleAVG_a + vxIv(EpauleAVG_v, EpauleAVG_I) - fext[EPAULEAVG];
    
    // First pass, link 'HJambeAVG'
    HJambeAVG_v = ((xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_v);
    HJambeAVG_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeAVG_v, vcross);
    
    HJambeAVG_a = (xm->fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    HJambeAVG_a(iit::rbd::AZ) += qdd(LF_HFE_JOINT);
    
    HJambeAVG_f = HJambeAVG_I * HJambeAVG_a + vxIv(HJambeAVG_v, HJambeAVG_I) - fext[HJAMBEAVG];
    
    // First pass, link 'BJambeAVG'
    BJambeAVG_v = ((xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_v);
    BJambeAVG_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeAVG_v, vcross);
    
    BJambeAVG_a = (xm->fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    BJambeAVG_a(iit::rbd::AZ) += qdd(LF_KFE_JOINT);
    
    BJambeAVG_f = BJambeAVG_I * BJambeAVG_a + vxIv(BJambeAVG_v, BJambeAVG_I) - fext[BJAMBEAVG];
    
    // First pass, link 'EpauleARD'
    EpauleARD_v = ((xm->fr_EpauleARD_X_fr_base) * base_v);
    EpauleARD_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleARD_v, vcross);
    
    EpauleARD_a = (xm->fr_EpauleARD_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT);
    EpauleARD_a(iit::rbd::AZ) += qdd(RH_HAA_JOINT);
    
    EpauleARD_f = EpauleARD_I * EpauleARD_a + vxIv(EpauleARD_v, EpauleARD_I) - fext[EPAULEARD];
    
    // First pass, link 'HJambeARD'
    HJambeARD_v = ((xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_v);
    HJambeARD_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeARD_v, vcross);
    
    HJambeARD_a = (xm->fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    HJambeARD_a(iit::rbd::AZ) += qdd(RH_HFE_JOINT);
    
    HJambeARD_f = HJambeARD_I * HJambeARD_a + vxIv(HJambeARD_v, HJambeARD_I) - fext[HJAMBEARD];
    
    // First pass, link 'BJambeARD'
    BJambeARD_v = ((xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_v);
    BJambeARD_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeARD_v, vcross);
    
    BJambeARD_a = (xm->fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    BJambeARD_a(iit::rbd::AZ) += qdd(RH_KFE_JOINT);
    
    BJambeARD_f = BJambeARD_I * BJambeARD_a + vxIv(BJambeARD_v, BJambeARD_I) - fext[BJAMBEARD];
    
    // First pass, link 'EpauleARG'
    EpauleARG_v = ((xm->fr_EpauleARG_X_fr_base) * base_v);
    EpauleARG_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    
    motionCrossProductMx<Scalar>(EpauleARG_v, vcross);
    
    EpauleARG_a = (xm->fr_EpauleARG_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT);
    EpauleARG_a(iit::rbd::AZ) += qdd(LH_HAA_JOINT);
    
    EpauleARG_f = EpauleARG_I * EpauleARG_a + vxIv(EpauleARG_v, EpauleARG_I) - fext[EPAULEARG];
    
    // First pass, link 'HJambeARG'
    HJambeARG_v = ((xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_v);
    HJambeARG_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    
    motionCrossProductMx<Scalar>(HJambeARG_v, vcross);
    
    HJambeARG_a = (xm->fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    HJambeARG_a(iit::rbd::AZ) += qdd(LH_HFE_JOINT);
    
    HJambeARG_f = HJambeARG_I * HJambeARG_a + vxIv(HJambeARG_v, HJambeARG_I) - fext[HJAMBEARG];
    
    // First pass, link 'BJambeARG'
    BJambeARG_v = ((xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_v);
    BJambeARG_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    
    motionCrossProductMx<Scalar>(BJambeARG_v, vcross);
    
    BJambeARG_a = (xm->fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    BJambeARG_a(iit::rbd::AZ) += qdd(LH_KFE_JOINT);
    
    BJambeARG_f = BJambeARG_I * BJambeARG_a + vxIv(BJambeARG_v, BJambeARG_I) - fext[BJAMBEARG];
    

    // The base
    base_f = base_I * base_a + vxIv(base_v, base_I) - fext[BASE];

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}


void iit::popi::dyn::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'BJambeARG'
    jForces(LH_KFE_JOINT) = BJambeARG_f(iit::rbd::AZ);
    HJambeARG_f += xm->fr_BJambeARG_X_fr_HJambeARG.transpose() * BJambeARG_f;
    // Link 'HJambeARG'
    jForces(LH_HFE_JOINT) = HJambeARG_f(iit::rbd::AZ);
    EpauleARG_f += xm->fr_HJambeARG_X_fr_EpauleARG.transpose() * HJambeARG_f;
    // Link 'EpauleARG'
    jForces(LH_HAA_JOINT) = EpauleARG_f(iit::rbd::AZ);
    base_f += xm->fr_EpauleARG_X_fr_base.transpose() * EpauleARG_f;
    // Link 'BJambeARD'
    jForces(RH_KFE_JOINT) = BJambeARD_f(iit::rbd::AZ);
    HJambeARD_f += xm->fr_BJambeARD_X_fr_HJambeARD.transpose() * BJambeARD_f;
    // Link 'HJambeARD'
    jForces(RH_HFE_JOINT) = HJambeARD_f(iit::rbd::AZ);
    EpauleARD_f += xm->fr_HJambeARD_X_fr_EpauleARD.transpose() * HJambeARD_f;
    // Link 'EpauleARD'
    jForces(RH_HAA_JOINT) = EpauleARD_f(iit::rbd::AZ);
    base_f += xm->fr_EpauleARD_X_fr_base.transpose() * EpauleARD_f;
    // Link 'BJambeAVG'
    jForces(LF_KFE_JOINT) = BJambeAVG_f(iit::rbd::AZ);
    HJambeAVG_f += xm->fr_BJambeAVG_X_fr_HJambeAVG.transpose() * BJambeAVG_f;
    // Link 'HJambeAVG'
    jForces(LF_HFE_JOINT) = HJambeAVG_f(iit::rbd::AZ);
    EpauleAVG_f += xm->fr_HJambeAVG_X_fr_EpauleAVG.transpose() * HJambeAVG_f;
    // Link 'EpauleAVG'
    jForces(LF_HAA_JOINT) = EpauleAVG_f(iit::rbd::AZ);
    base_f += xm->fr_EpauleAVG_X_fr_base.transpose() * EpauleAVG_f;
    // Link 'BJambeAVD'
    jForces(RF_KFE_JOINT) = BJambeAVD_f(iit::rbd::AZ);
    HJambeAVD_f += xm->fr_BJambeAVD_X_fr_HJambeAVD.transpose() * BJambeAVD_f;
    // Link 'HJambeAVD'
    jForces(RF_HFE_JOINT) = HJambeAVD_f(iit::rbd::AZ);
    EpauleAVD_f += xm->fr_HJambeAVD_X_fr_EpauleAVD.transpose() * HJambeAVD_f;
    // Link 'EpauleAVD'
    jForces(RF_HAA_JOINT) = EpauleAVD_f(iit::rbd::AZ);
    base_f += xm->fr_EpauleAVD_X_fr_base.transpose() * EpauleAVD_f;
}
