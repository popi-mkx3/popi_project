#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const iit::popi::dyn::ForwardDynamics::ExtForces
    iit::popi::dyn::ForwardDynamics::zeroExtForces(Force::Zero());

iit::popi::dyn::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    EpauleAVD_v.setZero();
    EpauleAVD_c.setZero();
    HJambeAVD_v.setZero();
    HJambeAVD_c.setZero();
    BJambeAVD_v.setZero();
    BJambeAVD_c.setZero();
    EpauleAVG_v.setZero();
    EpauleAVG_c.setZero();
    HJambeAVG_v.setZero();
    HJambeAVG_c.setZero();
    BJambeAVG_v.setZero();
    BJambeAVG_c.setZero();
    EpauleARD_v.setZero();
    EpauleARD_c.setZero();
    HJambeARD_v.setZero();
    HJambeARD_c.setZero();
    BJambeARD_v.setZero();
    BJambeARD_c.setZero();
    EpauleARG_v.setZero();
    EpauleARG_c.setZero();
    HJambeARG_v.setZero();
    HJambeARG_c.setZero();
    BJambeARG_v.setZero();
    BJambeARG_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void iit::popi::dyn::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& base_a,
    const Velocity& base_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_AI = inertiaProps->getTensor_base();
    base_p = - fext[BASE];
    EpauleAVD_AI = inertiaProps->getTensor_EpauleAVD();
    EpauleAVD_p = - fext[EPAULEAVD];
    HJambeAVD_AI = inertiaProps->getTensor_HJambeAVD();
    HJambeAVD_p = - fext[HJAMBEAVD];
    BJambeAVD_AI = inertiaProps->getTensor_BJambeAVD();
    BJambeAVD_p = - fext[BJAMBEAVD];
    EpauleAVG_AI = inertiaProps->getTensor_EpauleAVG();
    EpauleAVG_p = - fext[EPAULEAVG];
    HJambeAVG_AI = inertiaProps->getTensor_HJambeAVG();
    HJambeAVG_p = - fext[HJAMBEAVG];
    BJambeAVG_AI = inertiaProps->getTensor_BJambeAVG();
    BJambeAVG_p = - fext[BJAMBEAVG];
    EpauleARD_AI = inertiaProps->getTensor_EpauleARD();
    EpauleARD_p = - fext[EPAULEARD];
    HJambeARD_AI = inertiaProps->getTensor_HJambeARD();
    HJambeARD_p = - fext[HJAMBEARD];
    BJambeARD_AI = inertiaProps->getTensor_BJambeARD();
    BJambeARD_p = - fext[BJAMBEARD];
    EpauleARG_AI = inertiaProps->getTensor_EpauleARG();
    EpauleARG_p = - fext[EPAULEARG];
    HJambeARG_AI = inertiaProps->getTensor_HJambeARG();
    HJambeARG_p = - fext[HJAMBEARG];
    BJambeARG_AI = inertiaProps->getTensor_BJambeARG();
    BJambeARG_p = - fext[BJAMBEARG];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link EpauleAVD
    //  - The spatial velocity:
    EpauleAVD_v = (motionTransforms-> fr_EpauleAVD_X_fr_base) * base_v;
    EpauleAVD_v(AZ) += qd(RF_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(EpauleAVD_v, vcross);
    EpauleAVD_c = vcross.col(AZ) * qd(RF_HAA_JOINT);
    
    //  - The bias force term:
    EpauleAVD_p += vxIv(EpauleAVD_v, EpauleAVD_AI);
    
    // + Link HJambeAVD
    //  - The spatial velocity:
    HJambeAVD_v = (motionTransforms-> fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_v;
    HJambeAVD_v(AZ) += qd(RF_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(HJambeAVD_v, vcross);
    HJambeAVD_c = vcross.col(AZ) * qd(RF_HFE_JOINT);
    
    //  - The bias force term:
    HJambeAVD_p += vxIv(HJambeAVD_v, HJambeAVD_AI);
    
    // + Link BJambeAVD
    //  - The spatial velocity:
    BJambeAVD_v = (motionTransforms-> fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_v;
    BJambeAVD_v(AZ) += qd(RF_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(BJambeAVD_v, vcross);
    BJambeAVD_c = vcross.col(AZ) * qd(RF_KFE_JOINT);
    
    //  - The bias force term:
    BJambeAVD_p += vxIv(BJambeAVD_v, BJambeAVD_AI);
    
    // + Link EpauleAVG
    //  - The spatial velocity:
    EpauleAVG_v = (motionTransforms-> fr_EpauleAVG_X_fr_base) * base_v;
    EpauleAVG_v(AZ) += qd(LF_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(EpauleAVG_v, vcross);
    EpauleAVG_c = vcross.col(AZ) * qd(LF_HAA_JOINT);
    
    //  - The bias force term:
    EpauleAVG_p += vxIv(EpauleAVG_v, EpauleAVG_AI);
    
    // + Link HJambeAVG
    //  - The spatial velocity:
    HJambeAVG_v = (motionTransforms-> fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_v;
    HJambeAVG_v(AZ) += qd(LF_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(HJambeAVG_v, vcross);
    HJambeAVG_c = vcross.col(AZ) * qd(LF_HFE_JOINT);
    
    //  - The bias force term:
    HJambeAVG_p += vxIv(HJambeAVG_v, HJambeAVG_AI);
    
    // + Link BJambeAVG
    //  - The spatial velocity:
    BJambeAVG_v = (motionTransforms-> fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_v;
    BJambeAVG_v(AZ) += qd(LF_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(BJambeAVG_v, vcross);
    BJambeAVG_c = vcross.col(AZ) * qd(LF_KFE_JOINT);
    
    //  - The bias force term:
    BJambeAVG_p += vxIv(BJambeAVG_v, BJambeAVG_AI);
    
    // + Link EpauleARD
    //  - The spatial velocity:
    EpauleARD_v = (motionTransforms-> fr_EpauleARD_X_fr_base) * base_v;
    EpauleARD_v(AZ) += qd(RH_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(EpauleARD_v, vcross);
    EpauleARD_c = vcross.col(AZ) * qd(RH_HAA_JOINT);
    
    //  - The bias force term:
    EpauleARD_p += vxIv(EpauleARD_v, EpauleARD_AI);
    
    // + Link HJambeARD
    //  - The spatial velocity:
    HJambeARD_v = (motionTransforms-> fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_v;
    HJambeARD_v(AZ) += qd(RH_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(HJambeARD_v, vcross);
    HJambeARD_c = vcross.col(AZ) * qd(RH_HFE_JOINT);
    
    //  - The bias force term:
    HJambeARD_p += vxIv(HJambeARD_v, HJambeARD_AI);
    
    // + Link BJambeARD
    //  - The spatial velocity:
    BJambeARD_v = (motionTransforms-> fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_v;
    BJambeARD_v(AZ) += qd(RH_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(BJambeARD_v, vcross);
    BJambeARD_c = vcross.col(AZ) * qd(RH_KFE_JOINT);
    
    //  - The bias force term:
    BJambeARD_p += vxIv(BJambeARD_v, BJambeARD_AI);
    
    // + Link EpauleARG
    //  - The spatial velocity:
    EpauleARG_v = (motionTransforms-> fr_EpauleARG_X_fr_base) * base_v;
    EpauleARG_v(AZ) += qd(LH_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(EpauleARG_v, vcross);
    EpauleARG_c = vcross.col(AZ) * qd(LH_HAA_JOINT);
    
    //  - The bias force term:
    EpauleARG_p += vxIv(EpauleARG_v, EpauleARG_AI);
    
    // + Link HJambeARG
    //  - The spatial velocity:
    HJambeARG_v = (motionTransforms-> fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_v;
    HJambeARG_v(AZ) += qd(LH_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(HJambeARG_v, vcross);
    HJambeARG_c = vcross.col(AZ) * qd(LH_HFE_JOINT);
    
    //  - The bias force term:
    HJambeARG_p += vxIv(HJambeARG_v, HJambeARG_AI);
    
    // + Link BJambeARG
    //  - The spatial velocity:
    BJambeARG_v = (motionTransforms-> fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_v;
    BJambeARG_v(AZ) += qd(LH_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(BJambeARG_v, vcross);
    BJambeARG_c = vcross.col(AZ) * qd(LH_KFE_JOINT);
    
    //  - The bias force term:
    BJambeARG_p += vxIv(BJambeARG_v, BJambeARG_AI);
    
    // + The floating base body
    base_p += vxIv(base_v, base_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link BJambeARG
    BJambeARG_u = tau(LH_KFE_JOINT) - BJambeARG_p(AZ);
    BJambeARG_U = BJambeARG_AI.col(AZ);
    BJambeARG_D = BJambeARG_U(AZ);
    
    compute_Ia_revolute(BJambeARG_AI, BJambeARG_U, BJambeARG_D, Ia_r);  // same as: Ia_r = BJambeARG_AI - BJambeARG_U/BJambeARG_D * BJambeARG_U.transpose();
    pa = BJambeARG_p + Ia_r * BJambeARG_c + BJambeARG_U * BJambeARG_u/BJambeARG_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_BJambeARG_X_fr_HJambeARG, IaB);
    HJambeARG_AI += IaB;
    HJambeARG_p += (motionTransforms-> fr_BJambeARG_X_fr_HJambeARG).transpose() * pa;
    
    // + Link HJambeARG
    HJambeARG_u = tau(LH_HFE_JOINT) - HJambeARG_p(AZ);
    HJambeARG_U = HJambeARG_AI.col(AZ);
    HJambeARG_D = HJambeARG_U(AZ);
    
    compute_Ia_revolute(HJambeARG_AI, HJambeARG_U, HJambeARG_D, Ia_r);  // same as: Ia_r = HJambeARG_AI - HJambeARG_U/HJambeARG_D * HJambeARG_U.transpose();
    pa = HJambeARG_p + Ia_r * HJambeARG_c + HJambeARG_U * HJambeARG_u/HJambeARG_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_HJambeARG_X_fr_EpauleARG, IaB);
    EpauleARG_AI += IaB;
    EpauleARG_p += (motionTransforms-> fr_HJambeARG_X_fr_EpauleARG).transpose() * pa;
    
    // + Link EpauleARG
    EpauleARG_u = tau(LH_HAA_JOINT) - EpauleARG_p(AZ);
    EpauleARG_U = EpauleARG_AI.col(AZ);
    EpauleARG_D = EpauleARG_U(AZ);
    
    compute_Ia_revolute(EpauleARG_AI, EpauleARG_U, EpauleARG_D, Ia_r);  // same as: Ia_r = EpauleARG_AI - EpauleARG_U/EpauleARG_D * EpauleARG_U.transpose();
    pa = EpauleARG_p + Ia_r * EpauleARG_c + EpauleARG_U * EpauleARG_u/EpauleARG_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_EpauleARG_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_EpauleARG_X_fr_base).transpose() * pa;
    
    // + Link BJambeARD
    BJambeARD_u = tau(RH_KFE_JOINT) - BJambeARD_p(AZ);
    BJambeARD_U = BJambeARD_AI.col(AZ);
    BJambeARD_D = BJambeARD_U(AZ);
    
    compute_Ia_revolute(BJambeARD_AI, BJambeARD_U, BJambeARD_D, Ia_r);  // same as: Ia_r = BJambeARD_AI - BJambeARD_U/BJambeARD_D * BJambeARD_U.transpose();
    pa = BJambeARD_p + Ia_r * BJambeARD_c + BJambeARD_U * BJambeARD_u/BJambeARD_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_BJambeARD_X_fr_HJambeARD, IaB);
    HJambeARD_AI += IaB;
    HJambeARD_p += (motionTransforms-> fr_BJambeARD_X_fr_HJambeARD).transpose() * pa;
    
    // + Link HJambeARD
    HJambeARD_u = tau(RH_HFE_JOINT) - HJambeARD_p(AZ);
    HJambeARD_U = HJambeARD_AI.col(AZ);
    HJambeARD_D = HJambeARD_U(AZ);
    
    compute_Ia_revolute(HJambeARD_AI, HJambeARD_U, HJambeARD_D, Ia_r);  // same as: Ia_r = HJambeARD_AI - HJambeARD_U/HJambeARD_D * HJambeARD_U.transpose();
    pa = HJambeARD_p + Ia_r * HJambeARD_c + HJambeARD_U * HJambeARD_u/HJambeARD_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_HJambeARD_X_fr_EpauleARD, IaB);
    EpauleARD_AI += IaB;
    EpauleARD_p += (motionTransforms-> fr_HJambeARD_X_fr_EpauleARD).transpose() * pa;
    
    // + Link EpauleARD
    EpauleARD_u = tau(RH_HAA_JOINT) - EpauleARD_p(AZ);
    EpauleARD_U = EpauleARD_AI.col(AZ);
    EpauleARD_D = EpauleARD_U(AZ);
    
    compute_Ia_revolute(EpauleARD_AI, EpauleARD_U, EpauleARD_D, Ia_r);  // same as: Ia_r = EpauleARD_AI - EpauleARD_U/EpauleARD_D * EpauleARD_U.transpose();
    pa = EpauleARD_p + Ia_r * EpauleARD_c + EpauleARD_U * EpauleARD_u/EpauleARD_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_EpauleARD_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_EpauleARD_X_fr_base).transpose() * pa;
    
    // + Link BJambeAVG
    BJambeAVG_u = tau(LF_KFE_JOINT) - BJambeAVG_p(AZ);
    BJambeAVG_U = BJambeAVG_AI.col(AZ);
    BJambeAVG_D = BJambeAVG_U(AZ);
    
    compute_Ia_revolute(BJambeAVG_AI, BJambeAVG_U, BJambeAVG_D, Ia_r);  // same as: Ia_r = BJambeAVG_AI - BJambeAVG_U/BJambeAVG_D * BJambeAVG_U.transpose();
    pa = BJambeAVG_p + Ia_r * BJambeAVG_c + BJambeAVG_U * BJambeAVG_u/BJambeAVG_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_BJambeAVG_X_fr_HJambeAVG, IaB);
    HJambeAVG_AI += IaB;
    HJambeAVG_p += (motionTransforms-> fr_BJambeAVG_X_fr_HJambeAVG).transpose() * pa;
    
    // + Link HJambeAVG
    HJambeAVG_u = tau(LF_HFE_JOINT) - HJambeAVG_p(AZ);
    HJambeAVG_U = HJambeAVG_AI.col(AZ);
    HJambeAVG_D = HJambeAVG_U(AZ);
    
    compute_Ia_revolute(HJambeAVG_AI, HJambeAVG_U, HJambeAVG_D, Ia_r);  // same as: Ia_r = HJambeAVG_AI - HJambeAVG_U/HJambeAVG_D * HJambeAVG_U.transpose();
    pa = HJambeAVG_p + Ia_r * HJambeAVG_c + HJambeAVG_U * HJambeAVG_u/HJambeAVG_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_HJambeAVG_X_fr_EpauleAVG, IaB);
    EpauleAVG_AI += IaB;
    EpauleAVG_p += (motionTransforms-> fr_HJambeAVG_X_fr_EpauleAVG).transpose() * pa;
    
    // + Link EpauleAVG
    EpauleAVG_u = tau(LF_HAA_JOINT) - EpauleAVG_p(AZ);
    EpauleAVG_U = EpauleAVG_AI.col(AZ);
    EpauleAVG_D = EpauleAVG_U(AZ);
    
    compute_Ia_revolute(EpauleAVG_AI, EpauleAVG_U, EpauleAVG_D, Ia_r);  // same as: Ia_r = EpauleAVG_AI - EpauleAVG_U/EpauleAVG_D * EpauleAVG_U.transpose();
    pa = EpauleAVG_p + Ia_r * EpauleAVG_c + EpauleAVG_U * EpauleAVG_u/EpauleAVG_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_EpauleAVG_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_EpauleAVG_X_fr_base).transpose() * pa;
    
    // + Link BJambeAVD
    BJambeAVD_u = tau(RF_KFE_JOINT) - BJambeAVD_p(AZ);
    BJambeAVD_U = BJambeAVD_AI.col(AZ);
    BJambeAVD_D = BJambeAVD_U(AZ);
    
    compute_Ia_revolute(BJambeAVD_AI, BJambeAVD_U, BJambeAVD_D, Ia_r);  // same as: Ia_r = BJambeAVD_AI - BJambeAVD_U/BJambeAVD_D * BJambeAVD_U.transpose();
    pa = BJambeAVD_p + Ia_r * BJambeAVD_c + BJambeAVD_U * BJambeAVD_u/BJambeAVD_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_BJambeAVD_X_fr_HJambeAVD, IaB);
    HJambeAVD_AI += IaB;
    HJambeAVD_p += (motionTransforms-> fr_BJambeAVD_X_fr_HJambeAVD).transpose() * pa;
    
    // + Link HJambeAVD
    HJambeAVD_u = tau(RF_HFE_JOINT) - HJambeAVD_p(AZ);
    HJambeAVD_U = HJambeAVD_AI.col(AZ);
    HJambeAVD_D = HJambeAVD_U(AZ);
    
    compute_Ia_revolute(HJambeAVD_AI, HJambeAVD_U, HJambeAVD_D, Ia_r);  // same as: Ia_r = HJambeAVD_AI - HJambeAVD_U/HJambeAVD_D * HJambeAVD_U.transpose();
    pa = HJambeAVD_p + Ia_r * HJambeAVD_c + HJambeAVD_U * HJambeAVD_u/HJambeAVD_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_HJambeAVD_X_fr_EpauleAVD, IaB);
    EpauleAVD_AI += IaB;
    EpauleAVD_p += (motionTransforms-> fr_HJambeAVD_X_fr_EpauleAVD).transpose() * pa;
    
    // + Link EpauleAVD
    EpauleAVD_u = tau(RF_HAA_JOINT) - EpauleAVD_p(AZ);
    EpauleAVD_U = EpauleAVD_AI.col(AZ);
    EpauleAVD_D = EpauleAVD_U(AZ);
    
    compute_Ia_revolute(EpauleAVD_AI, EpauleAVD_U, EpauleAVD_D, Ia_r);  // same as: Ia_r = EpauleAVD_AI - EpauleAVD_U/EpauleAVD_D * EpauleAVD_U.transpose();
    pa = EpauleAVD_p + Ia_r * EpauleAVD_c + EpauleAVD_U * EpauleAVD_u/EpauleAVD_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_EpauleAVD_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_EpauleAVD_X_fr_base).transpose() * pa;
    
    // + The acceleration of the floating base base, without gravity
    Eigen::LLT<Matrix66d> llt(base_AI);
    base_a = - llt.solve(base_p);  // base_a = - IA^-1 * base_p
    
    // ---------------------- THIRD PASS ---------------------- //
    EpauleAVD_a = (motionTransforms-> fr_EpauleAVD_X_fr_base) * base_a + EpauleAVD_c;
    qdd(RF_HAA_JOINT) = (EpauleAVD_u - EpauleAVD_U.dot(EpauleAVD_a)) / EpauleAVD_D;
    EpauleAVD_a(AZ) += qdd(RF_HAA_JOINT);
    
    HJambeAVD_a = (motionTransforms-> fr_HJambeAVD_X_fr_EpauleAVD) * EpauleAVD_a + HJambeAVD_c;
    qdd(RF_HFE_JOINT) = (HJambeAVD_u - HJambeAVD_U.dot(HJambeAVD_a)) / HJambeAVD_D;
    HJambeAVD_a(AZ) += qdd(RF_HFE_JOINT);
    
    BJambeAVD_a = (motionTransforms-> fr_BJambeAVD_X_fr_HJambeAVD) * HJambeAVD_a + BJambeAVD_c;
    qdd(RF_KFE_JOINT) = (BJambeAVD_u - BJambeAVD_U.dot(BJambeAVD_a)) / BJambeAVD_D;
    BJambeAVD_a(AZ) += qdd(RF_KFE_JOINT);
    
    EpauleAVG_a = (motionTransforms-> fr_EpauleAVG_X_fr_base) * base_a + EpauleAVG_c;
    qdd(LF_HAA_JOINT) = (EpauleAVG_u - EpauleAVG_U.dot(EpauleAVG_a)) / EpauleAVG_D;
    EpauleAVG_a(AZ) += qdd(LF_HAA_JOINT);
    
    HJambeAVG_a = (motionTransforms-> fr_HJambeAVG_X_fr_EpauleAVG) * EpauleAVG_a + HJambeAVG_c;
    qdd(LF_HFE_JOINT) = (HJambeAVG_u - HJambeAVG_U.dot(HJambeAVG_a)) / HJambeAVG_D;
    HJambeAVG_a(AZ) += qdd(LF_HFE_JOINT);
    
    BJambeAVG_a = (motionTransforms-> fr_BJambeAVG_X_fr_HJambeAVG) * HJambeAVG_a + BJambeAVG_c;
    qdd(LF_KFE_JOINT) = (BJambeAVG_u - BJambeAVG_U.dot(BJambeAVG_a)) / BJambeAVG_D;
    BJambeAVG_a(AZ) += qdd(LF_KFE_JOINT);
    
    EpauleARD_a = (motionTransforms-> fr_EpauleARD_X_fr_base) * base_a + EpauleARD_c;
    qdd(RH_HAA_JOINT) = (EpauleARD_u - EpauleARD_U.dot(EpauleARD_a)) / EpauleARD_D;
    EpauleARD_a(AZ) += qdd(RH_HAA_JOINT);
    
    HJambeARD_a = (motionTransforms-> fr_HJambeARD_X_fr_EpauleARD) * EpauleARD_a + HJambeARD_c;
    qdd(RH_HFE_JOINT) = (HJambeARD_u - HJambeARD_U.dot(HJambeARD_a)) / HJambeARD_D;
    HJambeARD_a(AZ) += qdd(RH_HFE_JOINT);
    
    BJambeARD_a = (motionTransforms-> fr_BJambeARD_X_fr_HJambeARD) * HJambeARD_a + BJambeARD_c;
    qdd(RH_KFE_JOINT) = (BJambeARD_u - BJambeARD_U.dot(BJambeARD_a)) / BJambeARD_D;
    BJambeARD_a(AZ) += qdd(RH_KFE_JOINT);
    
    EpauleARG_a = (motionTransforms-> fr_EpauleARG_X_fr_base) * base_a + EpauleARG_c;
    qdd(LH_HAA_JOINT) = (EpauleARG_u - EpauleARG_U.dot(EpauleARG_a)) / EpauleARG_D;
    EpauleARG_a(AZ) += qdd(LH_HAA_JOINT);
    
    HJambeARG_a = (motionTransforms-> fr_HJambeARG_X_fr_EpauleARG) * EpauleARG_a + HJambeARG_c;
    qdd(LH_HFE_JOINT) = (HJambeARG_u - HJambeARG_U.dot(HJambeARG_a)) / HJambeARG_D;
    HJambeARG_a(AZ) += qdd(LH_HFE_JOINT);
    
    BJambeARG_a = (motionTransforms-> fr_BJambeARG_X_fr_HJambeARG) * HJambeARG_a + BJambeARG_c;
    qdd(LH_KFE_JOINT) = (BJambeARG_u - BJambeARG_U.dot(BJambeARG_a)) / BJambeARG_D;
    BJambeARG_a(AZ) += qdd(LH_KFE_JOINT);
    
    
    // + Add gravity to the acceleration of the floating base
    base_a += g;
}
