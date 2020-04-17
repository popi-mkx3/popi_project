#include "transforms.h"

using namespace iit::popi;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_EpauleAVD_X_fr_base(),
    fr_base_X_fr_EpauleAVD(),
    fr_HJambeAVD_X_fr_EpauleAVD(),
    fr_EpauleAVD_X_fr_HJambeAVD(),
    fr_BJambeAVD_X_fr_HJambeAVD(),
    fr_HJambeAVD_X_fr_BJambeAVD(),
    fr_EpauleAVG_X_fr_base(),
    fr_base_X_fr_EpauleAVG(),
    fr_HJambeAVG_X_fr_EpauleAVG(),
    fr_EpauleAVG_X_fr_HJambeAVG(),
    fr_BJambeAVG_X_fr_HJambeAVG(),
    fr_HJambeAVG_X_fr_BJambeAVG(),
    fr_EpauleARD_X_fr_base(),
    fr_base_X_fr_EpauleARD(),
    fr_HJambeARD_X_fr_EpauleARD(),
    fr_EpauleARD_X_fr_HJambeARD(),
    fr_BJambeARD_X_fr_HJambeARD(),
    fr_HJambeARD_X_fr_BJambeARD(),
    fr_EpauleARG_X_fr_base(),
    fr_base_X_fr_EpauleARG(),
    fr_HJambeARG_X_fr_EpauleARG(),
    fr_EpauleARG_X_fr_HJambeARG(),
    fr_BJambeARG_X_fr_HJambeARG(),
    fr_HJambeARG_X_fr_BJambeARG()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_EpauleAVD_X_fr_base(),
    fr_base_X_fr_EpauleAVD(),
    fr_HJambeAVD_X_fr_EpauleAVD(),
    fr_EpauleAVD_X_fr_HJambeAVD(),
    fr_BJambeAVD_X_fr_HJambeAVD(),
    fr_HJambeAVD_X_fr_BJambeAVD(),
    fr_EpauleAVG_X_fr_base(),
    fr_base_X_fr_EpauleAVG(),
    fr_HJambeAVG_X_fr_EpauleAVG(),
    fr_EpauleAVG_X_fr_HJambeAVG(),
    fr_BJambeAVG_X_fr_HJambeAVG(),
    fr_HJambeAVG_X_fr_BJambeAVG(),
    fr_EpauleARD_X_fr_base(),
    fr_base_X_fr_EpauleARD(),
    fr_HJambeARD_X_fr_EpauleARD(),
    fr_EpauleARD_X_fr_HJambeARD(),
    fr_BJambeARD_X_fr_HJambeARD(),
    fr_HJambeARD_X_fr_BJambeARD(),
    fr_EpauleARG_X_fr_base(),
    fr_base_X_fr_EpauleARG(),
    fr_HJambeARG_X_fr_EpauleARG(),
    fr_EpauleARG_X_fr_HJambeARG(),
    fr_BJambeARG_X_fr_HJambeARG(),
    fr_HJambeARG_X_fr_BJambeARG()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_EpauleAVD_X_fr_base(),
    fr_base_X_fr_EpauleAVD(),
    fr_HJambeAVD_X_fr_EpauleAVD(),
    fr_EpauleAVD_X_fr_HJambeAVD(),
    fr_BJambeAVD_X_fr_HJambeAVD(),
    fr_HJambeAVD_X_fr_BJambeAVD(),
    fr_EpauleAVG_X_fr_base(),
    fr_base_X_fr_EpauleAVG(),
    fr_HJambeAVG_X_fr_EpauleAVG(),
    fr_EpauleAVG_X_fr_HJambeAVG(),
    fr_BJambeAVG_X_fr_HJambeAVG(),
    fr_HJambeAVG_X_fr_BJambeAVG(),
    fr_EpauleARD_X_fr_base(),
    fr_base_X_fr_EpauleARD(),
    fr_HJambeARD_X_fr_EpauleARD(),
    fr_EpauleARD_X_fr_HJambeARD(),
    fr_BJambeARD_X_fr_HJambeARD(),
    fr_HJambeARD_X_fr_BJambeARD(),
    fr_EpauleARG_X_fr_base(),
    fr_base_X_fr_EpauleARG(),
    fr_HJambeARG_X_fr_EpauleARG(),
    fr_EpauleARG_X_fr_HJambeARG(),
    fr_BJambeARG_X_fr_HJambeARG(),
    fr_HJambeARG_X_fr_BJambeARG()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_EpauleAVD_X_fr_base::Type_fr_EpauleAVD_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tz_rf_haa_joint;    // Maxima DSL: -_k__tz_rf_haa_joint
    (*this)(5,2) =  ty_rf_haa_joint;    // Maxima DSL: _k__ty_rf_haa_joint
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleAVD_X_fr_base& MotionTransforms::Type_fr_EpauleAVD_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_rf_haa_joint  = ScalarTraits::sin( q(RF_HAA_JOINT) );
    Scalar cos_q_rf_haa_joint  = ScalarTraits::cos( q(RF_HAA_JOINT) );
    (*this)(0,1) = sin_q_rf_haa_joint;
    (*this)(0,2) = cos_q_rf_haa_joint;
    (*this)(1,1) = cos_q_rf_haa_joint;
    (*this)(1,2) = -sin_q_rf_haa_joint;
    (*this)(3,0) = ( ty_rf_haa_joint * cos_q_rf_haa_joint)-( tz_rf_haa_joint * sin_q_rf_haa_joint);
    (*this)(3,1) = - tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(3,2) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(3,4) = sin_q_rf_haa_joint;
    (*this)(3,5) = cos_q_rf_haa_joint;
    (*this)(4,0) = (- ty_rf_haa_joint * sin_q_rf_haa_joint)-( tz_rf_haa_joint * cos_q_rf_haa_joint);
    (*this)(4,1) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(4,2) =  tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(4,4) = cos_q_rf_haa_joint;
    (*this)(4,5) = -sin_q_rf_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_EpauleAVD::Type_fr_base_X_fr_EpauleAVD()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = -1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = -1.0;
    (*this)(4,2) = - tz_rf_haa_joint;    // Maxima DSL: -_k__tz_rf_haa_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) =  ty_rf_haa_joint;    // Maxima DSL: _k__ty_rf_haa_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_EpauleAVD& MotionTransforms::Type_fr_base_X_fr_EpauleAVD::update(const state_t& q)
{
    Scalar sin_q_rf_haa_joint  = ScalarTraits::sin( q(RF_HAA_JOINT) );
    Scalar cos_q_rf_haa_joint  = ScalarTraits::cos( q(RF_HAA_JOINT) );
    (*this)(1,0) = sin_q_rf_haa_joint;
    (*this)(1,1) = cos_q_rf_haa_joint;
    (*this)(2,0) = cos_q_rf_haa_joint;
    (*this)(2,1) = -sin_q_rf_haa_joint;
    (*this)(3,0) = ( ty_rf_haa_joint * cos_q_rf_haa_joint)-( tz_rf_haa_joint * sin_q_rf_haa_joint);
    (*this)(3,1) = (- ty_rf_haa_joint * sin_q_rf_haa_joint)-( tz_rf_haa_joint * cos_q_rf_haa_joint);
    (*this)(4,0) = - tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(4,1) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(4,3) = sin_q_rf_haa_joint;
    (*this)(4,4) = cos_q_rf_haa_joint;
    (*this)(5,0) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(5,1) =  tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(5,3) = cos_q_rf_haa_joint;
    (*this)(5,4) = -sin_q_rf_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD::Type_fr_HJambeAVD_X_fr_EpauleAVD()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD& MotionTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD::update(const state_t& q)
{
    Scalar sin_q_rf_hfe_joint  = ScalarTraits::sin( q(RF_HFE_JOINT) );
    Scalar cos_q_rf_hfe_joint  = ScalarTraits::cos( q(RF_HFE_JOINT) );
    (*this)(0,0) = sin_q_rf_hfe_joint;
    (*this)(0,2) = -cos_q_rf_hfe_joint;
    (*this)(1,0) = cos_q_rf_hfe_joint;
    (*this)(1,2) = sin_q_rf_hfe_joint;
    (*this)(3,0) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(3,2) = - ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(3,3) = sin_q_rf_hfe_joint;
    (*this)(3,5) = -cos_q_rf_hfe_joint;
    (*this)(4,0) =  ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(4,2) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(4,3) = cos_q_rf_hfe_joint;
    (*this)(4,5) = sin_q_rf_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD::Type_fr_EpauleAVD_X_fr_HJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD& MotionTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_hfe_joint  = ScalarTraits::sin( q(RF_HFE_JOINT) );
    Scalar cos_q_rf_hfe_joint  = ScalarTraits::cos( q(RF_HFE_JOINT) );
    (*this)(0,0) = sin_q_rf_hfe_joint;
    (*this)(0,1) = cos_q_rf_hfe_joint;
    (*this)(2,0) = -cos_q_rf_hfe_joint;
    (*this)(2,1) = sin_q_rf_hfe_joint;
    (*this)(3,0) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(3,1) =  ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(3,3) = sin_q_rf_hfe_joint;
    (*this)(3,4) = cos_q_rf_hfe_joint;
    (*this)(5,0) = - ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(5,1) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(5,3) = -cos_q_rf_hfe_joint;
    (*this)(5,4) = sin_q_rf_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD::Type_fr_BJambeAVD_X_fr_HJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_rf_kfe_joint;    // Maxima DSL: -_k__ty_rf_kfe_joint
    (*this)(5,1) =  tx_rf_kfe_joint;    // Maxima DSL: _k__tx_rf_kfe_joint
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD& MotionTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_kfe_joint  = ScalarTraits::sin( q(RF_KFE_JOINT) );
    Scalar cos_q_rf_kfe_joint  = ScalarTraits::cos( q(RF_KFE_JOINT) );
    (*this)(0,0) = cos_q_rf_kfe_joint;
    (*this)(0,1) = -sin_q_rf_kfe_joint;
    (*this)(1,0) = -sin_q_rf_kfe_joint;
    (*this)(1,1) = -cos_q_rf_kfe_joint;
    (*this)(3,2) = (- tx_rf_kfe_joint * sin_q_rf_kfe_joint)-( ty_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(3,3) = cos_q_rf_kfe_joint;
    (*this)(3,4) = -sin_q_rf_kfe_joint;
    (*this)(4,2) = ( ty_rf_kfe_joint * sin_q_rf_kfe_joint)-( tx_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(4,3) = -sin_q_rf_kfe_joint;
    (*this)(4,4) = -cos_q_rf_kfe_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD::Type_fr_HJambeAVD_X_fr_BJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - ty_rf_kfe_joint;    // Maxima DSL: -_k__ty_rf_kfe_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_rf_kfe_joint;    // Maxima DSL: _k__tx_rf_kfe_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD& MotionTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_kfe_joint  = ScalarTraits::sin( q(RF_KFE_JOINT) );
    Scalar cos_q_rf_kfe_joint  = ScalarTraits::cos( q(RF_KFE_JOINT) );
    (*this)(0,0) = cos_q_rf_kfe_joint;
    (*this)(0,1) = -sin_q_rf_kfe_joint;
    (*this)(1,0) = -sin_q_rf_kfe_joint;
    (*this)(1,1) = -cos_q_rf_kfe_joint;
    (*this)(3,3) = cos_q_rf_kfe_joint;
    (*this)(3,4) = -sin_q_rf_kfe_joint;
    (*this)(4,3) = -sin_q_rf_kfe_joint;
    (*this)(4,4) = -cos_q_rf_kfe_joint;
    (*this)(5,0) = (- tx_rf_kfe_joint * sin_q_rf_kfe_joint)-( ty_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(5,1) = ( ty_rf_kfe_joint * sin_q_rf_kfe_joint)-( tx_rf_kfe_joint * cos_q_rf_kfe_joint);
    return *this;
}
MotionTransforms::Type_fr_EpauleAVG_X_fr_base::Type_fr_EpauleAVG_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tz_lf_haa_joint;    // Maxima DSL: _k__tz_lf_haa_joint
    (*this)(5,2) = - ty_lf_haa_joint;    // Maxima DSL: -_k__ty_lf_haa_joint
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleAVG_X_fr_base& MotionTransforms::Type_fr_EpauleAVG_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_lf_haa_joint  = ScalarTraits::sin( q(LF_HAA_JOINT) );
    Scalar cos_q_lf_haa_joint  = ScalarTraits::cos( q(LF_HAA_JOINT) );
    (*this)(0,1) = sin_q_lf_haa_joint;
    (*this)(0,2) = -cos_q_lf_haa_joint;
    (*this)(1,1) = cos_q_lf_haa_joint;
    (*this)(1,2) = sin_q_lf_haa_joint;
    (*this)(3,0) = (- tz_lf_haa_joint * sin_q_lf_haa_joint)-( ty_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(3,1) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(3,2) =  tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(3,4) = sin_q_lf_haa_joint;
    (*this)(3,5) = -cos_q_lf_haa_joint;
    (*this)(4,0) = ( ty_lf_haa_joint * sin_q_lf_haa_joint)-( tz_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(4,1) = - tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(4,2) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(4,4) = cos_q_lf_haa_joint;
    (*this)(4,5) = sin_q_lf_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_EpauleAVG::Type_fr_base_X_fr_EpauleAVG()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) =  tz_lf_haa_joint;    // Maxima DSL: _k__tz_lf_haa_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_lf_haa_joint;    // Maxima DSL: -_k__ty_lf_haa_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_EpauleAVG& MotionTransforms::Type_fr_base_X_fr_EpauleAVG::update(const state_t& q)
{
    Scalar sin_q_lf_haa_joint  = ScalarTraits::sin( q(LF_HAA_JOINT) );
    Scalar cos_q_lf_haa_joint  = ScalarTraits::cos( q(LF_HAA_JOINT) );
    (*this)(1,0) = sin_q_lf_haa_joint;
    (*this)(1,1) = cos_q_lf_haa_joint;
    (*this)(2,0) = -cos_q_lf_haa_joint;
    (*this)(2,1) = sin_q_lf_haa_joint;
    (*this)(3,0) = (- tz_lf_haa_joint * sin_q_lf_haa_joint)-( ty_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(3,1) = ( ty_lf_haa_joint * sin_q_lf_haa_joint)-( tz_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(4,0) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(4,1) = - tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(4,3) = sin_q_lf_haa_joint;
    (*this)(4,4) = cos_q_lf_haa_joint;
    (*this)(5,0) =  tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(5,1) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(5,3) = -cos_q_lf_haa_joint;
    (*this)(5,4) = sin_q_lf_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG::Type_fr_HJambeAVG_X_fr_EpauleAVG()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG& MotionTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG::update(const state_t& q)
{
    Scalar sin_q_lf_hfe_joint  = ScalarTraits::sin( q(LF_HFE_JOINT) );
    Scalar cos_q_lf_hfe_joint  = ScalarTraits::cos( q(LF_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lf_hfe_joint;
    (*this)(0,2) = cos_q_lf_hfe_joint;
    (*this)(1,0) = -cos_q_lf_hfe_joint;
    (*this)(1,2) = -sin_q_lf_hfe_joint;
    (*this)(3,0) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(3,2) =  ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(3,3) = -sin_q_lf_hfe_joint;
    (*this)(3,5) = cos_q_lf_hfe_joint;
    (*this)(4,0) = - ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(4,2) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(4,3) = -cos_q_lf_hfe_joint;
    (*this)(4,5) = -sin_q_lf_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG::Type_fr_EpauleAVG_X_fr_HJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG& MotionTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_hfe_joint  = ScalarTraits::sin( q(LF_HFE_JOINT) );
    Scalar cos_q_lf_hfe_joint  = ScalarTraits::cos( q(LF_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lf_hfe_joint;
    (*this)(0,1) = -cos_q_lf_hfe_joint;
    (*this)(2,0) = cos_q_lf_hfe_joint;
    (*this)(2,1) = -sin_q_lf_hfe_joint;
    (*this)(3,0) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(3,1) = - ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(3,3) = -sin_q_lf_hfe_joint;
    (*this)(3,4) = -cos_q_lf_hfe_joint;
    (*this)(5,0) =  ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(5,1) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(5,3) = cos_q_lf_hfe_joint;
    (*this)(5,4) = -sin_q_lf_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG::Type_fr_BJambeAVG_X_fr_HJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_lf_kfe_joint;    // Maxima DSL: -_k__ty_lf_kfe_joint
    (*this)(5,1) =  tx_lf_kfe_joint;    // Maxima DSL: _k__tx_lf_kfe_joint
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG& MotionTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_kfe_joint  = ScalarTraits::sin( q(LF_KFE_JOINT) );
    Scalar cos_q_lf_kfe_joint  = ScalarTraits::cos( q(LF_KFE_JOINT) );
    (*this)(0,0) = cos_q_lf_kfe_joint;
    (*this)(0,1) = -sin_q_lf_kfe_joint;
    (*this)(1,0) = -sin_q_lf_kfe_joint;
    (*this)(1,1) = -cos_q_lf_kfe_joint;
    (*this)(3,2) = (- tx_lf_kfe_joint * sin_q_lf_kfe_joint)-( ty_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(3,3) = cos_q_lf_kfe_joint;
    (*this)(3,4) = -sin_q_lf_kfe_joint;
    (*this)(4,2) = ( ty_lf_kfe_joint * sin_q_lf_kfe_joint)-( tx_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(4,3) = -sin_q_lf_kfe_joint;
    (*this)(4,4) = -cos_q_lf_kfe_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG::Type_fr_HJambeAVG_X_fr_BJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - ty_lf_kfe_joint;    // Maxima DSL: -_k__ty_lf_kfe_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_lf_kfe_joint;    // Maxima DSL: _k__tx_lf_kfe_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG& MotionTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_kfe_joint  = ScalarTraits::sin( q(LF_KFE_JOINT) );
    Scalar cos_q_lf_kfe_joint  = ScalarTraits::cos( q(LF_KFE_JOINT) );
    (*this)(0,0) = cos_q_lf_kfe_joint;
    (*this)(0,1) = -sin_q_lf_kfe_joint;
    (*this)(1,0) = -sin_q_lf_kfe_joint;
    (*this)(1,1) = -cos_q_lf_kfe_joint;
    (*this)(3,3) = cos_q_lf_kfe_joint;
    (*this)(3,4) = -sin_q_lf_kfe_joint;
    (*this)(4,3) = -sin_q_lf_kfe_joint;
    (*this)(4,4) = -cos_q_lf_kfe_joint;
    (*this)(5,0) = (- tx_lf_kfe_joint * sin_q_lf_kfe_joint)-( ty_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(5,1) = ( ty_lf_kfe_joint * sin_q_lf_kfe_joint)-( tx_lf_kfe_joint * cos_q_lf_kfe_joint);
    return *this;
}
MotionTransforms::Type_fr_EpauleARD_X_fr_base::Type_fr_EpauleARD_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tz_rh_haa_joint;    // Maxima DSL: -_k__tz_rh_haa_joint
    (*this)(5,2) =  ty_rh_haa_joint;    // Maxima DSL: _k__ty_rh_haa_joint
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleARD_X_fr_base& MotionTransforms::Type_fr_EpauleARD_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_rh_haa_joint  = ScalarTraits::sin( q(RH_HAA_JOINT) );
    Scalar cos_q_rh_haa_joint  = ScalarTraits::cos( q(RH_HAA_JOINT) );
    (*this)(0,1) = sin_q_rh_haa_joint;
    (*this)(0,2) = cos_q_rh_haa_joint;
    (*this)(1,1) = cos_q_rh_haa_joint;
    (*this)(1,2) = -sin_q_rh_haa_joint;
    (*this)(3,0) = ( ty_rh_haa_joint * cos_q_rh_haa_joint)-( tz_rh_haa_joint * sin_q_rh_haa_joint);
    (*this)(3,1) = - tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(3,2) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(3,4) = sin_q_rh_haa_joint;
    (*this)(3,5) = cos_q_rh_haa_joint;
    (*this)(4,0) = (- ty_rh_haa_joint * sin_q_rh_haa_joint)-( tz_rh_haa_joint * cos_q_rh_haa_joint);
    (*this)(4,1) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(4,2) =  tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(4,4) = cos_q_rh_haa_joint;
    (*this)(4,5) = -sin_q_rh_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_EpauleARD::Type_fr_base_X_fr_EpauleARD()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = -1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = -1.0;
    (*this)(4,2) = - tz_rh_haa_joint;    // Maxima DSL: -_k__tz_rh_haa_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) =  ty_rh_haa_joint;    // Maxima DSL: _k__ty_rh_haa_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_EpauleARD& MotionTransforms::Type_fr_base_X_fr_EpauleARD::update(const state_t& q)
{
    Scalar sin_q_rh_haa_joint  = ScalarTraits::sin( q(RH_HAA_JOINT) );
    Scalar cos_q_rh_haa_joint  = ScalarTraits::cos( q(RH_HAA_JOINT) );
    (*this)(1,0) = sin_q_rh_haa_joint;
    (*this)(1,1) = cos_q_rh_haa_joint;
    (*this)(2,0) = cos_q_rh_haa_joint;
    (*this)(2,1) = -sin_q_rh_haa_joint;
    (*this)(3,0) = ( ty_rh_haa_joint * cos_q_rh_haa_joint)-( tz_rh_haa_joint * sin_q_rh_haa_joint);
    (*this)(3,1) = (- ty_rh_haa_joint * sin_q_rh_haa_joint)-( tz_rh_haa_joint * cos_q_rh_haa_joint);
    (*this)(4,0) = - tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(4,1) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(4,3) = sin_q_rh_haa_joint;
    (*this)(4,4) = cos_q_rh_haa_joint;
    (*this)(5,0) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(5,1) =  tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(5,3) = cos_q_rh_haa_joint;
    (*this)(5,4) = -sin_q_rh_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeARD_X_fr_EpauleARD::Type_fr_HJambeARD_X_fr_EpauleARD()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_HJambeARD_X_fr_EpauleARD& MotionTransforms::Type_fr_HJambeARD_X_fr_EpauleARD::update(const state_t& q)
{
    Scalar sin_q_rh_hfe_joint  = ScalarTraits::sin( q(RH_HFE_JOINT) );
    Scalar cos_q_rh_hfe_joint  = ScalarTraits::cos( q(RH_HFE_JOINT) );
    (*this)(0,0) = sin_q_rh_hfe_joint;
    (*this)(0,2) = -cos_q_rh_hfe_joint;
    (*this)(1,0) = cos_q_rh_hfe_joint;
    (*this)(1,2) = sin_q_rh_hfe_joint;
    (*this)(3,0) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(3,2) = - ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(3,3) = sin_q_rh_hfe_joint;
    (*this)(3,5) = -cos_q_rh_hfe_joint;
    (*this)(4,0) =  ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(4,2) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(4,3) = cos_q_rh_hfe_joint;
    (*this)(4,5) = sin_q_rh_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_EpauleARD_X_fr_HJambeARD::Type_fr_EpauleARD_X_fr_HJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleARD_X_fr_HJambeARD& MotionTransforms::Type_fr_EpauleARD_X_fr_HJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_hfe_joint  = ScalarTraits::sin( q(RH_HFE_JOINT) );
    Scalar cos_q_rh_hfe_joint  = ScalarTraits::cos( q(RH_HFE_JOINT) );
    (*this)(0,0) = sin_q_rh_hfe_joint;
    (*this)(0,1) = cos_q_rh_hfe_joint;
    (*this)(2,0) = -cos_q_rh_hfe_joint;
    (*this)(2,1) = sin_q_rh_hfe_joint;
    (*this)(3,0) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(3,1) =  ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(3,3) = sin_q_rh_hfe_joint;
    (*this)(3,4) = cos_q_rh_hfe_joint;
    (*this)(5,0) = - ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(5,1) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(5,3) = -cos_q_rh_hfe_joint;
    (*this)(5,4) = sin_q_rh_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_BJambeARD_X_fr_HJambeARD::Type_fr_BJambeARD_X_fr_HJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_rh_kfe_joint;    // Maxima DSL: -_k__ty_rh_kfe_joint
    (*this)(5,1) =  tx_rh_kfe_joint;    // Maxima DSL: _k__tx_rh_kfe_joint
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_BJambeARD_X_fr_HJambeARD& MotionTransforms::Type_fr_BJambeARD_X_fr_HJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_kfe_joint  = ScalarTraits::sin( q(RH_KFE_JOINT) );
    Scalar cos_q_rh_kfe_joint  = ScalarTraits::cos( q(RH_KFE_JOINT) );
    (*this)(0,0) = cos_q_rh_kfe_joint;
    (*this)(0,1) = -sin_q_rh_kfe_joint;
    (*this)(1,0) = -sin_q_rh_kfe_joint;
    (*this)(1,1) = -cos_q_rh_kfe_joint;
    (*this)(3,2) = (- tx_rh_kfe_joint * sin_q_rh_kfe_joint)-( ty_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(3,3) = cos_q_rh_kfe_joint;
    (*this)(3,4) = -sin_q_rh_kfe_joint;
    (*this)(4,2) = ( ty_rh_kfe_joint * sin_q_rh_kfe_joint)-( tx_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(4,3) = -sin_q_rh_kfe_joint;
    (*this)(4,4) = -cos_q_rh_kfe_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeARD_X_fr_BJambeARD::Type_fr_HJambeARD_X_fr_BJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - ty_rh_kfe_joint;    // Maxima DSL: -_k__ty_rh_kfe_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_rh_kfe_joint;    // Maxima DSL: _k__tx_rh_kfe_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_HJambeARD_X_fr_BJambeARD& MotionTransforms::Type_fr_HJambeARD_X_fr_BJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_kfe_joint  = ScalarTraits::sin( q(RH_KFE_JOINT) );
    Scalar cos_q_rh_kfe_joint  = ScalarTraits::cos( q(RH_KFE_JOINT) );
    (*this)(0,0) = cos_q_rh_kfe_joint;
    (*this)(0,1) = -sin_q_rh_kfe_joint;
    (*this)(1,0) = -sin_q_rh_kfe_joint;
    (*this)(1,1) = -cos_q_rh_kfe_joint;
    (*this)(3,3) = cos_q_rh_kfe_joint;
    (*this)(3,4) = -sin_q_rh_kfe_joint;
    (*this)(4,3) = -sin_q_rh_kfe_joint;
    (*this)(4,4) = -cos_q_rh_kfe_joint;
    (*this)(5,0) = (- tx_rh_kfe_joint * sin_q_rh_kfe_joint)-( ty_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(5,1) = ( ty_rh_kfe_joint * sin_q_rh_kfe_joint)-( tx_rh_kfe_joint * cos_q_rh_kfe_joint);
    return *this;
}
MotionTransforms::Type_fr_EpauleARG_X_fr_base::Type_fr_EpauleARG_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tz_lh_haa_joint;    // Maxima DSL: _k__tz_lh_haa_joint
    (*this)(5,2) = - ty_lh_haa_joint;    // Maxima DSL: -_k__ty_lh_haa_joint
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleARG_X_fr_base& MotionTransforms::Type_fr_EpauleARG_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_lh_haa_joint  = ScalarTraits::sin( q(LH_HAA_JOINT) );
    Scalar cos_q_lh_haa_joint  = ScalarTraits::cos( q(LH_HAA_JOINT) );
    (*this)(0,1) = sin_q_lh_haa_joint;
    (*this)(0,2) = -cos_q_lh_haa_joint;
    (*this)(1,1) = cos_q_lh_haa_joint;
    (*this)(1,2) = sin_q_lh_haa_joint;
    (*this)(3,0) = (- tz_lh_haa_joint * sin_q_lh_haa_joint)-( ty_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(3,1) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(3,2) =  tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(3,4) = sin_q_lh_haa_joint;
    (*this)(3,5) = -cos_q_lh_haa_joint;
    (*this)(4,0) = ( ty_lh_haa_joint * sin_q_lh_haa_joint)-( tz_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(4,1) = - tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(4,2) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(4,4) = cos_q_lh_haa_joint;
    (*this)(4,5) = sin_q_lh_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_EpauleARG::Type_fr_base_X_fr_EpauleARG()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) =  tz_lh_haa_joint;    // Maxima DSL: _k__tz_lh_haa_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_lh_haa_joint;    // Maxima DSL: -_k__ty_lh_haa_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_EpauleARG& MotionTransforms::Type_fr_base_X_fr_EpauleARG::update(const state_t& q)
{
    Scalar sin_q_lh_haa_joint  = ScalarTraits::sin( q(LH_HAA_JOINT) );
    Scalar cos_q_lh_haa_joint  = ScalarTraits::cos( q(LH_HAA_JOINT) );
    (*this)(1,0) = sin_q_lh_haa_joint;
    (*this)(1,1) = cos_q_lh_haa_joint;
    (*this)(2,0) = -cos_q_lh_haa_joint;
    (*this)(2,1) = sin_q_lh_haa_joint;
    (*this)(3,0) = (- tz_lh_haa_joint * sin_q_lh_haa_joint)-( ty_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(3,1) = ( ty_lh_haa_joint * sin_q_lh_haa_joint)-( tz_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(4,0) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(4,1) = - tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(4,3) = sin_q_lh_haa_joint;
    (*this)(4,4) = cos_q_lh_haa_joint;
    (*this)(5,0) =  tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(5,1) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(5,3) = -cos_q_lh_haa_joint;
    (*this)(5,4) = sin_q_lh_haa_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeARG_X_fr_EpauleARG::Type_fr_HJambeARG_X_fr_EpauleARG()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_HJambeARG_X_fr_EpauleARG& MotionTransforms::Type_fr_HJambeARG_X_fr_EpauleARG::update(const state_t& q)
{
    Scalar sin_q_lh_hfe_joint  = ScalarTraits::sin( q(LH_HFE_JOINT) );
    Scalar cos_q_lh_hfe_joint  = ScalarTraits::cos( q(LH_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lh_hfe_joint;
    (*this)(0,2) = cos_q_lh_hfe_joint;
    (*this)(1,0) = -cos_q_lh_hfe_joint;
    (*this)(1,2) = -sin_q_lh_hfe_joint;
    (*this)(3,0) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(3,2) =  ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(3,3) = -sin_q_lh_hfe_joint;
    (*this)(3,5) = cos_q_lh_hfe_joint;
    (*this)(4,0) = - ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(4,2) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(4,3) = -cos_q_lh_hfe_joint;
    (*this)(4,5) = -sin_q_lh_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_EpauleARG_X_fr_HJambeARG::Type_fr_EpauleARG_X_fr_HJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_EpauleARG_X_fr_HJambeARG& MotionTransforms::Type_fr_EpauleARG_X_fr_HJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_hfe_joint  = ScalarTraits::sin( q(LH_HFE_JOINT) );
    Scalar cos_q_lh_hfe_joint  = ScalarTraits::cos( q(LH_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lh_hfe_joint;
    (*this)(0,1) = -cos_q_lh_hfe_joint;
    (*this)(2,0) = cos_q_lh_hfe_joint;
    (*this)(2,1) = -sin_q_lh_hfe_joint;
    (*this)(3,0) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(3,1) = - ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(3,3) = -sin_q_lh_hfe_joint;
    (*this)(3,4) = -cos_q_lh_hfe_joint;
    (*this)(5,0) =  ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(5,1) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(5,3) = cos_q_lh_hfe_joint;
    (*this)(5,4) = -sin_q_lh_hfe_joint;
    return *this;
}
MotionTransforms::Type_fr_BJambeARG_X_fr_HJambeARG::Type_fr_BJambeARG_X_fr_HJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_lh_kfe_joint;    // Maxima DSL: -_k__ty_lh_kfe_joint
    (*this)(5,1) =  tx_lh_kfe_joint;    // Maxima DSL: _k__tx_lh_kfe_joint
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_BJambeARG_X_fr_HJambeARG& MotionTransforms::Type_fr_BJambeARG_X_fr_HJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_kfe_joint  = ScalarTraits::sin( q(LH_KFE_JOINT) );
    Scalar cos_q_lh_kfe_joint  = ScalarTraits::cos( q(LH_KFE_JOINT) );
    (*this)(0,0) = cos_q_lh_kfe_joint;
    (*this)(0,1) = -sin_q_lh_kfe_joint;
    (*this)(1,0) = -sin_q_lh_kfe_joint;
    (*this)(1,1) = -cos_q_lh_kfe_joint;
    (*this)(3,2) = (- tx_lh_kfe_joint * sin_q_lh_kfe_joint)-( ty_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(3,3) = cos_q_lh_kfe_joint;
    (*this)(3,4) = -sin_q_lh_kfe_joint;
    (*this)(4,2) = ( ty_lh_kfe_joint * sin_q_lh_kfe_joint)-( tx_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(4,3) = -sin_q_lh_kfe_joint;
    (*this)(4,4) = -cos_q_lh_kfe_joint;
    return *this;
}
MotionTransforms::Type_fr_HJambeARG_X_fr_BJambeARG::Type_fr_HJambeARG_X_fr_BJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - ty_lh_kfe_joint;    // Maxima DSL: -_k__ty_lh_kfe_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_lh_kfe_joint;    // Maxima DSL: _k__tx_lh_kfe_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_HJambeARG_X_fr_BJambeARG& MotionTransforms::Type_fr_HJambeARG_X_fr_BJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_kfe_joint  = ScalarTraits::sin( q(LH_KFE_JOINT) );
    Scalar cos_q_lh_kfe_joint  = ScalarTraits::cos( q(LH_KFE_JOINT) );
    (*this)(0,0) = cos_q_lh_kfe_joint;
    (*this)(0,1) = -sin_q_lh_kfe_joint;
    (*this)(1,0) = -sin_q_lh_kfe_joint;
    (*this)(1,1) = -cos_q_lh_kfe_joint;
    (*this)(3,3) = cos_q_lh_kfe_joint;
    (*this)(3,4) = -sin_q_lh_kfe_joint;
    (*this)(4,3) = -sin_q_lh_kfe_joint;
    (*this)(4,4) = -cos_q_lh_kfe_joint;
    (*this)(5,0) = (- tx_lh_kfe_joint * sin_q_lh_kfe_joint)-( ty_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(5,1) = ( ty_lh_kfe_joint * sin_q_lh_kfe_joint)-( tx_lh_kfe_joint * cos_q_lh_kfe_joint);
    return *this;
}

ForceTransforms::Type_fr_EpauleAVD_X_fr_base::Type_fr_EpauleAVD_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tz_rf_haa_joint;    // Maxima DSL: -_k__tz_rf_haa_joint
    (*this)(2,5) =  ty_rf_haa_joint;    // Maxima DSL: _k__ty_rf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleAVD_X_fr_base& ForceTransforms::Type_fr_EpauleAVD_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_rf_haa_joint  = ScalarTraits::sin( q(RF_HAA_JOINT) );
    Scalar cos_q_rf_haa_joint  = ScalarTraits::cos( q(RF_HAA_JOINT) );
    (*this)(0,1) = sin_q_rf_haa_joint;
    (*this)(0,2) = cos_q_rf_haa_joint;
    (*this)(0,3) = ( ty_rf_haa_joint * cos_q_rf_haa_joint)-( tz_rf_haa_joint * sin_q_rf_haa_joint);
    (*this)(0,4) = - tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(0,5) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(1,1) = cos_q_rf_haa_joint;
    (*this)(1,2) = -sin_q_rf_haa_joint;
    (*this)(1,3) = (- ty_rf_haa_joint * sin_q_rf_haa_joint)-( tz_rf_haa_joint * cos_q_rf_haa_joint);
    (*this)(1,4) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(1,5) =  tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(3,4) = sin_q_rf_haa_joint;
    (*this)(3,5) = cos_q_rf_haa_joint;
    (*this)(4,4) = cos_q_rf_haa_joint;
    (*this)(4,5) = -sin_q_rf_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_EpauleAVD::Type_fr_base_X_fr_EpauleAVD()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = -1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = - tz_rf_haa_joint;    // Maxima DSL: -_k__tz_rf_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  ty_rf_haa_joint;    // Maxima DSL: _k__ty_rf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = -1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_EpauleAVD& ForceTransforms::Type_fr_base_X_fr_EpauleAVD::update(const state_t& q)
{
    Scalar sin_q_rf_haa_joint  = ScalarTraits::sin( q(RF_HAA_JOINT) );
    Scalar cos_q_rf_haa_joint  = ScalarTraits::cos( q(RF_HAA_JOINT) );
    (*this)(0,3) = ( ty_rf_haa_joint * cos_q_rf_haa_joint)-( tz_rf_haa_joint * sin_q_rf_haa_joint);
    (*this)(0,4) = (- ty_rf_haa_joint * sin_q_rf_haa_joint)-( tz_rf_haa_joint * cos_q_rf_haa_joint);
    (*this)(1,0) = sin_q_rf_haa_joint;
    (*this)(1,1) = cos_q_rf_haa_joint;
    (*this)(1,3) = - tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(1,4) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(2,0) = cos_q_rf_haa_joint;
    (*this)(2,1) = -sin_q_rf_haa_joint;
    (*this)(2,3) =  tx_rf_haa_joint * sin_q_rf_haa_joint;
    (*this)(2,4) =  tx_rf_haa_joint * cos_q_rf_haa_joint;
    (*this)(4,3) = sin_q_rf_haa_joint;
    (*this)(4,4) = cos_q_rf_haa_joint;
    (*this)(5,3) = cos_q_rf_haa_joint;
    (*this)(5,4) = -sin_q_rf_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD::Type_fr_HJambeAVD_X_fr_EpauleAVD()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD& ForceTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD::update(const state_t& q)
{
    Scalar sin_q_rf_hfe_joint  = ScalarTraits::sin( q(RF_HFE_JOINT) );
    Scalar cos_q_rf_hfe_joint  = ScalarTraits::cos( q(RF_HFE_JOINT) );
    (*this)(0,0) = sin_q_rf_hfe_joint;
    (*this)(0,2) = -cos_q_rf_hfe_joint;
    (*this)(0,3) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(0,5) = - ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(1,0) = cos_q_rf_hfe_joint;
    (*this)(1,2) = sin_q_rf_hfe_joint;
    (*this)(1,3) =  ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(1,5) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(3,3) = sin_q_rf_hfe_joint;
    (*this)(3,5) = -cos_q_rf_hfe_joint;
    (*this)(4,3) = cos_q_rf_hfe_joint;
    (*this)(4,5) = sin_q_rf_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD::Type_fr_EpauleAVD_X_fr_HJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD& ForceTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_hfe_joint  = ScalarTraits::sin( q(RF_HFE_JOINT) );
    Scalar cos_q_rf_hfe_joint  = ScalarTraits::cos( q(RF_HFE_JOINT) );
    (*this)(0,0) = sin_q_rf_hfe_joint;
    (*this)(0,1) = cos_q_rf_hfe_joint;
    (*this)(0,3) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(0,4) =  ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(2,0) = -cos_q_rf_hfe_joint;
    (*this)(2,1) = sin_q_rf_hfe_joint;
    (*this)(2,3) = - ty_rf_hfe_joint * sin_q_rf_hfe_joint;
    (*this)(2,4) = - ty_rf_hfe_joint * cos_q_rf_hfe_joint;
    (*this)(3,3) = sin_q_rf_hfe_joint;
    (*this)(3,4) = cos_q_rf_hfe_joint;
    (*this)(5,3) = -cos_q_rf_hfe_joint;
    (*this)(5,4) = sin_q_rf_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD::Type_fr_BJambeAVD_X_fr_HJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = - ty_rf_kfe_joint;    // Maxima DSL: -_k__ty_rf_kfe_joint
    (*this)(2,4) =  tx_rf_kfe_joint;    // Maxima DSL: _k__tx_rf_kfe_joint
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD& ForceTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_kfe_joint  = ScalarTraits::sin( q(RF_KFE_JOINT) );
    Scalar cos_q_rf_kfe_joint  = ScalarTraits::cos( q(RF_KFE_JOINT) );
    (*this)(0,0) = cos_q_rf_kfe_joint;
    (*this)(0,1) = -sin_q_rf_kfe_joint;
    (*this)(0,5) = (- tx_rf_kfe_joint * sin_q_rf_kfe_joint)-( ty_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(1,0) = -sin_q_rf_kfe_joint;
    (*this)(1,1) = -cos_q_rf_kfe_joint;
    (*this)(1,5) = ( ty_rf_kfe_joint * sin_q_rf_kfe_joint)-( tx_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(3,3) = cos_q_rf_kfe_joint;
    (*this)(3,4) = -sin_q_rf_kfe_joint;
    (*this)(4,3) = -sin_q_rf_kfe_joint;
    (*this)(4,4) = -cos_q_rf_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD::Type_fr_HJambeAVD_X_fr_BJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - ty_rf_kfe_joint;    // Maxima DSL: -_k__ty_rf_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_rf_kfe_joint;    // Maxima DSL: _k__tx_rf_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD& ForceTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_kfe_joint  = ScalarTraits::sin( q(RF_KFE_JOINT) );
    Scalar cos_q_rf_kfe_joint  = ScalarTraits::cos( q(RF_KFE_JOINT) );
    (*this)(0,0) = cos_q_rf_kfe_joint;
    (*this)(0,1) = -sin_q_rf_kfe_joint;
    (*this)(1,0) = -sin_q_rf_kfe_joint;
    (*this)(1,1) = -cos_q_rf_kfe_joint;
    (*this)(2,3) = (- tx_rf_kfe_joint * sin_q_rf_kfe_joint)-( ty_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(2,4) = ( ty_rf_kfe_joint * sin_q_rf_kfe_joint)-( tx_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(3,3) = cos_q_rf_kfe_joint;
    (*this)(3,4) = -sin_q_rf_kfe_joint;
    (*this)(4,3) = -sin_q_rf_kfe_joint;
    (*this)(4,4) = -cos_q_rf_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleAVG_X_fr_base::Type_fr_EpauleAVG_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tz_lf_haa_joint;    // Maxima DSL: _k__tz_lf_haa_joint
    (*this)(2,5) = - ty_lf_haa_joint;    // Maxima DSL: -_k__ty_lf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleAVG_X_fr_base& ForceTransforms::Type_fr_EpauleAVG_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_lf_haa_joint  = ScalarTraits::sin( q(LF_HAA_JOINT) );
    Scalar cos_q_lf_haa_joint  = ScalarTraits::cos( q(LF_HAA_JOINT) );
    (*this)(0,1) = sin_q_lf_haa_joint;
    (*this)(0,2) = -cos_q_lf_haa_joint;
    (*this)(0,3) = (- tz_lf_haa_joint * sin_q_lf_haa_joint)-( ty_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(0,4) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(0,5) =  tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(1,1) = cos_q_lf_haa_joint;
    (*this)(1,2) = sin_q_lf_haa_joint;
    (*this)(1,3) = ( ty_lf_haa_joint * sin_q_lf_haa_joint)-( tz_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(1,4) = - tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(1,5) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(3,4) = sin_q_lf_haa_joint;
    (*this)(3,5) = -cos_q_lf_haa_joint;
    (*this)(4,4) = cos_q_lf_haa_joint;
    (*this)(4,5) = sin_q_lf_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_EpauleAVG::Type_fr_base_X_fr_EpauleAVG()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) =  tz_lf_haa_joint;    // Maxima DSL: _k__tz_lf_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_lf_haa_joint;    // Maxima DSL: -_k__ty_lf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_EpauleAVG& ForceTransforms::Type_fr_base_X_fr_EpauleAVG::update(const state_t& q)
{
    Scalar sin_q_lf_haa_joint  = ScalarTraits::sin( q(LF_HAA_JOINT) );
    Scalar cos_q_lf_haa_joint  = ScalarTraits::cos( q(LF_HAA_JOINT) );
    (*this)(0,3) = (- tz_lf_haa_joint * sin_q_lf_haa_joint)-( ty_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(0,4) = ( ty_lf_haa_joint * sin_q_lf_haa_joint)-( tz_lf_haa_joint * cos_q_lf_haa_joint);
    (*this)(1,0) = sin_q_lf_haa_joint;
    (*this)(1,1) = cos_q_lf_haa_joint;
    (*this)(1,3) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(1,4) = - tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(2,0) = -cos_q_lf_haa_joint;
    (*this)(2,1) = sin_q_lf_haa_joint;
    (*this)(2,3) =  tx_lf_haa_joint * sin_q_lf_haa_joint;
    (*this)(2,4) =  tx_lf_haa_joint * cos_q_lf_haa_joint;
    (*this)(4,3) = sin_q_lf_haa_joint;
    (*this)(4,4) = cos_q_lf_haa_joint;
    (*this)(5,3) = -cos_q_lf_haa_joint;
    (*this)(5,4) = sin_q_lf_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG::Type_fr_HJambeAVG_X_fr_EpauleAVG()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG& ForceTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG::update(const state_t& q)
{
    Scalar sin_q_lf_hfe_joint  = ScalarTraits::sin( q(LF_HFE_JOINT) );
    Scalar cos_q_lf_hfe_joint  = ScalarTraits::cos( q(LF_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lf_hfe_joint;
    (*this)(0,2) = cos_q_lf_hfe_joint;
    (*this)(0,3) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(0,5) =  ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(1,0) = -cos_q_lf_hfe_joint;
    (*this)(1,2) = -sin_q_lf_hfe_joint;
    (*this)(1,3) = - ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(1,5) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(3,3) = -sin_q_lf_hfe_joint;
    (*this)(3,5) = cos_q_lf_hfe_joint;
    (*this)(4,3) = -cos_q_lf_hfe_joint;
    (*this)(4,5) = -sin_q_lf_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG::Type_fr_EpauleAVG_X_fr_HJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG& ForceTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_hfe_joint  = ScalarTraits::sin( q(LF_HFE_JOINT) );
    Scalar cos_q_lf_hfe_joint  = ScalarTraits::cos( q(LF_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lf_hfe_joint;
    (*this)(0,1) = -cos_q_lf_hfe_joint;
    (*this)(0,3) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(0,4) = - ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(2,0) = cos_q_lf_hfe_joint;
    (*this)(2,1) = -sin_q_lf_hfe_joint;
    (*this)(2,3) =  ty_lf_hfe_joint * sin_q_lf_hfe_joint;
    (*this)(2,4) =  ty_lf_hfe_joint * cos_q_lf_hfe_joint;
    (*this)(3,3) = -sin_q_lf_hfe_joint;
    (*this)(3,4) = -cos_q_lf_hfe_joint;
    (*this)(5,3) = cos_q_lf_hfe_joint;
    (*this)(5,4) = -sin_q_lf_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG::Type_fr_BJambeAVG_X_fr_HJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = - ty_lf_kfe_joint;    // Maxima DSL: -_k__ty_lf_kfe_joint
    (*this)(2,4) =  tx_lf_kfe_joint;    // Maxima DSL: _k__tx_lf_kfe_joint
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG& ForceTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_kfe_joint  = ScalarTraits::sin( q(LF_KFE_JOINT) );
    Scalar cos_q_lf_kfe_joint  = ScalarTraits::cos( q(LF_KFE_JOINT) );
    (*this)(0,0) = cos_q_lf_kfe_joint;
    (*this)(0,1) = -sin_q_lf_kfe_joint;
    (*this)(0,5) = (- tx_lf_kfe_joint * sin_q_lf_kfe_joint)-( ty_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(1,0) = -sin_q_lf_kfe_joint;
    (*this)(1,1) = -cos_q_lf_kfe_joint;
    (*this)(1,5) = ( ty_lf_kfe_joint * sin_q_lf_kfe_joint)-( tx_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(3,3) = cos_q_lf_kfe_joint;
    (*this)(3,4) = -sin_q_lf_kfe_joint;
    (*this)(4,3) = -sin_q_lf_kfe_joint;
    (*this)(4,4) = -cos_q_lf_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG::Type_fr_HJambeAVG_X_fr_BJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - ty_lf_kfe_joint;    // Maxima DSL: -_k__ty_lf_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_lf_kfe_joint;    // Maxima DSL: _k__tx_lf_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG& ForceTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_kfe_joint  = ScalarTraits::sin( q(LF_KFE_JOINT) );
    Scalar cos_q_lf_kfe_joint  = ScalarTraits::cos( q(LF_KFE_JOINT) );
    (*this)(0,0) = cos_q_lf_kfe_joint;
    (*this)(0,1) = -sin_q_lf_kfe_joint;
    (*this)(1,0) = -sin_q_lf_kfe_joint;
    (*this)(1,1) = -cos_q_lf_kfe_joint;
    (*this)(2,3) = (- tx_lf_kfe_joint * sin_q_lf_kfe_joint)-( ty_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(2,4) = ( ty_lf_kfe_joint * sin_q_lf_kfe_joint)-( tx_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(3,3) = cos_q_lf_kfe_joint;
    (*this)(3,4) = -sin_q_lf_kfe_joint;
    (*this)(4,3) = -sin_q_lf_kfe_joint;
    (*this)(4,4) = -cos_q_lf_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleARD_X_fr_base::Type_fr_EpauleARD_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tz_rh_haa_joint;    // Maxima DSL: -_k__tz_rh_haa_joint
    (*this)(2,5) =  ty_rh_haa_joint;    // Maxima DSL: _k__ty_rh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleARD_X_fr_base& ForceTransforms::Type_fr_EpauleARD_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_rh_haa_joint  = ScalarTraits::sin( q(RH_HAA_JOINT) );
    Scalar cos_q_rh_haa_joint  = ScalarTraits::cos( q(RH_HAA_JOINT) );
    (*this)(0,1) = sin_q_rh_haa_joint;
    (*this)(0,2) = cos_q_rh_haa_joint;
    (*this)(0,3) = ( ty_rh_haa_joint * cos_q_rh_haa_joint)-( tz_rh_haa_joint * sin_q_rh_haa_joint);
    (*this)(0,4) = - tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(0,5) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(1,1) = cos_q_rh_haa_joint;
    (*this)(1,2) = -sin_q_rh_haa_joint;
    (*this)(1,3) = (- ty_rh_haa_joint * sin_q_rh_haa_joint)-( tz_rh_haa_joint * cos_q_rh_haa_joint);
    (*this)(1,4) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(1,5) =  tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(3,4) = sin_q_rh_haa_joint;
    (*this)(3,5) = cos_q_rh_haa_joint;
    (*this)(4,4) = cos_q_rh_haa_joint;
    (*this)(4,5) = -sin_q_rh_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_EpauleARD::Type_fr_base_X_fr_EpauleARD()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = -1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = - tz_rh_haa_joint;    // Maxima DSL: -_k__tz_rh_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  ty_rh_haa_joint;    // Maxima DSL: _k__ty_rh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = -1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_EpauleARD& ForceTransforms::Type_fr_base_X_fr_EpauleARD::update(const state_t& q)
{
    Scalar sin_q_rh_haa_joint  = ScalarTraits::sin( q(RH_HAA_JOINT) );
    Scalar cos_q_rh_haa_joint  = ScalarTraits::cos( q(RH_HAA_JOINT) );
    (*this)(0,3) = ( ty_rh_haa_joint * cos_q_rh_haa_joint)-( tz_rh_haa_joint * sin_q_rh_haa_joint);
    (*this)(0,4) = (- ty_rh_haa_joint * sin_q_rh_haa_joint)-( tz_rh_haa_joint * cos_q_rh_haa_joint);
    (*this)(1,0) = sin_q_rh_haa_joint;
    (*this)(1,1) = cos_q_rh_haa_joint;
    (*this)(1,3) = - tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(1,4) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(2,0) = cos_q_rh_haa_joint;
    (*this)(2,1) = -sin_q_rh_haa_joint;
    (*this)(2,3) =  tx_rh_haa_joint * sin_q_rh_haa_joint;
    (*this)(2,4) =  tx_rh_haa_joint * cos_q_rh_haa_joint;
    (*this)(4,3) = sin_q_rh_haa_joint;
    (*this)(4,4) = cos_q_rh_haa_joint;
    (*this)(5,3) = cos_q_rh_haa_joint;
    (*this)(5,4) = -sin_q_rh_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeARD_X_fr_EpauleARD::Type_fr_HJambeARD_X_fr_EpauleARD()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_HJambeARD_X_fr_EpauleARD& ForceTransforms::Type_fr_HJambeARD_X_fr_EpauleARD::update(const state_t& q)
{
    Scalar sin_q_rh_hfe_joint  = ScalarTraits::sin( q(RH_HFE_JOINT) );
    Scalar cos_q_rh_hfe_joint  = ScalarTraits::cos( q(RH_HFE_JOINT) );
    (*this)(0,0) = sin_q_rh_hfe_joint;
    (*this)(0,2) = -cos_q_rh_hfe_joint;
    (*this)(0,3) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(0,5) = - ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(1,0) = cos_q_rh_hfe_joint;
    (*this)(1,2) = sin_q_rh_hfe_joint;
    (*this)(1,3) =  ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(1,5) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(3,3) = sin_q_rh_hfe_joint;
    (*this)(3,5) = -cos_q_rh_hfe_joint;
    (*this)(4,3) = cos_q_rh_hfe_joint;
    (*this)(4,5) = sin_q_rh_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleARD_X_fr_HJambeARD::Type_fr_EpauleARD_X_fr_HJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleARD_X_fr_HJambeARD& ForceTransforms::Type_fr_EpauleARD_X_fr_HJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_hfe_joint  = ScalarTraits::sin( q(RH_HFE_JOINT) );
    Scalar cos_q_rh_hfe_joint  = ScalarTraits::cos( q(RH_HFE_JOINT) );
    (*this)(0,0) = sin_q_rh_hfe_joint;
    (*this)(0,1) = cos_q_rh_hfe_joint;
    (*this)(0,3) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(0,4) =  ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(2,0) = -cos_q_rh_hfe_joint;
    (*this)(2,1) = sin_q_rh_hfe_joint;
    (*this)(2,3) = - ty_rh_hfe_joint * sin_q_rh_hfe_joint;
    (*this)(2,4) = - ty_rh_hfe_joint * cos_q_rh_hfe_joint;
    (*this)(3,3) = sin_q_rh_hfe_joint;
    (*this)(3,4) = cos_q_rh_hfe_joint;
    (*this)(5,3) = -cos_q_rh_hfe_joint;
    (*this)(5,4) = sin_q_rh_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_BJambeARD_X_fr_HJambeARD::Type_fr_BJambeARD_X_fr_HJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = - ty_rh_kfe_joint;    // Maxima DSL: -_k__ty_rh_kfe_joint
    (*this)(2,4) =  tx_rh_kfe_joint;    // Maxima DSL: _k__tx_rh_kfe_joint
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_BJambeARD_X_fr_HJambeARD& ForceTransforms::Type_fr_BJambeARD_X_fr_HJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_kfe_joint  = ScalarTraits::sin( q(RH_KFE_JOINT) );
    Scalar cos_q_rh_kfe_joint  = ScalarTraits::cos( q(RH_KFE_JOINT) );
    (*this)(0,0) = cos_q_rh_kfe_joint;
    (*this)(0,1) = -sin_q_rh_kfe_joint;
    (*this)(0,5) = (- tx_rh_kfe_joint * sin_q_rh_kfe_joint)-( ty_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(1,0) = -sin_q_rh_kfe_joint;
    (*this)(1,1) = -cos_q_rh_kfe_joint;
    (*this)(1,5) = ( ty_rh_kfe_joint * sin_q_rh_kfe_joint)-( tx_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(3,3) = cos_q_rh_kfe_joint;
    (*this)(3,4) = -sin_q_rh_kfe_joint;
    (*this)(4,3) = -sin_q_rh_kfe_joint;
    (*this)(4,4) = -cos_q_rh_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeARD_X_fr_BJambeARD::Type_fr_HJambeARD_X_fr_BJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - ty_rh_kfe_joint;    // Maxima DSL: -_k__ty_rh_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_rh_kfe_joint;    // Maxima DSL: _k__tx_rh_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_HJambeARD_X_fr_BJambeARD& ForceTransforms::Type_fr_HJambeARD_X_fr_BJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_kfe_joint  = ScalarTraits::sin( q(RH_KFE_JOINT) );
    Scalar cos_q_rh_kfe_joint  = ScalarTraits::cos( q(RH_KFE_JOINT) );
    (*this)(0,0) = cos_q_rh_kfe_joint;
    (*this)(0,1) = -sin_q_rh_kfe_joint;
    (*this)(1,0) = -sin_q_rh_kfe_joint;
    (*this)(1,1) = -cos_q_rh_kfe_joint;
    (*this)(2,3) = (- tx_rh_kfe_joint * sin_q_rh_kfe_joint)-( ty_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(2,4) = ( ty_rh_kfe_joint * sin_q_rh_kfe_joint)-( tx_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(3,3) = cos_q_rh_kfe_joint;
    (*this)(3,4) = -sin_q_rh_kfe_joint;
    (*this)(4,3) = -sin_q_rh_kfe_joint;
    (*this)(4,4) = -cos_q_rh_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleARG_X_fr_base::Type_fr_EpauleARG_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tz_lh_haa_joint;    // Maxima DSL: _k__tz_lh_haa_joint
    (*this)(2,5) = - ty_lh_haa_joint;    // Maxima DSL: -_k__ty_lh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleARG_X_fr_base& ForceTransforms::Type_fr_EpauleARG_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_lh_haa_joint  = ScalarTraits::sin( q(LH_HAA_JOINT) );
    Scalar cos_q_lh_haa_joint  = ScalarTraits::cos( q(LH_HAA_JOINT) );
    (*this)(0,1) = sin_q_lh_haa_joint;
    (*this)(0,2) = -cos_q_lh_haa_joint;
    (*this)(0,3) = (- tz_lh_haa_joint * sin_q_lh_haa_joint)-( ty_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(0,4) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(0,5) =  tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(1,1) = cos_q_lh_haa_joint;
    (*this)(1,2) = sin_q_lh_haa_joint;
    (*this)(1,3) = ( ty_lh_haa_joint * sin_q_lh_haa_joint)-( tz_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(1,4) = - tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(1,5) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(3,4) = sin_q_lh_haa_joint;
    (*this)(3,5) = -cos_q_lh_haa_joint;
    (*this)(4,4) = cos_q_lh_haa_joint;
    (*this)(4,5) = sin_q_lh_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_EpauleARG::Type_fr_base_X_fr_EpauleARG()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) =  tz_lh_haa_joint;    // Maxima DSL: _k__tz_lh_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_lh_haa_joint;    // Maxima DSL: -_k__ty_lh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_EpauleARG& ForceTransforms::Type_fr_base_X_fr_EpauleARG::update(const state_t& q)
{
    Scalar sin_q_lh_haa_joint  = ScalarTraits::sin( q(LH_HAA_JOINT) );
    Scalar cos_q_lh_haa_joint  = ScalarTraits::cos( q(LH_HAA_JOINT) );
    (*this)(0,3) = (- tz_lh_haa_joint * sin_q_lh_haa_joint)-( ty_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(0,4) = ( ty_lh_haa_joint * sin_q_lh_haa_joint)-( tz_lh_haa_joint * cos_q_lh_haa_joint);
    (*this)(1,0) = sin_q_lh_haa_joint;
    (*this)(1,1) = cos_q_lh_haa_joint;
    (*this)(1,3) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(1,4) = - tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(2,0) = -cos_q_lh_haa_joint;
    (*this)(2,1) = sin_q_lh_haa_joint;
    (*this)(2,3) =  tx_lh_haa_joint * sin_q_lh_haa_joint;
    (*this)(2,4) =  tx_lh_haa_joint * cos_q_lh_haa_joint;
    (*this)(4,3) = sin_q_lh_haa_joint;
    (*this)(4,4) = cos_q_lh_haa_joint;
    (*this)(5,3) = -cos_q_lh_haa_joint;
    (*this)(5,4) = sin_q_lh_haa_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeARG_X_fr_EpauleARG::Type_fr_HJambeARG_X_fr_EpauleARG()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_HJambeARG_X_fr_EpauleARG& ForceTransforms::Type_fr_HJambeARG_X_fr_EpauleARG::update(const state_t& q)
{
    Scalar sin_q_lh_hfe_joint  = ScalarTraits::sin( q(LH_HFE_JOINT) );
    Scalar cos_q_lh_hfe_joint  = ScalarTraits::cos( q(LH_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lh_hfe_joint;
    (*this)(0,2) = cos_q_lh_hfe_joint;
    (*this)(0,3) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(0,5) =  ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(1,0) = -cos_q_lh_hfe_joint;
    (*this)(1,2) = -sin_q_lh_hfe_joint;
    (*this)(1,3) = - ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(1,5) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(3,3) = -sin_q_lh_hfe_joint;
    (*this)(3,5) = cos_q_lh_hfe_joint;
    (*this)(4,3) = -cos_q_lh_hfe_joint;
    (*this)(4,5) = -sin_q_lh_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_EpauleARG_X_fr_HJambeARG::Type_fr_EpauleARG_X_fr_HJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_EpauleARG_X_fr_HJambeARG& ForceTransforms::Type_fr_EpauleARG_X_fr_HJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_hfe_joint  = ScalarTraits::sin( q(LH_HFE_JOINT) );
    Scalar cos_q_lh_hfe_joint  = ScalarTraits::cos( q(LH_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lh_hfe_joint;
    (*this)(0,1) = -cos_q_lh_hfe_joint;
    (*this)(0,3) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(0,4) = - ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(2,0) = cos_q_lh_hfe_joint;
    (*this)(2,1) = -sin_q_lh_hfe_joint;
    (*this)(2,3) =  ty_lh_hfe_joint * sin_q_lh_hfe_joint;
    (*this)(2,4) =  ty_lh_hfe_joint * cos_q_lh_hfe_joint;
    (*this)(3,3) = -sin_q_lh_hfe_joint;
    (*this)(3,4) = -cos_q_lh_hfe_joint;
    (*this)(5,3) = cos_q_lh_hfe_joint;
    (*this)(5,4) = -sin_q_lh_hfe_joint;
    return *this;
}
ForceTransforms::Type_fr_BJambeARG_X_fr_HJambeARG::Type_fr_BJambeARG_X_fr_HJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = - ty_lh_kfe_joint;    // Maxima DSL: -_k__ty_lh_kfe_joint
    (*this)(2,4) =  tx_lh_kfe_joint;    // Maxima DSL: _k__tx_lh_kfe_joint
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_BJambeARG_X_fr_HJambeARG& ForceTransforms::Type_fr_BJambeARG_X_fr_HJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_kfe_joint  = ScalarTraits::sin( q(LH_KFE_JOINT) );
    Scalar cos_q_lh_kfe_joint  = ScalarTraits::cos( q(LH_KFE_JOINT) );
    (*this)(0,0) = cos_q_lh_kfe_joint;
    (*this)(0,1) = -sin_q_lh_kfe_joint;
    (*this)(0,5) = (- tx_lh_kfe_joint * sin_q_lh_kfe_joint)-( ty_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(1,0) = -sin_q_lh_kfe_joint;
    (*this)(1,1) = -cos_q_lh_kfe_joint;
    (*this)(1,5) = ( ty_lh_kfe_joint * sin_q_lh_kfe_joint)-( tx_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(3,3) = cos_q_lh_kfe_joint;
    (*this)(3,4) = -sin_q_lh_kfe_joint;
    (*this)(4,3) = -sin_q_lh_kfe_joint;
    (*this)(4,4) = -cos_q_lh_kfe_joint;
    return *this;
}
ForceTransforms::Type_fr_HJambeARG_X_fr_BJambeARG::Type_fr_HJambeARG_X_fr_BJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - ty_lh_kfe_joint;    // Maxima DSL: -_k__ty_lh_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_lh_kfe_joint;    // Maxima DSL: _k__tx_lh_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_HJambeARG_X_fr_BJambeARG& ForceTransforms::Type_fr_HJambeARG_X_fr_BJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_kfe_joint  = ScalarTraits::sin( q(LH_KFE_JOINT) );
    Scalar cos_q_lh_kfe_joint  = ScalarTraits::cos( q(LH_KFE_JOINT) );
    (*this)(0,0) = cos_q_lh_kfe_joint;
    (*this)(0,1) = -sin_q_lh_kfe_joint;
    (*this)(1,0) = -sin_q_lh_kfe_joint;
    (*this)(1,1) = -cos_q_lh_kfe_joint;
    (*this)(2,3) = (- tx_lh_kfe_joint * sin_q_lh_kfe_joint)-( ty_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(2,4) = ( ty_lh_kfe_joint * sin_q_lh_kfe_joint)-( tx_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(3,3) = cos_q_lh_kfe_joint;
    (*this)(3,4) = -sin_q_lh_kfe_joint;
    (*this)(4,3) = -sin_q_lh_kfe_joint;
    (*this)(4,4) = -cos_q_lh_kfe_joint;
    return *this;
}

HomogeneousTransforms::Type_fr_EpauleAVD_X_fr_base::Type_fr_EpauleAVD_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tx_rf_haa_joint;    // Maxima DSL: _k__tx_rf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleAVD_X_fr_base& HomogeneousTransforms::Type_fr_EpauleAVD_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_rf_haa_joint  = ScalarTraits::sin( q(RF_HAA_JOINT) );
    Scalar cos_q_rf_haa_joint  = ScalarTraits::cos( q(RF_HAA_JOINT) );
    (*this)(0,1) = sin_q_rf_haa_joint;
    (*this)(0,2) = cos_q_rf_haa_joint;
    (*this)(0,3) = (- ty_rf_haa_joint * sin_q_rf_haa_joint)-( tz_rf_haa_joint * cos_q_rf_haa_joint);
    (*this)(1,1) = cos_q_rf_haa_joint;
    (*this)(1,2) = -sin_q_rf_haa_joint;
    (*this)(1,3) = ( tz_rf_haa_joint * sin_q_rf_haa_joint)-( ty_rf_haa_joint * cos_q_rf_haa_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_EpauleAVD::Type_fr_base_X_fr_EpauleAVD()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = -1.0;
    (*this)(0,3) =  tx_rf_haa_joint;    // Maxima DSL: _k__tx_rf_haa_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_rf_haa_joint;    // Maxima DSL: _k__ty_rf_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_rf_haa_joint;    // Maxima DSL: _k__tz_rf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_EpauleAVD& HomogeneousTransforms::Type_fr_base_X_fr_EpauleAVD::update(const state_t& q)
{
    Scalar sin_q_rf_haa_joint  = ScalarTraits::sin( q(RF_HAA_JOINT) );
    Scalar cos_q_rf_haa_joint  = ScalarTraits::cos( q(RF_HAA_JOINT) );
    (*this)(1,0) = sin_q_rf_haa_joint;
    (*this)(1,1) = cos_q_rf_haa_joint;
    (*this)(2,0) = cos_q_rf_haa_joint;
    (*this)(2,1) = -sin_q_rf_haa_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD::Type_fr_HJambeAVD_X_fr_EpauleAVD()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  ty_rf_hfe_joint;    // Maxima DSL: _k__ty_rf_hfe_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD& HomogeneousTransforms::Type_fr_HJambeAVD_X_fr_EpauleAVD::update(const state_t& q)
{
    Scalar sin_q_rf_hfe_joint  = ScalarTraits::sin( q(RF_HFE_JOINT) );
    Scalar cos_q_rf_hfe_joint  = ScalarTraits::cos( q(RF_HFE_JOINT) );
    (*this)(0,0) = sin_q_rf_hfe_joint;
    (*this)(0,2) = -cos_q_rf_hfe_joint;
    (*this)(1,0) = cos_q_rf_hfe_joint;
    (*this)(1,2) = sin_q_rf_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD::Type_fr_EpauleAVD_X_fr_HJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) =  ty_rf_hfe_joint;    // Maxima DSL: _k__ty_rf_hfe_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD& HomogeneousTransforms::Type_fr_EpauleAVD_X_fr_HJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_hfe_joint  = ScalarTraits::sin( q(RF_HFE_JOINT) );
    Scalar cos_q_rf_hfe_joint  = ScalarTraits::cos( q(RF_HFE_JOINT) );
    (*this)(0,0) = sin_q_rf_hfe_joint;
    (*this)(0,1) = cos_q_rf_hfe_joint;
    (*this)(2,0) = -cos_q_rf_hfe_joint;
    (*this)(2,1) = sin_q_rf_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD::Type_fr_BJambeAVD_X_fr_HJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD& HomogeneousTransforms::Type_fr_BJambeAVD_X_fr_HJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_kfe_joint  = ScalarTraits::sin( q(RF_KFE_JOINT) );
    Scalar cos_q_rf_kfe_joint  = ScalarTraits::cos( q(RF_KFE_JOINT) );
    (*this)(0,0) = cos_q_rf_kfe_joint;
    (*this)(0,1) = -sin_q_rf_kfe_joint;
    (*this)(0,3) = ( ty_rf_kfe_joint * sin_q_rf_kfe_joint)-( tx_rf_kfe_joint * cos_q_rf_kfe_joint);
    (*this)(1,0) = -sin_q_rf_kfe_joint;
    (*this)(1,1) = -cos_q_rf_kfe_joint;
    (*this)(1,3) = ( tx_rf_kfe_joint * sin_q_rf_kfe_joint)+( ty_rf_kfe_joint * cos_q_rf_kfe_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD::Type_fr_HJambeAVD_X_fr_BJambeAVD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_rf_kfe_joint;    // Maxima DSL: _k__tx_rf_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_rf_kfe_joint;    // Maxima DSL: _k__ty_rf_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD& HomogeneousTransforms::Type_fr_HJambeAVD_X_fr_BJambeAVD::update(const state_t& q)
{
    Scalar sin_q_rf_kfe_joint  = ScalarTraits::sin( q(RF_KFE_JOINT) );
    Scalar cos_q_rf_kfe_joint  = ScalarTraits::cos( q(RF_KFE_JOINT) );
    (*this)(0,0) = cos_q_rf_kfe_joint;
    (*this)(0,1) = -sin_q_rf_kfe_joint;
    (*this)(1,0) = -sin_q_rf_kfe_joint;
    (*this)(1,1) = -cos_q_rf_kfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleAVG_X_fr_base::Type_fr_EpauleAVG_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_lf_haa_joint;    // Maxima DSL: -_k__tx_lf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleAVG_X_fr_base& HomogeneousTransforms::Type_fr_EpauleAVG_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_lf_haa_joint  = ScalarTraits::sin( q(LF_HAA_JOINT) );
    Scalar cos_q_lf_haa_joint  = ScalarTraits::cos( q(LF_HAA_JOINT) );
    (*this)(0,1) = sin_q_lf_haa_joint;
    (*this)(0,2) = -cos_q_lf_haa_joint;
    (*this)(0,3) = ( tz_lf_haa_joint * cos_q_lf_haa_joint)-( ty_lf_haa_joint * sin_q_lf_haa_joint);
    (*this)(1,1) = cos_q_lf_haa_joint;
    (*this)(1,2) = sin_q_lf_haa_joint;
    (*this)(1,3) = (- tz_lf_haa_joint * sin_q_lf_haa_joint)-( ty_lf_haa_joint * cos_q_lf_haa_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_EpauleAVG::Type_fr_base_X_fr_EpauleAVG()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_lf_haa_joint;    // Maxima DSL: _k__tx_lf_haa_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_lf_haa_joint;    // Maxima DSL: _k__ty_lf_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_lf_haa_joint;    // Maxima DSL: _k__tz_lf_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_EpauleAVG& HomogeneousTransforms::Type_fr_base_X_fr_EpauleAVG::update(const state_t& q)
{
    Scalar sin_q_lf_haa_joint  = ScalarTraits::sin( q(LF_HAA_JOINT) );
    Scalar cos_q_lf_haa_joint  = ScalarTraits::cos( q(LF_HAA_JOINT) );
    (*this)(1,0) = sin_q_lf_haa_joint;
    (*this)(1,1) = cos_q_lf_haa_joint;
    (*this)(2,0) = -cos_q_lf_haa_joint;
    (*this)(2,1) = sin_q_lf_haa_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG::Type_fr_HJambeAVG_X_fr_EpauleAVG()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  ty_lf_hfe_joint;    // Maxima DSL: _k__ty_lf_hfe_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG& HomogeneousTransforms::Type_fr_HJambeAVG_X_fr_EpauleAVG::update(const state_t& q)
{
    Scalar sin_q_lf_hfe_joint  = ScalarTraits::sin( q(LF_HFE_JOINT) );
    Scalar cos_q_lf_hfe_joint  = ScalarTraits::cos( q(LF_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lf_hfe_joint;
    (*this)(0,2) = cos_q_lf_hfe_joint;
    (*this)(1,0) = -cos_q_lf_hfe_joint;
    (*this)(1,2) = -sin_q_lf_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG::Type_fr_EpauleAVG_X_fr_HJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) =  ty_lf_hfe_joint;    // Maxima DSL: _k__ty_lf_hfe_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG& HomogeneousTransforms::Type_fr_EpauleAVG_X_fr_HJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_hfe_joint  = ScalarTraits::sin( q(LF_HFE_JOINT) );
    Scalar cos_q_lf_hfe_joint  = ScalarTraits::cos( q(LF_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lf_hfe_joint;
    (*this)(0,1) = -cos_q_lf_hfe_joint;
    (*this)(2,0) = cos_q_lf_hfe_joint;
    (*this)(2,1) = -sin_q_lf_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG::Type_fr_BJambeAVG_X_fr_HJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG& HomogeneousTransforms::Type_fr_BJambeAVG_X_fr_HJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_kfe_joint  = ScalarTraits::sin( q(LF_KFE_JOINT) );
    Scalar cos_q_lf_kfe_joint  = ScalarTraits::cos( q(LF_KFE_JOINT) );
    (*this)(0,0) = cos_q_lf_kfe_joint;
    (*this)(0,1) = -sin_q_lf_kfe_joint;
    (*this)(0,3) = ( ty_lf_kfe_joint * sin_q_lf_kfe_joint)-( tx_lf_kfe_joint * cos_q_lf_kfe_joint);
    (*this)(1,0) = -sin_q_lf_kfe_joint;
    (*this)(1,1) = -cos_q_lf_kfe_joint;
    (*this)(1,3) = ( tx_lf_kfe_joint * sin_q_lf_kfe_joint)+( ty_lf_kfe_joint * cos_q_lf_kfe_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG::Type_fr_HJambeAVG_X_fr_BJambeAVG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_lf_kfe_joint;    // Maxima DSL: _k__tx_lf_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_lf_kfe_joint;    // Maxima DSL: _k__ty_lf_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG& HomogeneousTransforms::Type_fr_HJambeAVG_X_fr_BJambeAVG::update(const state_t& q)
{
    Scalar sin_q_lf_kfe_joint  = ScalarTraits::sin( q(LF_KFE_JOINT) );
    Scalar cos_q_lf_kfe_joint  = ScalarTraits::cos( q(LF_KFE_JOINT) );
    (*this)(0,0) = cos_q_lf_kfe_joint;
    (*this)(0,1) = -sin_q_lf_kfe_joint;
    (*this)(1,0) = -sin_q_lf_kfe_joint;
    (*this)(1,1) = -cos_q_lf_kfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleARD_X_fr_base::Type_fr_EpauleARD_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tx_rh_haa_joint;    // Maxima DSL: _k__tx_rh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleARD_X_fr_base& HomogeneousTransforms::Type_fr_EpauleARD_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_rh_haa_joint  = ScalarTraits::sin( q(RH_HAA_JOINT) );
    Scalar cos_q_rh_haa_joint  = ScalarTraits::cos( q(RH_HAA_JOINT) );
    (*this)(0,1) = sin_q_rh_haa_joint;
    (*this)(0,2) = cos_q_rh_haa_joint;
    (*this)(0,3) = (- ty_rh_haa_joint * sin_q_rh_haa_joint)-( tz_rh_haa_joint * cos_q_rh_haa_joint);
    (*this)(1,1) = cos_q_rh_haa_joint;
    (*this)(1,2) = -sin_q_rh_haa_joint;
    (*this)(1,3) = ( tz_rh_haa_joint * sin_q_rh_haa_joint)-( ty_rh_haa_joint * cos_q_rh_haa_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_EpauleARD::Type_fr_base_X_fr_EpauleARD()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = -1.0;
    (*this)(0,3) =  tx_rh_haa_joint;    // Maxima DSL: _k__tx_rh_haa_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_rh_haa_joint;    // Maxima DSL: _k__ty_rh_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_rh_haa_joint;    // Maxima DSL: _k__tz_rh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_EpauleARD& HomogeneousTransforms::Type_fr_base_X_fr_EpauleARD::update(const state_t& q)
{
    Scalar sin_q_rh_haa_joint  = ScalarTraits::sin( q(RH_HAA_JOINT) );
    Scalar cos_q_rh_haa_joint  = ScalarTraits::cos( q(RH_HAA_JOINT) );
    (*this)(1,0) = sin_q_rh_haa_joint;
    (*this)(1,1) = cos_q_rh_haa_joint;
    (*this)(2,0) = cos_q_rh_haa_joint;
    (*this)(2,1) = -sin_q_rh_haa_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeARD_X_fr_EpauleARD::Type_fr_HJambeARD_X_fr_EpauleARD()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  ty_rh_hfe_joint;    // Maxima DSL: _k__ty_rh_hfe_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeARD_X_fr_EpauleARD& HomogeneousTransforms::Type_fr_HJambeARD_X_fr_EpauleARD::update(const state_t& q)
{
    Scalar sin_q_rh_hfe_joint  = ScalarTraits::sin( q(RH_HFE_JOINT) );
    Scalar cos_q_rh_hfe_joint  = ScalarTraits::cos( q(RH_HFE_JOINT) );
    (*this)(0,0) = sin_q_rh_hfe_joint;
    (*this)(0,2) = -cos_q_rh_hfe_joint;
    (*this)(1,0) = cos_q_rh_hfe_joint;
    (*this)(1,2) = sin_q_rh_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleARD_X_fr_HJambeARD::Type_fr_EpauleARD_X_fr_HJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) =  ty_rh_hfe_joint;    // Maxima DSL: _k__ty_rh_hfe_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleARD_X_fr_HJambeARD& HomogeneousTransforms::Type_fr_EpauleARD_X_fr_HJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_hfe_joint  = ScalarTraits::sin( q(RH_HFE_JOINT) );
    Scalar cos_q_rh_hfe_joint  = ScalarTraits::cos( q(RH_HFE_JOINT) );
    (*this)(0,0) = sin_q_rh_hfe_joint;
    (*this)(0,1) = cos_q_rh_hfe_joint;
    (*this)(2,0) = -cos_q_rh_hfe_joint;
    (*this)(2,1) = sin_q_rh_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_BJambeARD_X_fr_HJambeARD::Type_fr_BJambeARD_X_fr_HJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_BJambeARD_X_fr_HJambeARD& HomogeneousTransforms::Type_fr_BJambeARD_X_fr_HJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_kfe_joint  = ScalarTraits::sin( q(RH_KFE_JOINT) );
    Scalar cos_q_rh_kfe_joint  = ScalarTraits::cos( q(RH_KFE_JOINT) );
    (*this)(0,0) = cos_q_rh_kfe_joint;
    (*this)(0,1) = -sin_q_rh_kfe_joint;
    (*this)(0,3) = ( ty_rh_kfe_joint * sin_q_rh_kfe_joint)-( tx_rh_kfe_joint * cos_q_rh_kfe_joint);
    (*this)(1,0) = -sin_q_rh_kfe_joint;
    (*this)(1,1) = -cos_q_rh_kfe_joint;
    (*this)(1,3) = ( tx_rh_kfe_joint * sin_q_rh_kfe_joint)+( ty_rh_kfe_joint * cos_q_rh_kfe_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeARD_X_fr_BJambeARD::Type_fr_HJambeARD_X_fr_BJambeARD()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_rh_kfe_joint;    // Maxima DSL: _k__tx_rh_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_rh_kfe_joint;    // Maxima DSL: _k__ty_rh_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeARD_X_fr_BJambeARD& HomogeneousTransforms::Type_fr_HJambeARD_X_fr_BJambeARD::update(const state_t& q)
{
    Scalar sin_q_rh_kfe_joint  = ScalarTraits::sin( q(RH_KFE_JOINT) );
    Scalar cos_q_rh_kfe_joint  = ScalarTraits::cos( q(RH_KFE_JOINT) );
    (*this)(0,0) = cos_q_rh_kfe_joint;
    (*this)(0,1) = -sin_q_rh_kfe_joint;
    (*this)(1,0) = -sin_q_rh_kfe_joint;
    (*this)(1,1) = -cos_q_rh_kfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleARG_X_fr_base::Type_fr_EpauleARG_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_lh_haa_joint;    // Maxima DSL: -_k__tx_lh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleARG_X_fr_base& HomogeneousTransforms::Type_fr_EpauleARG_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_lh_haa_joint  = ScalarTraits::sin( q(LH_HAA_JOINT) );
    Scalar cos_q_lh_haa_joint  = ScalarTraits::cos( q(LH_HAA_JOINT) );
    (*this)(0,1) = sin_q_lh_haa_joint;
    (*this)(0,2) = -cos_q_lh_haa_joint;
    (*this)(0,3) = ( tz_lh_haa_joint * cos_q_lh_haa_joint)-( ty_lh_haa_joint * sin_q_lh_haa_joint);
    (*this)(1,1) = cos_q_lh_haa_joint;
    (*this)(1,2) = sin_q_lh_haa_joint;
    (*this)(1,3) = (- tz_lh_haa_joint * sin_q_lh_haa_joint)-( ty_lh_haa_joint * cos_q_lh_haa_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_EpauleARG::Type_fr_base_X_fr_EpauleARG()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_lh_haa_joint;    // Maxima DSL: _k__tx_lh_haa_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_lh_haa_joint;    // Maxima DSL: _k__ty_lh_haa_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_lh_haa_joint;    // Maxima DSL: _k__tz_lh_haa_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_EpauleARG& HomogeneousTransforms::Type_fr_base_X_fr_EpauleARG::update(const state_t& q)
{
    Scalar sin_q_lh_haa_joint  = ScalarTraits::sin( q(LH_HAA_JOINT) );
    Scalar cos_q_lh_haa_joint  = ScalarTraits::cos( q(LH_HAA_JOINT) );
    (*this)(1,0) = sin_q_lh_haa_joint;
    (*this)(1,1) = cos_q_lh_haa_joint;
    (*this)(2,0) = -cos_q_lh_haa_joint;
    (*this)(2,1) = sin_q_lh_haa_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeARG_X_fr_EpauleARG::Type_fr_HJambeARG_X_fr_EpauleARG()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  ty_lh_hfe_joint;    // Maxima DSL: _k__ty_lh_hfe_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeARG_X_fr_EpauleARG& HomogeneousTransforms::Type_fr_HJambeARG_X_fr_EpauleARG::update(const state_t& q)
{
    Scalar sin_q_lh_hfe_joint  = ScalarTraits::sin( q(LH_HFE_JOINT) );
    Scalar cos_q_lh_hfe_joint  = ScalarTraits::cos( q(LH_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lh_hfe_joint;
    (*this)(0,2) = cos_q_lh_hfe_joint;
    (*this)(1,0) = -cos_q_lh_hfe_joint;
    (*this)(1,2) = -sin_q_lh_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_EpauleARG_X_fr_HJambeARG::Type_fr_EpauleARG_X_fr_HJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) =  ty_lh_hfe_joint;    // Maxima DSL: _k__ty_lh_hfe_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_EpauleARG_X_fr_HJambeARG& HomogeneousTransforms::Type_fr_EpauleARG_X_fr_HJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_hfe_joint  = ScalarTraits::sin( q(LH_HFE_JOINT) );
    Scalar cos_q_lh_hfe_joint  = ScalarTraits::cos( q(LH_HFE_JOINT) );
    (*this)(0,0) = -sin_q_lh_hfe_joint;
    (*this)(0,1) = -cos_q_lh_hfe_joint;
    (*this)(2,0) = cos_q_lh_hfe_joint;
    (*this)(2,1) = -sin_q_lh_hfe_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_BJambeARG_X_fr_HJambeARG::Type_fr_BJambeARG_X_fr_HJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_BJambeARG_X_fr_HJambeARG& HomogeneousTransforms::Type_fr_BJambeARG_X_fr_HJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_kfe_joint  = ScalarTraits::sin( q(LH_KFE_JOINT) );
    Scalar cos_q_lh_kfe_joint  = ScalarTraits::cos( q(LH_KFE_JOINT) );
    (*this)(0,0) = cos_q_lh_kfe_joint;
    (*this)(0,1) = -sin_q_lh_kfe_joint;
    (*this)(0,3) = ( ty_lh_kfe_joint * sin_q_lh_kfe_joint)-( tx_lh_kfe_joint * cos_q_lh_kfe_joint);
    (*this)(1,0) = -sin_q_lh_kfe_joint;
    (*this)(1,1) = -cos_q_lh_kfe_joint;
    (*this)(1,3) = ( tx_lh_kfe_joint * sin_q_lh_kfe_joint)+( ty_lh_kfe_joint * cos_q_lh_kfe_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_HJambeARG_X_fr_BJambeARG::Type_fr_HJambeARG_X_fr_BJambeARG()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_lh_kfe_joint;    // Maxima DSL: _k__tx_lh_kfe_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_lh_kfe_joint;    // Maxima DSL: _k__ty_lh_kfe_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_HJambeARG_X_fr_BJambeARG& HomogeneousTransforms::Type_fr_HJambeARG_X_fr_BJambeARG::update(const state_t& q)
{
    Scalar sin_q_lh_kfe_joint  = ScalarTraits::sin( q(LH_KFE_JOINT) );
    Scalar cos_q_lh_kfe_joint  = ScalarTraits::cos( q(LH_KFE_JOINT) );
    (*this)(0,0) = cos_q_lh_kfe_joint;
    (*this)(0,1) = -sin_q_lh_kfe_joint;
    (*this)(1,0) = -sin_q_lh_kfe_joint;
    (*this)(1,1) = -cos_q_lh_kfe_joint;
    return *this;
}

