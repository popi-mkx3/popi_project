#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
iit::popi::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    BJambeAVD_Ic(linkInertias.getTensor_BJambeAVD()),
    BJambeAVG_Ic(linkInertias.getTensor_BJambeAVG()),
    BJambeARD_Ic(linkInertias.getTensor_BJambeARD()),
    BJambeARG_Ic(linkInertias.getTensor_BJambeARG())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const iit::popi::dyn::JSIM& iit::popi::dyn::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_HJambeARG_X_fr_BJambeARG(state);
    frcTransf -> fr_EpauleARG_X_fr_HJambeARG(state);
    frcTransf -> fr_base_X_fr_EpauleARG(state);
    frcTransf -> fr_HJambeARD_X_fr_BJambeARD(state);
    frcTransf -> fr_EpauleARD_X_fr_HJambeARD(state);
    frcTransf -> fr_base_X_fr_EpauleARD(state);
    frcTransf -> fr_HJambeAVG_X_fr_BJambeAVG(state);
    frcTransf -> fr_EpauleAVG_X_fr_HJambeAVG(state);
    frcTransf -> fr_base_X_fr_EpauleAVG(state);
    frcTransf -> fr_HJambeAVD_X_fr_BJambeAVD(state);
    frcTransf -> fr_EpauleAVD_X_fr_HJambeAVD(state);
    frcTransf -> fr_base_X_fr_EpauleAVD(state);

    // Initializes the composite inertia tensors
    base_Ic = linkInertias.getTensor_base();
    EpauleAVD_Ic = linkInertias.getTensor_EpauleAVD();
    HJambeAVD_Ic = linkInertias.getTensor_HJambeAVD();
    EpauleAVG_Ic = linkInertias.getTensor_EpauleAVG();
    HJambeAVG_Ic = linkInertias.getTensor_HJambeAVG();
    EpauleARD_Ic = linkInertias.getTensor_EpauleARD();
    HJambeARD_Ic = linkInertias.getTensor_HJambeARD();
    EpauleARG_Ic = linkInertias.getTensor_EpauleARG();
    HJambeARG_Ic = linkInertias.getTensor_HJambeARG();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link BJambeARG:
    iit::rbd::transformInertia<Scalar>(BJambeARG_Ic, frcTransf -> fr_HJambeARG_X_fr_BJambeARG, Ic_spare);
    HJambeARG_Ic += Ic_spare;

    Fcol(LH_KFE_JOINT) = BJambeARG_Ic.col(AZ);
    DATA(LH_KFE_JOINT+6, LH_KFE_JOINT+6) = Fcol(LH_KFE_JOINT)(AZ);

    Fcol(LH_KFE_JOINT) = frcTransf -> fr_HJambeARG_X_fr_BJambeARG * Fcol(LH_KFE_JOINT);
    DATA(LH_KFE_JOINT+6, LH_HFE_JOINT+6) = F(AZ,LH_KFE_JOINT);
    DATA(LH_HFE_JOINT+6, LH_KFE_JOINT+6) = DATA(LH_KFE_JOINT+6, LH_HFE_JOINT+6);
    Fcol(LH_KFE_JOINT) = frcTransf -> fr_EpauleARG_X_fr_HJambeARG * Fcol(LH_KFE_JOINT);
    DATA(LH_KFE_JOINT+6, LH_HAA_JOINT+6) = F(AZ,LH_KFE_JOINT);
    DATA(LH_HAA_JOINT+6, LH_KFE_JOINT+6) = DATA(LH_KFE_JOINT+6, LH_HAA_JOINT+6);
    Fcol(LH_KFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleARG * Fcol(LH_KFE_JOINT);

    // Link HJambeARG:
    iit::rbd::transformInertia<Scalar>(HJambeARG_Ic, frcTransf -> fr_EpauleARG_X_fr_HJambeARG, Ic_spare);
    EpauleARG_Ic += Ic_spare;

    Fcol(LH_HFE_JOINT) = HJambeARG_Ic.col(AZ);
    DATA(LH_HFE_JOINT+6, LH_HFE_JOINT+6) = Fcol(LH_HFE_JOINT)(AZ);

    Fcol(LH_HFE_JOINT) = frcTransf -> fr_EpauleARG_X_fr_HJambeARG * Fcol(LH_HFE_JOINT);
    DATA(LH_HFE_JOINT+6, LH_HAA_JOINT+6) = F(AZ,LH_HFE_JOINT);
    DATA(LH_HAA_JOINT+6, LH_HFE_JOINT+6) = DATA(LH_HFE_JOINT+6, LH_HAA_JOINT+6);
    Fcol(LH_HFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleARG * Fcol(LH_HFE_JOINT);

    // Link EpauleARG:
    iit::rbd::transformInertia<Scalar>(EpauleARG_Ic, frcTransf -> fr_base_X_fr_EpauleARG, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(LH_HAA_JOINT) = EpauleARG_Ic.col(AZ);
    DATA(LH_HAA_JOINT+6, LH_HAA_JOINT+6) = Fcol(LH_HAA_JOINT)(AZ);

    Fcol(LH_HAA_JOINT) = frcTransf -> fr_base_X_fr_EpauleARG * Fcol(LH_HAA_JOINT);

    // Link BJambeARD:
    iit::rbd::transformInertia<Scalar>(BJambeARD_Ic, frcTransf -> fr_HJambeARD_X_fr_BJambeARD, Ic_spare);
    HJambeARD_Ic += Ic_spare;

    Fcol(RH_KFE_JOINT) = BJambeARD_Ic.col(AZ);
    DATA(RH_KFE_JOINT+6, RH_KFE_JOINT+6) = Fcol(RH_KFE_JOINT)(AZ);

    Fcol(RH_KFE_JOINT) = frcTransf -> fr_HJambeARD_X_fr_BJambeARD * Fcol(RH_KFE_JOINT);
    DATA(RH_KFE_JOINT+6, RH_HFE_JOINT+6) = F(AZ,RH_KFE_JOINT);
    DATA(RH_HFE_JOINT+6, RH_KFE_JOINT+6) = DATA(RH_KFE_JOINT+6, RH_HFE_JOINT+6);
    Fcol(RH_KFE_JOINT) = frcTransf -> fr_EpauleARD_X_fr_HJambeARD * Fcol(RH_KFE_JOINT);
    DATA(RH_KFE_JOINT+6, RH_HAA_JOINT+6) = F(AZ,RH_KFE_JOINT);
    DATA(RH_HAA_JOINT+6, RH_KFE_JOINT+6) = DATA(RH_KFE_JOINT+6, RH_HAA_JOINT+6);
    Fcol(RH_KFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleARD * Fcol(RH_KFE_JOINT);

    // Link HJambeARD:
    iit::rbd::transformInertia<Scalar>(HJambeARD_Ic, frcTransf -> fr_EpauleARD_X_fr_HJambeARD, Ic_spare);
    EpauleARD_Ic += Ic_spare;

    Fcol(RH_HFE_JOINT) = HJambeARD_Ic.col(AZ);
    DATA(RH_HFE_JOINT+6, RH_HFE_JOINT+6) = Fcol(RH_HFE_JOINT)(AZ);

    Fcol(RH_HFE_JOINT) = frcTransf -> fr_EpauleARD_X_fr_HJambeARD * Fcol(RH_HFE_JOINT);
    DATA(RH_HFE_JOINT+6, RH_HAA_JOINT+6) = F(AZ,RH_HFE_JOINT);
    DATA(RH_HAA_JOINT+6, RH_HFE_JOINT+6) = DATA(RH_HFE_JOINT+6, RH_HAA_JOINT+6);
    Fcol(RH_HFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleARD * Fcol(RH_HFE_JOINT);

    // Link EpauleARD:
    iit::rbd::transformInertia<Scalar>(EpauleARD_Ic, frcTransf -> fr_base_X_fr_EpauleARD, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(RH_HAA_JOINT) = EpauleARD_Ic.col(AZ);
    DATA(RH_HAA_JOINT+6, RH_HAA_JOINT+6) = Fcol(RH_HAA_JOINT)(AZ);

    Fcol(RH_HAA_JOINT) = frcTransf -> fr_base_X_fr_EpauleARD * Fcol(RH_HAA_JOINT);

    // Link BJambeAVG:
    iit::rbd::transformInertia<Scalar>(BJambeAVG_Ic, frcTransf -> fr_HJambeAVG_X_fr_BJambeAVG, Ic_spare);
    HJambeAVG_Ic += Ic_spare;

    Fcol(LF_KFE_JOINT) = BJambeAVG_Ic.col(AZ);
    DATA(LF_KFE_JOINT+6, LF_KFE_JOINT+6) = Fcol(LF_KFE_JOINT)(AZ);

    Fcol(LF_KFE_JOINT) = frcTransf -> fr_HJambeAVG_X_fr_BJambeAVG * Fcol(LF_KFE_JOINT);
    DATA(LF_KFE_JOINT+6, LF_HFE_JOINT+6) = F(AZ,LF_KFE_JOINT);
    DATA(LF_HFE_JOINT+6, LF_KFE_JOINT+6) = DATA(LF_KFE_JOINT+6, LF_HFE_JOINT+6);
    Fcol(LF_KFE_JOINT) = frcTransf -> fr_EpauleAVG_X_fr_HJambeAVG * Fcol(LF_KFE_JOINT);
    DATA(LF_KFE_JOINT+6, LF_HAA_JOINT+6) = F(AZ,LF_KFE_JOINT);
    DATA(LF_HAA_JOINT+6, LF_KFE_JOINT+6) = DATA(LF_KFE_JOINT+6, LF_HAA_JOINT+6);
    Fcol(LF_KFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleAVG * Fcol(LF_KFE_JOINT);

    // Link HJambeAVG:
    iit::rbd::transformInertia<Scalar>(HJambeAVG_Ic, frcTransf -> fr_EpauleAVG_X_fr_HJambeAVG, Ic_spare);
    EpauleAVG_Ic += Ic_spare;

    Fcol(LF_HFE_JOINT) = HJambeAVG_Ic.col(AZ);
    DATA(LF_HFE_JOINT+6, LF_HFE_JOINT+6) = Fcol(LF_HFE_JOINT)(AZ);

    Fcol(LF_HFE_JOINT) = frcTransf -> fr_EpauleAVG_X_fr_HJambeAVG * Fcol(LF_HFE_JOINT);
    DATA(LF_HFE_JOINT+6, LF_HAA_JOINT+6) = F(AZ,LF_HFE_JOINT);
    DATA(LF_HAA_JOINT+6, LF_HFE_JOINT+6) = DATA(LF_HFE_JOINT+6, LF_HAA_JOINT+6);
    Fcol(LF_HFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleAVG * Fcol(LF_HFE_JOINT);

    // Link EpauleAVG:
    iit::rbd::transformInertia<Scalar>(EpauleAVG_Ic, frcTransf -> fr_base_X_fr_EpauleAVG, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(LF_HAA_JOINT) = EpauleAVG_Ic.col(AZ);
    DATA(LF_HAA_JOINT+6, LF_HAA_JOINT+6) = Fcol(LF_HAA_JOINT)(AZ);

    Fcol(LF_HAA_JOINT) = frcTransf -> fr_base_X_fr_EpauleAVG * Fcol(LF_HAA_JOINT);

    // Link BJambeAVD:
    iit::rbd::transformInertia<Scalar>(BJambeAVD_Ic, frcTransf -> fr_HJambeAVD_X_fr_BJambeAVD, Ic_spare);
    HJambeAVD_Ic += Ic_spare;

    Fcol(RF_KFE_JOINT) = BJambeAVD_Ic.col(AZ);
    DATA(RF_KFE_JOINT+6, RF_KFE_JOINT+6) = Fcol(RF_KFE_JOINT)(AZ);

    Fcol(RF_KFE_JOINT) = frcTransf -> fr_HJambeAVD_X_fr_BJambeAVD * Fcol(RF_KFE_JOINT);
    DATA(RF_KFE_JOINT+6, RF_HFE_JOINT+6) = F(AZ,RF_KFE_JOINT);
    DATA(RF_HFE_JOINT+6, RF_KFE_JOINT+6) = DATA(RF_KFE_JOINT+6, RF_HFE_JOINT+6);
    Fcol(RF_KFE_JOINT) = frcTransf -> fr_EpauleAVD_X_fr_HJambeAVD * Fcol(RF_KFE_JOINT);
    DATA(RF_KFE_JOINT+6, RF_HAA_JOINT+6) = F(AZ,RF_KFE_JOINT);
    DATA(RF_HAA_JOINT+6, RF_KFE_JOINT+6) = DATA(RF_KFE_JOINT+6, RF_HAA_JOINT+6);
    Fcol(RF_KFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleAVD * Fcol(RF_KFE_JOINT);

    // Link HJambeAVD:
    iit::rbd::transformInertia<Scalar>(HJambeAVD_Ic, frcTransf -> fr_EpauleAVD_X_fr_HJambeAVD, Ic_spare);
    EpauleAVD_Ic += Ic_spare;

    Fcol(RF_HFE_JOINT) = HJambeAVD_Ic.col(AZ);
    DATA(RF_HFE_JOINT+6, RF_HFE_JOINT+6) = Fcol(RF_HFE_JOINT)(AZ);

    Fcol(RF_HFE_JOINT) = frcTransf -> fr_EpauleAVD_X_fr_HJambeAVD * Fcol(RF_HFE_JOINT);
    DATA(RF_HFE_JOINT+6, RF_HAA_JOINT+6) = F(AZ,RF_HFE_JOINT);
    DATA(RF_HAA_JOINT+6, RF_HFE_JOINT+6) = DATA(RF_HFE_JOINT+6, RF_HAA_JOINT+6);
    Fcol(RF_HFE_JOINT) = frcTransf -> fr_base_X_fr_EpauleAVD * Fcol(RF_HFE_JOINT);

    // Link EpauleAVD:
    iit::rbd::transformInertia<Scalar>(EpauleAVD_Ic, frcTransf -> fr_base_X_fr_EpauleAVD, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(RF_HAA_JOINT) = EpauleAVD_Ic.col(AZ);
    DATA(RF_HAA_JOINT+6, RF_HAA_JOINT+6) = Fcol(RF_HAA_JOINT)(AZ);

    Fcol(RF_HAA_JOINT) = frcTransf -> fr_base_X_fr_EpauleAVD * Fcol(RF_HAA_JOINT);

    // Copies the upper-right block into the lower-left block, after transposing
    block<12, 6>(6,0) = (block<6, 12>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = base_Ic;
    return *this;
}

#undef DATA
#undef F

void iit::popi::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint lh_kfe_joint, index 11 :
    L(11, 11) = ScalarTraits::sqrt(L(11, 11));
    L(11, 10) = L(11, 10) / L(11, 11);
    L(11, 9) = L(11, 9) / L(11, 11);
    L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);
    L(10, 9) = L(10, 9) - L(11, 10) * L(11, 9);
    L(9, 9) = L(9, 9) - L(11, 9) * L(11, 9);
    
    // Joint lh_hfe_joint, index 10 :
    L(10, 10) = ScalarTraits::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    
    // Joint lh_haa_joint, index 9 :
    L(9, 9) = ScalarTraits::sqrt(L(9, 9));
    
    // Joint rh_kfe_joint, index 8 :
    L(8, 8) = ScalarTraits::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    
    // Joint rh_hfe_joint, index 7 :
    L(7, 7) = ScalarTraits::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    
    // Joint rh_haa_joint, index 6 :
    L(6, 6) = ScalarTraits::sqrt(L(6, 6));
    
    // Joint lf_kfe_joint, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint lf_hfe_joint, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint lf_haa_joint, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    
    // Joint rf_kfe_joint, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint rf_hfe_joint, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint rf_haa_joint, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void iit::popi::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(6, 6) =  + (Linv(6, 6) * Linv(6, 6));
    inverse(7, 7) =  + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(8, 8) =  + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(9, 9) =  + (Linv(9, 9) * Linv(9, 9));
    inverse(10, 10) =  + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(11, 11) =  + (Linv(11, 9) * Linv(11, 9)) + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
    inverse(11, 10) =  + (Linv(11, 9) * Linv(10, 9)) + (Linv(11, 10) * Linv(10, 10));
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =  + (Linv(11, 9) * Linv(9, 9));
    inverse(9, 11) = inverse(11, 9);
}

void iit::popi::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
    Linv(11, 9) = - Linv(9, 9) * ((Linv(11, 10) * L(10, 9)) + (Linv(11, 11) * L(11, 9)) + 0);
}
