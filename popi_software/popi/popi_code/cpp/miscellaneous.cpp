#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::popi;
using namespace iit::popi::dyn;

Vector3 iit::popi::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_base() * inertiaProps.getMass_base();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_rf_haa_joint_chain;
    HomogeneousTransforms::MatrixType base_X_lf_haa_joint_chain;
    HomogeneousTransforms::MatrixType base_X_rh_haa_joint_chain;
    HomogeneousTransforms::MatrixType base_X_lh_haa_joint_chain;
    
    
    base_X_rf_haa_joint_chain = tmpX * ht.fr_base_X_fr_EpauleAVD;
    tmpSum += inertiaProps.getMass_EpauleAVD() *
            ( iit::rbd::Utils::transform(base_X_rf_haa_joint_chain, inertiaProps.getCOM_EpauleAVD()));
    
    base_X_rf_haa_joint_chain = base_X_rf_haa_joint_chain * ht.fr_EpauleAVD_X_fr_HJambeAVD;
    tmpSum += inertiaProps.getMass_HJambeAVD() *
            ( iit::rbd::Utils::transform(base_X_rf_haa_joint_chain, inertiaProps.getCOM_HJambeAVD()));
    
    base_X_rf_haa_joint_chain = base_X_rf_haa_joint_chain * ht.fr_HJambeAVD_X_fr_BJambeAVD;
    tmpSum += inertiaProps.getMass_BJambeAVD() *
            ( iit::rbd::Utils::transform(base_X_rf_haa_joint_chain, inertiaProps.getCOM_BJambeAVD()));
    
    base_X_lf_haa_joint_chain = tmpX * ht.fr_base_X_fr_EpauleAVG;
    tmpSum += inertiaProps.getMass_EpauleAVG() *
            ( iit::rbd::Utils::transform(base_X_lf_haa_joint_chain, inertiaProps.getCOM_EpauleAVG()));
    
    base_X_lf_haa_joint_chain = base_X_lf_haa_joint_chain * ht.fr_EpauleAVG_X_fr_HJambeAVG;
    tmpSum += inertiaProps.getMass_HJambeAVG() *
            ( iit::rbd::Utils::transform(base_X_lf_haa_joint_chain, inertiaProps.getCOM_HJambeAVG()));
    
    base_X_lf_haa_joint_chain = base_X_lf_haa_joint_chain * ht.fr_HJambeAVG_X_fr_BJambeAVG;
    tmpSum += inertiaProps.getMass_BJambeAVG() *
            ( iit::rbd::Utils::transform(base_X_lf_haa_joint_chain, inertiaProps.getCOM_BJambeAVG()));
    
    base_X_rh_haa_joint_chain = tmpX * ht.fr_base_X_fr_EpauleARD;
    tmpSum += inertiaProps.getMass_EpauleARD() *
            ( iit::rbd::Utils::transform(base_X_rh_haa_joint_chain, inertiaProps.getCOM_EpauleARD()));
    
    base_X_rh_haa_joint_chain = base_X_rh_haa_joint_chain * ht.fr_EpauleARD_X_fr_HJambeARD;
    tmpSum += inertiaProps.getMass_HJambeARD() *
            ( iit::rbd::Utils::transform(base_X_rh_haa_joint_chain, inertiaProps.getCOM_HJambeARD()));
    
    base_X_rh_haa_joint_chain = base_X_rh_haa_joint_chain * ht.fr_HJambeARD_X_fr_BJambeARD;
    tmpSum += inertiaProps.getMass_BJambeARD() *
            ( iit::rbd::Utils::transform(base_X_rh_haa_joint_chain, inertiaProps.getCOM_BJambeARD()));
    
    base_X_lh_haa_joint_chain = tmpX * ht.fr_base_X_fr_EpauleARG;
    tmpSum += inertiaProps.getMass_EpauleARG() *
            ( iit::rbd::Utils::transform(base_X_lh_haa_joint_chain, inertiaProps.getCOM_EpauleARG()));
    
    base_X_lh_haa_joint_chain = base_X_lh_haa_joint_chain * ht.fr_EpauleARG_X_fr_HJambeARG;
    tmpSum += inertiaProps.getMass_HJambeARG() *
            ( iit::rbd::Utils::transform(base_X_lh_haa_joint_chain, inertiaProps.getCOM_HJambeARG()));
    
    base_X_lh_haa_joint_chain = base_X_lh_haa_joint_chain * ht.fr_HJambeARG_X_fr_BJambeARG;
    tmpSum += inertiaProps.getMass_BJambeARG() *
            ( iit::rbd::Utils::transform(base_X_lh_haa_joint_chain, inertiaProps.getCOM_BJambeARG()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 iit::popi::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_EpauleAVD(q);
    ht.fr_base_X_fr_EpauleAVG(q);
    ht.fr_base_X_fr_EpauleARD(q);
    ht.fr_base_X_fr_EpauleARG(q);
    ht.fr_EpauleAVD_X_fr_HJambeAVD(q);
    ht.fr_HJambeAVD_X_fr_BJambeAVD(q);
    ht.fr_EpauleAVG_X_fr_HJambeAVG(q);
    ht.fr_HJambeAVG_X_fr_BJambeAVG(q);
    ht.fr_EpauleARD_X_fr_HJambeARD(q);
    ht.fr_HJambeARD_X_fr_BJambeARD(q);
    ht.fr_EpauleARG_X_fr_HJambeARG(q);
    ht.fr_HJambeARG_X_fr_BJambeARG(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
