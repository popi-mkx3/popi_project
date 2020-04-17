#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

iit::popi::dyn::InertiaProperties::InertiaProperties()
{
    com_base = Vector3(0.0,0.0,0.0);
    tensor_base.fill(
        m_base,
        com_base,
        Utils::buildInertiaTensor<Scalar>(ix_base,iy_base,iz_base,0.0,0.0,0.0) );

    com_EpauleAVD = Vector3(0.0,0.0,0.0);
    tensor_EpauleAVD.fill(
        m_EpauleAVD,
        com_EpauleAVD,
        Utils::buildInertiaTensor<Scalar>(ix_EpauleAVD,iy_EpauleAVD,iz_EpauleAVD,ixy_EpauleAVD,ixz_EpauleAVD,iyz_EpauleAVD) );

    com_HJambeAVD = Vector3(0.0,0.0,0.0);
    tensor_HJambeAVD.fill(
        m_HJambeAVD,
        com_HJambeAVD,
        Utils::buildInertiaTensor<Scalar>(ix_HJambeAVD,iy_HJambeAVD,iz_HJambeAVD,ixy_HJambeAVD,ixz_HJambeAVD,iyz_HJambeAVD) );

    com_BJambeAVD = Vector3(0.0,0.0,0.0);
    tensor_BJambeAVD.fill(
        m_BJambeAVD,
        com_BJambeAVD,
        Utils::buildInertiaTensor<Scalar>(ix_BJambeAVD,iy_BJambeAVD,iz_BJambeAVD,ixy_BJambeAVD,ixz_BJambeAVD,0.0) );

    com_EpauleAVG = Vector3(0.0,0.0,0.0);
    tensor_EpauleAVG.fill(
        m_EpauleAVG,
        com_EpauleAVG,
        Utils::buildInertiaTensor<Scalar>(ix_EpauleAVG,iy_EpauleAVG,iz_EpauleAVG,ixy_EpauleAVG,ixz_EpauleAVG,iyz_EpauleAVG) );

    com_HJambeAVG = Vector3(0.0,0.0,0.0);
    tensor_HJambeAVG.fill(
        m_HJambeAVG,
        com_HJambeAVG,
        Utils::buildInertiaTensor<Scalar>(ix_HJambeAVG,iy_HJambeAVG,iz_HJambeAVG,ixy_HJambeAVG,ixz_HJambeAVG,iyz_HJambeAVG) );

    com_BJambeAVG = Vector3(0.0,0.0,0.0);
    tensor_BJambeAVG.fill(
        m_BJambeAVG,
        com_BJambeAVG,
        Utils::buildInertiaTensor<Scalar>(ix_BJambeAVG,iy_BJambeAVG,iz_BJambeAVG,ixy_BJambeAVG,ixz_BJambeAVG,iyz_BJambeAVG) );

    com_EpauleARD = Vector3(0.0,0.0,0.0);
    tensor_EpauleARD.fill(
        m_EpauleARD,
        com_EpauleARD,
        Utils::buildInertiaTensor<Scalar>(ix_EpauleARD,iy_EpauleARD,iz_EpauleARD,ixy_EpauleARD,ixz_EpauleARD,iyz_EpauleARD) );

    com_HJambeARD = Vector3(0.0,0.0,0.0);
    tensor_HJambeARD.fill(
        m_HJambeARD,
        com_HJambeARD,
        Utils::buildInertiaTensor<Scalar>(ix_HJambeARD,iy_HJambeARD,iz_HJambeARD,ixy_HJambeARD,ixz_HJambeARD,iyz_HJambeARD) );

    com_BJambeARD = Vector3(0.0,0.0,0.0);
    tensor_BJambeARD.fill(
        m_BJambeARD,
        com_BJambeARD,
        Utils::buildInertiaTensor<Scalar>(ix_BJambeARD,iy_BJambeARD,iz_BJambeARD,0.0,ixz_BJambeARD,0.0) );

    com_EpauleARG = Vector3(0.0,0.0,0.0);
    tensor_EpauleARG.fill(
        m_EpauleARG,
        com_EpauleARG,
        Utils::buildInertiaTensor<Scalar>(ix_EpauleARG,iy_EpauleARG,iz_EpauleARG,ixy_EpauleARG,ixz_EpauleARG,iyz_EpauleARG) );

    com_HJambeARG = Vector3(0.0,0.0,0.0);
    tensor_HJambeARG.fill(
        m_HJambeARG,
        com_HJambeARG,
        Utils::buildInertiaTensor<Scalar>(ix_HJambeARG,iy_HJambeARG,iz_HJambeARG,ixy_HJambeARG,ixz_HJambeARG,iyz_HJambeARG) );

    com_BJambeARG = Vector3(0.0,0.0,0.0);
    tensor_BJambeARG.fill(
        m_BJambeARG,
        com_BJambeARG,
        Utils::buildInertiaTensor<Scalar>(ix_BJambeARG,iy_BJambeARG,iz_BJambeARG,ixy_BJambeARG,ixz_BJambeARG,iyz_BJambeARG) );

}


void iit::popi::dyn::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
