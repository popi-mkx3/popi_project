#ifndef IIT_ROBOT_POPI_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_POPI_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace popi {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot popi.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot popi, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_a
     * \param base_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_a, // output parameters,
       const Velocity& base_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_a, // output parameters,
        const Velocity& base_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base'
    Matrix66 base_AI;
    Force base_p;

    // Link 'EpauleAVD' :
    Matrix66 EpauleAVD_AI;
    Velocity EpauleAVD_a;
    Velocity EpauleAVD_v;
    Velocity EpauleAVD_c;
    Force    EpauleAVD_p;

    Column6 EpauleAVD_U;
    Scalar EpauleAVD_D;
    Scalar EpauleAVD_u;
    // Link 'HJambeAVD' :
    Matrix66 HJambeAVD_AI;
    Velocity HJambeAVD_a;
    Velocity HJambeAVD_v;
    Velocity HJambeAVD_c;
    Force    HJambeAVD_p;

    Column6 HJambeAVD_U;
    Scalar HJambeAVD_D;
    Scalar HJambeAVD_u;
    // Link 'BJambeAVD' :
    Matrix66 BJambeAVD_AI;
    Velocity BJambeAVD_a;
    Velocity BJambeAVD_v;
    Velocity BJambeAVD_c;
    Force    BJambeAVD_p;

    Column6 BJambeAVD_U;
    Scalar BJambeAVD_D;
    Scalar BJambeAVD_u;
    // Link 'EpauleAVG' :
    Matrix66 EpauleAVG_AI;
    Velocity EpauleAVG_a;
    Velocity EpauleAVG_v;
    Velocity EpauleAVG_c;
    Force    EpauleAVG_p;

    Column6 EpauleAVG_U;
    Scalar EpauleAVG_D;
    Scalar EpauleAVG_u;
    // Link 'HJambeAVG' :
    Matrix66 HJambeAVG_AI;
    Velocity HJambeAVG_a;
    Velocity HJambeAVG_v;
    Velocity HJambeAVG_c;
    Force    HJambeAVG_p;

    Column6 HJambeAVG_U;
    Scalar HJambeAVG_D;
    Scalar HJambeAVG_u;
    // Link 'BJambeAVG' :
    Matrix66 BJambeAVG_AI;
    Velocity BJambeAVG_a;
    Velocity BJambeAVG_v;
    Velocity BJambeAVG_c;
    Force    BJambeAVG_p;

    Column6 BJambeAVG_U;
    Scalar BJambeAVG_D;
    Scalar BJambeAVG_u;
    // Link 'EpauleARD' :
    Matrix66 EpauleARD_AI;
    Velocity EpauleARD_a;
    Velocity EpauleARD_v;
    Velocity EpauleARD_c;
    Force    EpauleARD_p;

    Column6 EpauleARD_U;
    Scalar EpauleARD_D;
    Scalar EpauleARD_u;
    // Link 'HJambeARD' :
    Matrix66 HJambeARD_AI;
    Velocity HJambeARD_a;
    Velocity HJambeARD_v;
    Velocity HJambeARD_c;
    Force    HJambeARD_p;

    Column6 HJambeARD_U;
    Scalar HJambeARD_D;
    Scalar HJambeARD_u;
    // Link 'BJambeARD' :
    Matrix66 BJambeARD_AI;
    Velocity BJambeARD_a;
    Velocity BJambeARD_v;
    Velocity BJambeARD_c;
    Force    BJambeARD_p;

    Column6 BJambeARD_U;
    Scalar BJambeARD_D;
    Scalar BJambeARD_u;
    // Link 'EpauleARG' :
    Matrix66 EpauleARG_AI;
    Velocity EpauleARG_a;
    Velocity EpauleARG_v;
    Velocity EpauleARG_c;
    Force    EpauleARG_p;

    Column6 EpauleARG_U;
    Scalar EpauleARG_D;
    Scalar EpauleARG_u;
    // Link 'HJambeARG' :
    Matrix66 HJambeARG_AI;
    Velocity HJambeARG_a;
    Velocity HJambeARG_v;
    Velocity HJambeARG_c;
    Force    HJambeARG_p;

    Column6 HJambeARG_U;
    Scalar HJambeARG_D;
    Scalar HJambeARG_u;
    // Link 'BJambeARG' :
    Matrix66 BJambeARG_AI;
    Velocity BJambeARG_a;
    Velocity BJambeARG_v;
    Velocity BJambeARG_c;
    Force    BJambeARG_p;

    Column6 BJambeARG_U;
    Scalar BJambeARG_D;
    Scalar BJambeARG_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_EpauleAVD_X_fr_base)(q);
    (motionTransforms-> fr_HJambeAVD_X_fr_EpauleAVD)(q);
    (motionTransforms-> fr_BJambeAVD_X_fr_HJambeAVD)(q);
    (motionTransforms-> fr_EpauleAVG_X_fr_base)(q);
    (motionTransforms-> fr_HJambeAVG_X_fr_EpauleAVG)(q);
    (motionTransforms-> fr_BJambeAVG_X_fr_HJambeAVG)(q);
    (motionTransforms-> fr_EpauleARD_X_fr_base)(q);
    (motionTransforms-> fr_HJambeARD_X_fr_EpauleARD)(q);
    (motionTransforms-> fr_BJambeARD_X_fr_HJambeARD)(q);
    (motionTransforms-> fr_EpauleARG_X_fr_base)(q);
    (motionTransforms-> fr_HJambeARG_X_fr_EpauleARG)(q);
    (motionTransforms-> fr_BJambeARG_X_fr_HJambeARG)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& base_a, // output parameters,
    const Velocity& base_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_a, base_v, g, qd, tau, fext);
}

}
}
}

#endif
