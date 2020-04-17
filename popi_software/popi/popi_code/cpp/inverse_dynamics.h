#ifndef IIT_POPI_INVERSE_DYNAMICS_H_
#define IIT_POPI_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace popi {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot popi.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot popi, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_a,
        const Acceleration& g, const Velocity& base_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_a,
        const Acceleration& g, const Velocity& base_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_base() const { return base_f; }
    const Velocity& getVelocity_EpauleAVD() const { return EpauleAVD_v; }
    const Acceleration& getAcceleration_EpauleAVD() const { return EpauleAVD_a; }
    const Force& getForce_EpauleAVD() const { return EpauleAVD_f; }
    const Velocity& getVelocity_HJambeAVD() const { return HJambeAVD_v; }
    const Acceleration& getAcceleration_HJambeAVD() const { return HJambeAVD_a; }
    const Force& getForce_HJambeAVD() const { return HJambeAVD_f; }
    const Velocity& getVelocity_BJambeAVD() const { return BJambeAVD_v; }
    const Acceleration& getAcceleration_BJambeAVD() const { return BJambeAVD_a; }
    const Force& getForce_BJambeAVD() const { return BJambeAVD_f; }
    const Velocity& getVelocity_EpauleAVG() const { return EpauleAVG_v; }
    const Acceleration& getAcceleration_EpauleAVG() const { return EpauleAVG_a; }
    const Force& getForce_EpauleAVG() const { return EpauleAVG_f; }
    const Velocity& getVelocity_HJambeAVG() const { return HJambeAVG_v; }
    const Acceleration& getAcceleration_HJambeAVG() const { return HJambeAVG_a; }
    const Force& getForce_HJambeAVG() const { return HJambeAVG_f; }
    const Velocity& getVelocity_BJambeAVG() const { return BJambeAVG_v; }
    const Acceleration& getAcceleration_BJambeAVG() const { return BJambeAVG_a; }
    const Force& getForce_BJambeAVG() const { return BJambeAVG_f; }
    const Velocity& getVelocity_EpauleARD() const { return EpauleARD_v; }
    const Acceleration& getAcceleration_EpauleARD() const { return EpauleARD_a; }
    const Force& getForce_EpauleARD() const { return EpauleARD_f; }
    const Velocity& getVelocity_HJambeARD() const { return HJambeARD_v; }
    const Acceleration& getAcceleration_HJambeARD() const { return HJambeARD_a; }
    const Force& getForce_HJambeARD() const { return HJambeARD_f; }
    const Velocity& getVelocity_BJambeARD() const { return BJambeARD_v; }
    const Acceleration& getAcceleration_BJambeARD() const { return BJambeARD_a; }
    const Force& getForce_BJambeARD() const { return BJambeARD_f; }
    const Velocity& getVelocity_EpauleARG() const { return EpauleARG_v; }
    const Acceleration& getAcceleration_EpauleARG() const { return EpauleARG_a; }
    const Force& getForce_EpauleARG() const { return EpauleARG_f; }
    const Velocity& getVelocity_HJambeARG() const { return HJambeARG_v; }
    const Acceleration& getAcceleration_HJambeARG() const { return HJambeARG_a; }
    const Force& getForce_HJambeARG() const { return HJambeARG_f; }
    const Velocity& getVelocity_BJambeARG() const { return BJambeARG_v; }
    const Acceleration& getAcceleration_BJambeARG() const { return BJambeARG_a; }
    const Force& getForce_BJambeARG() const { return BJambeARG_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'EpauleAVD' :
    const InertiaMatrix& EpauleAVD_I;
    Velocity      EpauleAVD_v;
    Acceleration  EpauleAVD_a;
    Force         EpauleAVD_f;
    // Link 'HJambeAVD' :
    const InertiaMatrix& HJambeAVD_I;
    Velocity      HJambeAVD_v;
    Acceleration  HJambeAVD_a;
    Force         HJambeAVD_f;
    // Link 'BJambeAVD' :
    const InertiaMatrix& BJambeAVD_I;
    Velocity      BJambeAVD_v;
    Acceleration  BJambeAVD_a;
    Force         BJambeAVD_f;
    // Link 'EpauleAVG' :
    const InertiaMatrix& EpauleAVG_I;
    Velocity      EpauleAVG_v;
    Acceleration  EpauleAVG_a;
    Force         EpauleAVG_f;
    // Link 'HJambeAVG' :
    const InertiaMatrix& HJambeAVG_I;
    Velocity      HJambeAVG_v;
    Acceleration  HJambeAVG_a;
    Force         HJambeAVG_f;
    // Link 'BJambeAVG' :
    const InertiaMatrix& BJambeAVG_I;
    Velocity      BJambeAVG_v;
    Acceleration  BJambeAVG_a;
    Force         BJambeAVG_f;
    // Link 'EpauleARD' :
    const InertiaMatrix& EpauleARD_I;
    Velocity      EpauleARD_v;
    Acceleration  EpauleARD_a;
    Force         EpauleARD_f;
    // Link 'HJambeARD' :
    const InertiaMatrix& HJambeARD_I;
    Velocity      HJambeARD_v;
    Acceleration  HJambeARD_a;
    Force         HJambeARD_f;
    // Link 'BJambeARD' :
    const InertiaMatrix& BJambeARD_I;
    Velocity      BJambeARD_v;
    Acceleration  BJambeARD_a;
    Force         BJambeARD_f;
    // Link 'EpauleARG' :
    const InertiaMatrix& EpauleARG_I;
    Velocity      EpauleARG_v;
    Acceleration  EpauleARG_a;
    Force         EpauleARG_f;
    // Link 'HJambeARG' :
    const InertiaMatrix& HJambeARG_I;
    Velocity      HJambeARG_v;
    Acceleration  HJambeARG_a;
    Force         HJambeARG_f;
    // Link 'BJambeARG' :
    const InertiaMatrix& BJambeARG_I;
    Velocity      BJambeARG_v;
    Acceleration  BJambeARG_a;
    Force         BJambeARG_f;

    // The robot base
    const InertiaMatrix& base_I;
    InertiaMatrix base_Ic;
    Force         base_f;
    // The composite inertia tensors
    InertiaMatrix EpauleAVD_Ic;
    InertiaMatrix HJambeAVD_Ic;
    const InertiaMatrix& BJambeAVD_Ic;
    InertiaMatrix EpauleAVG_Ic;
    InertiaMatrix HJambeAVG_Ic;
    const InertiaMatrix& BJambeAVG_Ic;
    InertiaMatrix EpauleARD_Ic;
    InertiaMatrix HJambeARD_Ic;
    const InertiaMatrix& BJambeARD_Ic;
    InertiaMatrix EpauleARG_Ic;
    InertiaMatrix HJambeARG_Ic;
    const InertiaMatrix& BJambeARG_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_EpauleAVD_X_fr_base)(q);
    (xm->fr_HJambeAVD_X_fr_EpauleAVD)(q);
    (xm->fr_BJambeAVD_X_fr_HJambeAVD)(q);
    (xm->fr_EpauleAVG_X_fr_base)(q);
    (xm->fr_HJambeAVG_X_fr_EpauleAVG)(q);
    (xm->fr_BJambeAVG_X_fr_HJambeAVG)(q);
    (xm->fr_EpauleARD_X_fr_base)(q);
    (xm->fr_HJambeARD_X_fr_EpauleARD)(q);
    (xm->fr_BJambeARD_X_fr_HJambeARD)(q);
    (xm->fr_EpauleARG_X_fr_base)(q);
    (xm->fr_HJambeARG_X_fr_EpauleARG)(q);
    (xm->fr_BJambeARG_X_fr_HJambeARG)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_a, g, base_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_v,
        baseAccel, qd, qdd, fext);
}

}
}
}

#endif
