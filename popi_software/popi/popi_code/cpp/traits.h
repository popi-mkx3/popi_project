#ifndef IIT_ROBOGEN__POPI_TRAITS_H_
#define IIT_ROBOGEN__POPI_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace popi {

struct Traits {
    typedef typename popi::ScalarTraits ScalarTraits;

    typedef typename popi::JointState JointState;

    typedef typename popi::JointIdentifiers JointID;
    typedef typename popi::LinkIdentifiers  LinkID;

    typedef typename popi::HomogeneousTransforms HomogeneousTransforms;
    typedef typename popi::MotionTransforms MotionTransforms;
    typedef typename popi::ForceTransforms ForceTransforms;

    typedef typename popi::dyn::InertiaProperties InertiaProperties;
    typedef typename popi::dyn::ForwardDynamics FwdDynEngine;
    typedef typename popi::dyn::InverseDynamics InvDynEngine;
    typedef typename popi::dyn::JSIM JSIM;

    static const int joints_count = popi::jointsCount;
    static const int links_count  = popi::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return popi::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return popi::orderedLinkIDs;
}

}
}

#endif
