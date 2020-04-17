#ifndef _IIT_POPI_SL__ROBOGEN_GLOBALS_H_
#define _IIT_POPI_SL__ROBOGEN_GLOBALS_H_

#include <iit/robots/popi/declarations.h>
#include <iit/robots/popi/kinematics_parameters.h>
#include <iit/robots/popi/transforms.h>
#include <iit/robots/popi/inertia_properties.h>
#include <iit/robots/popi/forward_dynamics.h>
#include <iit/robots/popi/inverse_dynamics.h>

#include <SL.h>
#include <SL_user.h>

namespace robot = iit::popi;

namespace iit {
namespace popi {
namespace SL {

extern iit::popi::HomogeneousTransforms* homogeneousTransforms;
extern iit::popi::MotionTransforms* motionTransforms;
extern iit::popi::ForceTransforms* forceTransforms;
extern iit::popi::dyn::InertiaProperties* linksInertia;
extern iit::popi::dyn::ForwardDynamics*   fwdDynEngine;
extern iit::popi::dyn::InverseDynamics*   invDynEngine;


extern iit::popi::HomogeneousTransforms::MatrixType world_X_base;


inline void updateEndeffectorsParams(SL_endeff* eff) {
    //TODO
}

void createDefaultTransformsAndDynamics();

void update__world_X_base(const SL_Cstate& base_pos,const SL_quat& base_orient);

}
}
}



#endif
