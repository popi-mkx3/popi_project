#include <iit/robcogen/test/dynamics_consistency.h>

#include <iit/robots/popi/inertia_properties.h>
#include <iit/robots/popi/transforms.h>
#include <iit/robots/popi/traits.h>
#include <iit/robots/popi/kinematics_parameters.h>
#include <iit/robots/popi/dynamics_parameters.h>

using namespace iit;
using namespace iit::popi;

/**
 * This program calls the dynamics-consistency-test implemented in
 * iit::robcogen::test.
 *
 * No arguments are required, all the relevant joint-space quantities are
 * randomly generated.
 *
 * The test prints some output on stdout. All the numerical values should be
 * zero; if that is not the case, there is some inconsistency among the generated
 * dynamics algorithms.
 */
int main(int argc, char** argv)
{
    MotionTransforms xm;
    ForceTransforms  xf;
    dyn::InertiaProperties ip;

    robcogen::test::consistencyTests<Traits>(ip, xm, xf);

    return 0;
}
