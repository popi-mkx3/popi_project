#ifndef POPI_JACOBIANS_H_
#define POPI_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "model_constants.h"

namespace iit {
namespace popi {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:

    protected:
        Parameters params;

};


}
}

#endif
