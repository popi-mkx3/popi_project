#ifndef POPI_TRANSFORMS_H_
#define POPI_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

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

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_EpauleAVD_X_fr_base : public TransformMotion<Type_fr_EpauleAVD_X_fr_base>
    {
        Type_fr_EpauleAVD_X_fr_base();
        const Type_fr_EpauleAVD_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleAVD : public TransformMotion<Type_fr_base_X_fr_EpauleAVD>
    {
        Type_fr_base_X_fr_EpauleAVD();
        const Type_fr_base_X_fr_EpauleAVD& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVD_X_fr_EpauleAVD : public TransformMotion<Type_fr_HJambeAVD_X_fr_EpauleAVD>
    {
        Type_fr_HJambeAVD_X_fr_EpauleAVD();
        const Type_fr_HJambeAVD_X_fr_EpauleAVD& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVD_X_fr_HJambeAVD : public TransformMotion<Type_fr_EpauleAVD_X_fr_HJambeAVD>
    {
        Type_fr_EpauleAVD_X_fr_HJambeAVD();
        const Type_fr_EpauleAVD_X_fr_HJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_BJambeAVD_X_fr_HJambeAVD : public TransformMotion<Type_fr_BJambeAVD_X_fr_HJambeAVD>
    {
        Type_fr_BJambeAVD_X_fr_HJambeAVD();
        const Type_fr_BJambeAVD_X_fr_HJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVD_X_fr_BJambeAVD : public TransformMotion<Type_fr_HJambeAVD_X_fr_BJambeAVD>
    {
        Type_fr_HJambeAVD_X_fr_BJambeAVD();
        const Type_fr_HJambeAVD_X_fr_BJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVG_X_fr_base : public TransformMotion<Type_fr_EpauleAVG_X_fr_base>
    {
        Type_fr_EpauleAVG_X_fr_base();
        const Type_fr_EpauleAVG_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleAVG : public TransformMotion<Type_fr_base_X_fr_EpauleAVG>
    {
        Type_fr_base_X_fr_EpauleAVG();
        const Type_fr_base_X_fr_EpauleAVG& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVG_X_fr_EpauleAVG : public TransformMotion<Type_fr_HJambeAVG_X_fr_EpauleAVG>
    {
        Type_fr_HJambeAVG_X_fr_EpauleAVG();
        const Type_fr_HJambeAVG_X_fr_EpauleAVG& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVG_X_fr_HJambeAVG : public TransformMotion<Type_fr_EpauleAVG_X_fr_HJambeAVG>
    {
        Type_fr_EpauleAVG_X_fr_HJambeAVG();
        const Type_fr_EpauleAVG_X_fr_HJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_BJambeAVG_X_fr_HJambeAVG : public TransformMotion<Type_fr_BJambeAVG_X_fr_HJambeAVG>
    {
        Type_fr_BJambeAVG_X_fr_HJambeAVG();
        const Type_fr_BJambeAVG_X_fr_HJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVG_X_fr_BJambeAVG : public TransformMotion<Type_fr_HJambeAVG_X_fr_BJambeAVG>
    {
        Type_fr_HJambeAVG_X_fr_BJambeAVG();
        const Type_fr_HJambeAVG_X_fr_BJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_EpauleARD_X_fr_base : public TransformMotion<Type_fr_EpauleARD_X_fr_base>
    {
        Type_fr_EpauleARD_X_fr_base();
        const Type_fr_EpauleARD_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleARD : public TransformMotion<Type_fr_base_X_fr_EpauleARD>
    {
        Type_fr_base_X_fr_EpauleARD();
        const Type_fr_base_X_fr_EpauleARD& update(const state_t&);
    };
    
    struct Type_fr_HJambeARD_X_fr_EpauleARD : public TransformMotion<Type_fr_HJambeARD_X_fr_EpauleARD>
    {
        Type_fr_HJambeARD_X_fr_EpauleARD();
        const Type_fr_HJambeARD_X_fr_EpauleARD& update(const state_t&);
    };
    
    struct Type_fr_EpauleARD_X_fr_HJambeARD : public TransformMotion<Type_fr_EpauleARD_X_fr_HJambeARD>
    {
        Type_fr_EpauleARD_X_fr_HJambeARD();
        const Type_fr_EpauleARD_X_fr_HJambeARD& update(const state_t&);
    };
    
    struct Type_fr_BJambeARD_X_fr_HJambeARD : public TransformMotion<Type_fr_BJambeARD_X_fr_HJambeARD>
    {
        Type_fr_BJambeARD_X_fr_HJambeARD();
        const Type_fr_BJambeARD_X_fr_HJambeARD& update(const state_t&);
    };
    
    struct Type_fr_HJambeARD_X_fr_BJambeARD : public TransformMotion<Type_fr_HJambeARD_X_fr_BJambeARD>
    {
        Type_fr_HJambeARD_X_fr_BJambeARD();
        const Type_fr_HJambeARD_X_fr_BJambeARD& update(const state_t&);
    };
    
    struct Type_fr_EpauleARG_X_fr_base : public TransformMotion<Type_fr_EpauleARG_X_fr_base>
    {
        Type_fr_EpauleARG_X_fr_base();
        const Type_fr_EpauleARG_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleARG : public TransformMotion<Type_fr_base_X_fr_EpauleARG>
    {
        Type_fr_base_X_fr_EpauleARG();
        const Type_fr_base_X_fr_EpauleARG& update(const state_t&);
    };
    
    struct Type_fr_HJambeARG_X_fr_EpauleARG : public TransformMotion<Type_fr_HJambeARG_X_fr_EpauleARG>
    {
        Type_fr_HJambeARG_X_fr_EpauleARG();
        const Type_fr_HJambeARG_X_fr_EpauleARG& update(const state_t&);
    };
    
    struct Type_fr_EpauleARG_X_fr_HJambeARG : public TransformMotion<Type_fr_EpauleARG_X_fr_HJambeARG>
    {
        Type_fr_EpauleARG_X_fr_HJambeARG();
        const Type_fr_EpauleARG_X_fr_HJambeARG& update(const state_t&);
    };
    
    struct Type_fr_BJambeARG_X_fr_HJambeARG : public TransformMotion<Type_fr_BJambeARG_X_fr_HJambeARG>
    {
        Type_fr_BJambeARG_X_fr_HJambeARG();
        const Type_fr_BJambeARG_X_fr_HJambeARG& update(const state_t&);
    };
    
    struct Type_fr_HJambeARG_X_fr_BJambeARG : public TransformMotion<Type_fr_HJambeARG_X_fr_BJambeARG>
    {
        Type_fr_HJambeARG_X_fr_BJambeARG();
        const Type_fr_HJambeARG_X_fr_BJambeARG& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_EpauleAVD_X_fr_base fr_EpauleAVD_X_fr_base;
    Type_fr_base_X_fr_EpauleAVD fr_base_X_fr_EpauleAVD;
    Type_fr_HJambeAVD_X_fr_EpauleAVD fr_HJambeAVD_X_fr_EpauleAVD;
    Type_fr_EpauleAVD_X_fr_HJambeAVD fr_EpauleAVD_X_fr_HJambeAVD;
    Type_fr_BJambeAVD_X_fr_HJambeAVD fr_BJambeAVD_X_fr_HJambeAVD;
    Type_fr_HJambeAVD_X_fr_BJambeAVD fr_HJambeAVD_X_fr_BJambeAVD;
    Type_fr_EpauleAVG_X_fr_base fr_EpauleAVG_X_fr_base;
    Type_fr_base_X_fr_EpauleAVG fr_base_X_fr_EpauleAVG;
    Type_fr_HJambeAVG_X_fr_EpauleAVG fr_HJambeAVG_X_fr_EpauleAVG;
    Type_fr_EpauleAVG_X_fr_HJambeAVG fr_EpauleAVG_X_fr_HJambeAVG;
    Type_fr_BJambeAVG_X_fr_HJambeAVG fr_BJambeAVG_X_fr_HJambeAVG;
    Type_fr_HJambeAVG_X_fr_BJambeAVG fr_HJambeAVG_X_fr_BJambeAVG;
    Type_fr_EpauleARD_X_fr_base fr_EpauleARD_X_fr_base;
    Type_fr_base_X_fr_EpauleARD fr_base_X_fr_EpauleARD;
    Type_fr_HJambeARD_X_fr_EpauleARD fr_HJambeARD_X_fr_EpauleARD;
    Type_fr_EpauleARD_X_fr_HJambeARD fr_EpauleARD_X_fr_HJambeARD;
    Type_fr_BJambeARD_X_fr_HJambeARD fr_BJambeARD_X_fr_HJambeARD;
    Type_fr_HJambeARD_X_fr_BJambeARD fr_HJambeARD_X_fr_BJambeARD;
    Type_fr_EpauleARG_X_fr_base fr_EpauleARG_X_fr_base;
    Type_fr_base_X_fr_EpauleARG fr_base_X_fr_EpauleARG;
    Type_fr_HJambeARG_X_fr_EpauleARG fr_HJambeARG_X_fr_EpauleARG;
    Type_fr_EpauleARG_X_fr_HJambeARG fr_EpauleARG_X_fr_HJambeARG;
    Type_fr_BJambeARG_X_fr_HJambeARG fr_BJambeARG_X_fr_HJambeARG;
    Type_fr_HJambeARG_X_fr_BJambeARG fr_HJambeARG_X_fr_BJambeARG;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_EpauleAVD_X_fr_base : public TransformForce<Type_fr_EpauleAVD_X_fr_base>
    {
        Type_fr_EpauleAVD_X_fr_base();
        const Type_fr_EpauleAVD_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleAVD : public TransformForce<Type_fr_base_X_fr_EpauleAVD>
    {
        Type_fr_base_X_fr_EpauleAVD();
        const Type_fr_base_X_fr_EpauleAVD& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVD_X_fr_EpauleAVD : public TransformForce<Type_fr_HJambeAVD_X_fr_EpauleAVD>
    {
        Type_fr_HJambeAVD_X_fr_EpauleAVD();
        const Type_fr_HJambeAVD_X_fr_EpauleAVD& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVD_X_fr_HJambeAVD : public TransformForce<Type_fr_EpauleAVD_X_fr_HJambeAVD>
    {
        Type_fr_EpauleAVD_X_fr_HJambeAVD();
        const Type_fr_EpauleAVD_X_fr_HJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_BJambeAVD_X_fr_HJambeAVD : public TransformForce<Type_fr_BJambeAVD_X_fr_HJambeAVD>
    {
        Type_fr_BJambeAVD_X_fr_HJambeAVD();
        const Type_fr_BJambeAVD_X_fr_HJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVD_X_fr_BJambeAVD : public TransformForce<Type_fr_HJambeAVD_X_fr_BJambeAVD>
    {
        Type_fr_HJambeAVD_X_fr_BJambeAVD();
        const Type_fr_HJambeAVD_X_fr_BJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVG_X_fr_base : public TransformForce<Type_fr_EpauleAVG_X_fr_base>
    {
        Type_fr_EpauleAVG_X_fr_base();
        const Type_fr_EpauleAVG_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleAVG : public TransformForce<Type_fr_base_X_fr_EpauleAVG>
    {
        Type_fr_base_X_fr_EpauleAVG();
        const Type_fr_base_X_fr_EpauleAVG& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVG_X_fr_EpauleAVG : public TransformForce<Type_fr_HJambeAVG_X_fr_EpauleAVG>
    {
        Type_fr_HJambeAVG_X_fr_EpauleAVG();
        const Type_fr_HJambeAVG_X_fr_EpauleAVG& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVG_X_fr_HJambeAVG : public TransformForce<Type_fr_EpauleAVG_X_fr_HJambeAVG>
    {
        Type_fr_EpauleAVG_X_fr_HJambeAVG();
        const Type_fr_EpauleAVG_X_fr_HJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_BJambeAVG_X_fr_HJambeAVG : public TransformForce<Type_fr_BJambeAVG_X_fr_HJambeAVG>
    {
        Type_fr_BJambeAVG_X_fr_HJambeAVG();
        const Type_fr_BJambeAVG_X_fr_HJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVG_X_fr_BJambeAVG : public TransformForce<Type_fr_HJambeAVG_X_fr_BJambeAVG>
    {
        Type_fr_HJambeAVG_X_fr_BJambeAVG();
        const Type_fr_HJambeAVG_X_fr_BJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_EpauleARD_X_fr_base : public TransformForce<Type_fr_EpauleARD_X_fr_base>
    {
        Type_fr_EpauleARD_X_fr_base();
        const Type_fr_EpauleARD_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleARD : public TransformForce<Type_fr_base_X_fr_EpauleARD>
    {
        Type_fr_base_X_fr_EpauleARD();
        const Type_fr_base_X_fr_EpauleARD& update(const state_t&);
    };
    
    struct Type_fr_HJambeARD_X_fr_EpauleARD : public TransformForce<Type_fr_HJambeARD_X_fr_EpauleARD>
    {
        Type_fr_HJambeARD_X_fr_EpauleARD();
        const Type_fr_HJambeARD_X_fr_EpauleARD& update(const state_t&);
    };
    
    struct Type_fr_EpauleARD_X_fr_HJambeARD : public TransformForce<Type_fr_EpauleARD_X_fr_HJambeARD>
    {
        Type_fr_EpauleARD_X_fr_HJambeARD();
        const Type_fr_EpauleARD_X_fr_HJambeARD& update(const state_t&);
    };
    
    struct Type_fr_BJambeARD_X_fr_HJambeARD : public TransformForce<Type_fr_BJambeARD_X_fr_HJambeARD>
    {
        Type_fr_BJambeARD_X_fr_HJambeARD();
        const Type_fr_BJambeARD_X_fr_HJambeARD& update(const state_t&);
    };
    
    struct Type_fr_HJambeARD_X_fr_BJambeARD : public TransformForce<Type_fr_HJambeARD_X_fr_BJambeARD>
    {
        Type_fr_HJambeARD_X_fr_BJambeARD();
        const Type_fr_HJambeARD_X_fr_BJambeARD& update(const state_t&);
    };
    
    struct Type_fr_EpauleARG_X_fr_base : public TransformForce<Type_fr_EpauleARG_X_fr_base>
    {
        Type_fr_EpauleARG_X_fr_base();
        const Type_fr_EpauleARG_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleARG : public TransformForce<Type_fr_base_X_fr_EpauleARG>
    {
        Type_fr_base_X_fr_EpauleARG();
        const Type_fr_base_X_fr_EpauleARG& update(const state_t&);
    };
    
    struct Type_fr_HJambeARG_X_fr_EpauleARG : public TransformForce<Type_fr_HJambeARG_X_fr_EpauleARG>
    {
        Type_fr_HJambeARG_X_fr_EpauleARG();
        const Type_fr_HJambeARG_X_fr_EpauleARG& update(const state_t&);
    };
    
    struct Type_fr_EpauleARG_X_fr_HJambeARG : public TransformForce<Type_fr_EpauleARG_X_fr_HJambeARG>
    {
        Type_fr_EpauleARG_X_fr_HJambeARG();
        const Type_fr_EpauleARG_X_fr_HJambeARG& update(const state_t&);
    };
    
    struct Type_fr_BJambeARG_X_fr_HJambeARG : public TransformForce<Type_fr_BJambeARG_X_fr_HJambeARG>
    {
        Type_fr_BJambeARG_X_fr_HJambeARG();
        const Type_fr_BJambeARG_X_fr_HJambeARG& update(const state_t&);
    };
    
    struct Type_fr_HJambeARG_X_fr_BJambeARG : public TransformForce<Type_fr_HJambeARG_X_fr_BJambeARG>
    {
        Type_fr_HJambeARG_X_fr_BJambeARG();
        const Type_fr_HJambeARG_X_fr_BJambeARG& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_EpauleAVD_X_fr_base fr_EpauleAVD_X_fr_base;
    Type_fr_base_X_fr_EpauleAVD fr_base_X_fr_EpauleAVD;
    Type_fr_HJambeAVD_X_fr_EpauleAVD fr_HJambeAVD_X_fr_EpauleAVD;
    Type_fr_EpauleAVD_X_fr_HJambeAVD fr_EpauleAVD_X_fr_HJambeAVD;
    Type_fr_BJambeAVD_X_fr_HJambeAVD fr_BJambeAVD_X_fr_HJambeAVD;
    Type_fr_HJambeAVD_X_fr_BJambeAVD fr_HJambeAVD_X_fr_BJambeAVD;
    Type_fr_EpauleAVG_X_fr_base fr_EpauleAVG_X_fr_base;
    Type_fr_base_X_fr_EpauleAVG fr_base_X_fr_EpauleAVG;
    Type_fr_HJambeAVG_X_fr_EpauleAVG fr_HJambeAVG_X_fr_EpauleAVG;
    Type_fr_EpauleAVG_X_fr_HJambeAVG fr_EpauleAVG_X_fr_HJambeAVG;
    Type_fr_BJambeAVG_X_fr_HJambeAVG fr_BJambeAVG_X_fr_HJambeAVG;
    Type_fr_HJambeAVG_X_fr_BJambeAVG fr_HJambeAVG_X_fr_BJambeAVG;
    Type_fr_EpauleARD_X_fr_base fr_EpauleARD_X_fr_base;
    Type_fr_base_X_fr_EpauleARD fr_base_X_fr_EpauleARD;
    Type_fr_HJambeARD_X_fr_EpauleARD fr_HJambeARD_X_fr_EpauleARD;
    Type_fr_EpauleARD_X_fr_HJambeARD fr_EpauleARD_X_fr_HJambeARD;
    Type_fr_BJambeARD_X_fr_HJambeARD fr_BJambeARD_X_fr_HJambeARD;
    Type_fr_HJambeARD_X_fr_BJambeARD fr_HJambeARD_X_fr_BJambeARD;
    Type_fr_EpauleARG_X_fr_base fr_EpauleARG_X_fr_base;
    Type_fr_base_X_fr_EpauleARG fr_base_X_fr_EpauleARG;
    Type_fr_HJambeARG_X_fr_EpauleARG fr_HJambeARG_X_fr_EpauleARG;
    Type_fr_EpauleARG_X_fr_HJambeARG fr_EpauleARG_X_fr_HJambeARG;
    Type_fr_BJambeARG_X_fr_HJambeARG fr_BJambeARG_X_fr_HJambeARG;
    Type_fr_HJambeARG_X_fr_BJambeARG fr_HJambeARG_X_fr_BJambeARG;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_EpauleAVD_X_fr_base : public TransformHomogeneous<Type_fr_EpauleAVD_X_fr_base>
    {
        Type_fr_EpauleAVD_X_fr_base();
        const Type_fr_EpauleAVD_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleAVD : public TransformHomogeneous<Type_fr_base_X_fr_EpauleAVD>
    {
        Type_fr_base_X_fr_EpauleAVD();
        const Type_fr_base_X_fr_EpauleAVD& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVD_X_fr_EpauleAVD : public TransformHomogeneous<Type_fr_HJambeAVD_X_fr_EpauleAVD>
    {
        Type_fr_HJambeAVD_X_fr_EpauleAVD();
        const Type_fr_HJambeAVD_X_fr_EpauleAVD& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVD_X_fr_HJambeAVD : public TransformHomogeneous<Type_fr_EpauleAVD_X_fr_HJambeAVD>
    {
        Type_fr_EpauleAVD_X_fr_HJambeAVD();
        const Type_fr_EpauleAVD_X_fr_HJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_BJambeAVD_X_fr_HJambeAVD : public TransformHomogeneous<Type_fr_BJambeAVD_X_fr_HJambeAVD>
    {
        Type_fr_BJambeAVD_X_fr_HJambeAVD();
        const Type_fr_BJambeAVD_X_fr_HJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVD_X_fr_BJambeAVD : public TransformHomogeneous<Type_fr_HJambeAVD_X_fr_BJambeAVD>
    {
        Type_fr_HJambeAVD_X_fr_BJambeAVD();
        const Type_fr_HJambeAVD_X_fr_BJambeAVD& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVG_X_fr_base : public TransformHomogeneous<Type_fr_EpauleAVG_X_fr_base>
    {
        Type_fr_EpauleAVG_X_fr_base();
        const Type_fr_EpauleAVG_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleAVG : public TransformHomogeneous<Type_fr_base_X_fr_EpauleAVG>
    {
        Type_fr_base_X_fr_EpauleAVG();
        const Type_fr_base_X_fr_EpauleAVG& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVG_X_fr_EpauleAVG : public TransformHomogeneous<Type_fr_HJambeAVG_X_fr_EpauleAVG>
    {
        Type_fr_HJambeAVG_X_fr_EpauleAVG();
        const Type_fr_HJambeAVG_X_fr_EpauleAVG& update(const state_t&);
    };
    
    struct Type_fr_EpauleAVG_X_fr_HJambeAVG : public TransformHomogeneous<Type_fr_EpauleAVG_X_fr_HJambeAVG>
    {
        Type_fr_EpauleAVG_X_fr_HJambeAVG();
        const Type_fr_EpauleAVG_X_fr_HJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_BJambeAVG_X_fr_HJambeAVG : public TransformHomogeneous<Type_fr_BJambeAVG_X_fr_HJambeAVG>
    {
        Type_fr_BJambeAVG_X_fr_HJambeAVG();
        const Type_fr_BJambeAVG_X_fr_HJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_HJambeAVG_X_fr_BJambeAVG : public TransformHomogeneous<Type_fr_HJambeAVG_X_fr_BJambeAVG>
    {
        Type_fr_HJambeAVG_X_fr_BJambeAVG();
        const Type_fr_HJambeAVG_X_fr_BJambeAVG& update(const state_t&);
    };
    
    struct Type_fr_EpauleARD_X_fr_base : public TransformHomogeneous<Type_fr_EpauleARD_X_fr_base>
    {
        Type_fr_EpauleARD_X_fr_base();
        const Type_fr_EpauleARD_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleARD : public TransformHomogeneous<Type_fr_base_X_fr_EpauleARD>
    {
        Type_fr_base_X_fr_EpauleARD();
        const Type_fr_base_X_fr_EpauleARD& update(const state_t&);
    };
    
    struct Type_fr_HJambeARD_X_fr_EpauleARD : public TransformHomogeneous<Type_fr_HJambeARD_X_fr_EpauleARD>
    {
        Type_fr_HJambeARD_X_fr_EpauleARD();
        const Type_fr_HJambeARD_X_fr_EpauleARD& update(const state_t&);
    };
    
    struct Type_fr_EpauleARD_X_fr_HJambeARD : public TransformHomogeneous<Type_fr_EpauleARD_X_fr_HJambeARD>
    {
        Type_fr_EpauleARD_X_fr_HJambeARD();
        const Type_fr_EpauleARD_X_fr_HJambeARD& update(const state_t&);
    };
    
    struct Type_fr_BJambeARD_X_fr_HJambeARD : public TransformHomogeneous<Type_fr_BJambeARD_X_fr_HJambeARD>
    {
        Type_fr_BJambeARD_X_fr_HJambeARD();
        const Type_fr_BJambeARD_X_fr_HJambeARD& update(const state_t&);
    };
    
    struct Type_fr_HJambeARD_X_fr_BJambeARD : public TransformHomogeneous<Type_fr_HJambeARD_X_fr_BJambeARD>
    {
        Type_fr_HJambeARD_X_fr_BJambeARD();
        const Type_fr_HJambeARD_X_fr_BJambeARD& update(const state_t&);
    };
    
    struct Type_fr_EpauleARG_X_fr_base : public TransformHomogeneous<Type_fr_EpauleARG_X_fr_base>
    {
        Type_fr_EpauleARG_X_fr_base();
        const Type_fr_EpauleARG_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_EpauleARG : public TransformHomogeneous<Type_fr_base_X_fr_EpauleARG>
    {
        Type_fr_base_X_fr_EpauleARG();
        const Type_fr_base_X_fr_EpauleARG& update(const state_t&);
    };
    
    struct Type_fr_HJambeARG_X_fr_EpauleARG : public TransformHomogeneous<Type_fr_HJambeARG_X_fr_EpauleARG>
    {
        Type_fr_HJambeARG_X_fr_EpauleARG();
        const Type_fr_HJambeARG_X_fr_EpauleARG& update(const state_t&);
    };
    
    struct Type_fr_EpauleARG_X_fr_HJambeARG : public TransformHomogeneous<Type_fr_EpauleARG_X_fr_HJambeARG>
    {
        Type_fr_EpauleARG_X_fr_HJambeARG();
        const Type_fr_EpauleARG_X_fr_HJambeARG& update(const state_t&);
    };
    
    struct Type_fr_BJambeARG_X_fr_HJambeARG : public TransformHomogeneous<Type_fr_BJambeARG_X_fr_HJambeARG>
    {
        Type_fr_BJambeARG_X_fr_HJambeARG();
        const Type_fr_BJambeARG_X_fr_HJambeARG& update(const state_t&);
    };
    
    struct Type_fr_HJambeARG_X_fr_BJambeARG : public TransformHomogeneous<Type_fr_HJambeARG_X_fr_BJambeARG>
    {
        Type_fr_HJambeARG_X_fr_BJambeARG();
        const Type_fr_HJambeARG_X_fr_BJambeARG& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_EpauleAVD_X_fr_base fr_EpauleAVD_X_fr_base;
    Type_fr_base_X_fr_EpauleAVD fr_base_X_fr_EpauleAVD;
    Type_fr_HJambeAVD_X_fr_EpauleAVD fr_HJambeAVD_X_fr_EpauleAVD;
    Type_fr_EpauleAVD_X_fr_HJambeAVD fr_EpauleAVD_X_fr_HJambeAVD;
    Type_fr_BJambeAVD_X_fr_HJambeAVD fr_BJambeAVD_X_fr_HJambeAVD;
    Type_fr_HJambeAVD_X_fr_BJambeAVD fr_HJambeAVD_X_fr_BJambeAVD;
    Type_fr_EpauleAVG_X_fr_base fr_EpauleAVG_X_fr_base;
    Type_fr_base_X_fr_EpauleAVG fr_base_X_fr_EpauleAVG;
    Type_fr_HJambeAVG_X_fr_EpauleAVG fr_HJambeAVG_X_fr_EpauleAVG;
    Type_fr_EpauleAVG_X_fr_HJambeAVG fr_EpauleAVG_X_fr_HJambeAVG;
    Type_fr_BJambeAVG_X_fr_HJambeAVG fr_BJambeAVG_X_fr_HJambeAVG;
    Type_fr_HJambeAVG_X_fr_BJambeAVG fr_HJambeAVG_X_fr_BJambeAVG;
    Type_fr_EpauleARD_X_fr_base fr_EpauleARD_X_fr_base;
    Type_fr_base_X_fr_EpauleARD fr_base_X_fr_EpauleARD;
    Type_fr_HJambeARD_X_fr_EpauleARD fr_HJambeARD_X_fr_EpauleARD;
    Type_fr_EpauleARD_X_fr_HJambeARD fr_EpauleARD_X_fr_HJambeARD;
    Type_fr_BJambeARD_X_fr_HJambeARD fr_BJambeARD_X_fr_HJambeARD;
    Type_fr_HJambeARD_X_fr_BJambeARD fr_HJambeARD_X_fr_BJambeARD;
    Type_fr_EpauleARG_X_fr_base fr_EpauleARG_X_fr_base;
    Type_fr_base_X_fr_EpauleARG fr_base_X_fr_EpauleARG;
    Type_fr_HJambeARG_X_fr_EpauleARG fr_HJambeARG_X_fr_EpauleARG;
    Type_fr_EpauleARG_X_fr_HJambeARG fr_EpauleARG_X_fr_HJambeARG;
    Type_fr_BJambeARG_X_fr_HJambeARG fr_BJambeARG_X_fr_HJambeARG;
    Type_fr_HJambeARG_X_fr_BJambeARG fr_HJambeARG_X_fr_BJambeARG;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
