#ifndef IIT_ROBOT_POPI_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_POPI_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace iit {
namespace popi {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot popi.
 */
namespace dyn {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_base() const;
        const InertiaMatrix& getTensor_EpauleAVD() const;
        const InertiaMatrix& getTensor_HJambeAVD() const;
        const InertiaMatrix& getTensor_BJambeAVD() const;
        const InertiaMatrix& getTensor_EpauleAVG() const;
        const InertiaMatrix& getTensor_HJambeAVG() const;
        const InertiaMatrix& getTensor_BJambeAVG() const;
        const InertiaMatrix& getTensor_EpauleARD() const;
        const InertiaMatrix& getTensor_HJambeARD() const;
        const InertiaMatrix& getTensor_BJambeARD() const;
        const InertiaMatrix& getTensor_EpauleARG() const;
        const InertiaMatrix& getTensor_HJambeARG() const;
        const InertiaMatrix& getTensor_BJambeARG() const;
        Scalar getMass_base() const;
        Scalar getMass_EpauleAVD() const;
        Scalar getMass_HJambeAVD() const;
        Scalar getMass_BJambeAVD() const;
        Scalar getMass_EpauleAVG() const;
        Scalar getMass_HJambeAVG() const;
        Scalar getMass_BJambeAVG() const;
        Scalar getMass_EpauleARD() const;
        Scalar getMass_HJambeARD() const;
        Scalar getMass_BJambeARD() const;
        Scalar getMass_EpauleARG() const;
        Scalar getMass_HJambeARG() const;
        Scalar getMass_BJambeARG() const;
        const Vector3& getCOM_base() const;
        const Vector3& getCOM_EpauleAVD() const;
        const Vector3& getCOM_HJambeAVD() const;
        const Vector3& getCOM_BJambeAVD() const;
        const Vector3& getCOM_EpauleAVG() const;
        const Vector3& getCOM_HJambeAVG() const;
        const Vector3& getCOM_BJambeAVG() const;
        const Vector3& getCOM_EpauleARD() const;
        const Vector3& getCOM_HJambeARD() const;
        const Vector3& getCOM_BJambeARD() const;
        const Vector3& getCOM_EpauleARG() const;
        const Vector3& getCOM_HJambeARG() const;
        const Vector3& getCOM_BJambeARG() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot popi,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_base;
        InertiaMatrix tensor_EpauleAVD;
        InertiaMatrix tensor_HJambeAVD;
        InertiaMatrix tensor_BJambeAVD;
        InertiaMatrix tensor_EpauleAVG;
        InertiaMatrix tensor_HJambeAVG;
        InertiaMatrix tensor_BJambeAVG;
        InertiaMatrix tensor_EpauleARD;
        InertiaMatrix tensor_HJambeARD;
        InertiaMatrix tensor_BJambeARD;
        InertiaMatrix tensor_EpauleARG;
        InertiaMatrix tensor_HJambeARG;
        InertiaMatrix tensor_BJambeARG;
        Vector3 com_base;
        Vector3 com_EpauleAVD;
        Vector3 com_HJambeAVD;
        Vector3 com_BJambeAVD;
        Vector3 com_EpauleAVG;
        Vector3 com_HJambeAVG;
        Vector3 com_BJambeAVG;
        Vector3 com_EpauleARD;
        Vector3 com_HJambeARD;
        Vector3 com_BJambeARD;
        Vector3 com_EpauleARG;
        Vector3 com_HJambeARG;
        Vector3 com_BJambeARG;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_base() const {
    return this->tensor_base;
}
inline const InertiaMatrix& InertiaProperties::getTensor_EpauleAVD() const {
    return this->tensor_EpauleAVD;
}
inline const InertiaMatrix& InertiaProperties::getTensor_HJambeAVD() const {
    return this->tensor_HJambeAVD;
}
inline const InertiaMatrix& InertiaProperties::getTensor_BJambeAVD() const {
    return this->tensor_BJambeAVD;
}
inline const InertiaMatrix& InertiaProperties::getTensor_EpauleAVG() const {
    return this->tensor_EpauleAVG;
}
inline const InertiaMatrix& InertiaProperties::getTensor_HJambeAVG() const {
    return this->tensor_HJambeAVG;
}
inline const InertiaMatrix& InertiaProperties::getTensor_BJambeAVG() const {
    return this->tensor_BJambeAVG;
}
inline const InertiaMatrix& InertiaProperties::getTensor_EpauleARD() const {
    return this->tensor_EpauleARD;
}
inline const InertiaMatrix& InertiaProperties::getTensor_HJambeARD() const {
    return this->tensor_HJambeARD;
}
inline const InertiaMatrix& InertiaProperties::getTensor_BJambeARD() const {
    return this->tensor_BJambeARD;
}
inline const InertiaMatrix& InertiaProperties::getTensor_EpauleARG() const {
    return this->tensor_EpauleARG;
}
inline const InertiaMatrix& InertiaProperties::getTensor_HJambeARG() const {
    return this->tensor_HJambeARG;
}
inline const InertiaMatrix& InertiaProperties::getTensor_BJambeARG() const {
    return this->tensor_BJambeARG;
}
inline Scalar InertiaProperties::getMass_base() const {
    return this->tensor_base.getMass();
}
inline Scalar InertiaProperties::getMass_EpauleAVD() const {
    return this->tensor_EpauleAVD.getMass();
}
inline Scalar InertiaProperties::getMass_HJambeAVD() const {
    return this->tensor_HJambeAVD.getMass();
}
inline Scalar InertiaProperties::getMass_BJambeAVD() const {
    return this->tensor_BJambeAVD.getMass();
}
inline Scalar InertiaProperties::getMass_EpauleAVG() const {
    return this->tensor_EpauleAVG.getMass();
}
inline Scalar InertiaProperties::getMass_HJambeAVG() const {
    return this->tensor_HJambeAVG.getMass();
}
inline Scalar InertiaProperties::getMass_BJambeAVG() const {
    return this->tensor_BJambeAVG.getMass();
}
inline Scalar InertiaProperties::getMass_EpauleARD() const {
    return this->tensor_EpauleARD.getMass();
}
inline Scalar InertiaProperties::getMass_HJambeARD() const {
    return this->tensor_HJambeARD.getMass();
}
inline Scalar InertiaProperties::getMass_BJambeARD() const {
    return this->tensor_BJambeARD.getMass();
}
inline Scalar InertiaProperties::getMass_EpauleARG() const {
    return this->tensor_EpauleARG.getMass();
}
inline Scalar InertiaProperties::getMass_HJambeARG() const {
    return this->tensor_HJambeARG.getMass();
}
inline Scalar InertiaProperties::getMass_BJambeARG() const {
    return this->tensor_BJambeARG.getMass();
}
inline const Vector3& InertiaProperties::getCOM_base() const {
    return this->com_base;
}
inline const Vector3& InertiaProperties::getCOM_EpauleAVD() const {
    return this->com_EpauleAVD;
}
inline const Vector3& InertiaProperties::getCOM_HJambeAVD() const {
    return this->com_HJambeAVD;
}
inline const Vector3& InertiaProperties::getCOM_BJambeAVD() const {
    return this->com_BJambeAVD;
}
inline const Vector3& InertiaProperties::getCOM_EpauleAVG() const {
    return this->com_EpauleAVG;
}
inline const Vector3& InertiaProperties::getCOM_HJambeAVG() const {
    return this->com_HJambeAVG;
}
inline const Vector3& InertiaProperties::getCOM_BJambeAVG() const {
    return this->com_BJambeAVG;
}
inline const Vector3& InertiaProperties::getCOM_EpauleARD() const {
    return this->com_EpauleARD;
}
inline const Vector3& InertiaProperties::getCOM_HJambeARD() const {
    return this->com_HJambeARD;
}
inline const Vector3& InertiaProperties::getCOM_BJambeARD() const {
    return this->com_BJambeARD;
}
inline const Vector3& InertiaProperties::getCOM_EpauleARG() const {
    return this->com_EpauleARG;
}
inline const Vector3& InertiaProperties::getCOM_HJambeARG() const {
    return this->com_HJambeARG;
}
inline const Vector3& InertiaProperties::getCOM_BJambeARG() const {
    return this->com_BJambeARG;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_base + m_EpauleAVD + m_HJambeAVD + m_BJambeAVD + m_EpauleAVG + m_HJambeAVG + m_BJambeAVG + m_EpauleARD + m_HJambeARD + m_BJambeARD + m_EpauleARG + m_HJambeARG + m_BJambeARG;
}

}
}
}

#endif
