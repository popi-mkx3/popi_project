#ifndef IIT_POPI_JOINT_DATA_MAP_H_
#define IIT_POPI_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace popi {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[RF_HAA_JOINT] = rhs[RF_HAA_JOINT];
    data[RF_HFE_JOINT] = rhs[RF_HFE_JOINT];
    data[RF_KFE_JOINT] = rhs[RF_KFE_JOINT];
    data[LF_HAA_JOINT] = rhs[LF_HAA_JOINT];
    data[LF_HFE_JOINT] = rhs[LF_HFE_JOINT];
    data[LF_KFE_JOINT] = rhs[LF_KFE_JOINT];
    data[RH_HAA_JOINT] = rhs[RH_HAA_JOINT];
    data[RH_HFE_JOINT] = rhs[RH_HFE_JOINT];
    data[RH_KFE_JOINT] = rhs[RH_KFE_JOINT];
    data[LH_HAA_JOINT] = rhs[LH_HAA_JOINT];
    data[LH_HFE_JOINT] = rhs[LH_HFE_JOINT];
    data[LH_KFE_JOINT] = rhs[LH_KFE_JOINT];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[RF_HAA_JOINT] = value;
    data[RF_HFE_JOINT] = value;
    data[RF_KFE_JOINT] = value;
    data[LF_HAA_JOINT] = value;
    data[LF_HFE_JOINT] = value;
    data[LF_KFE_JOINT] = value;
    data[RH_HAA_JOINT] = value;
    data[RH_HFE_JOINT] = value;
    data[RH_KFE_JOINT] = value;
    data[LH_HAA_JOINT] = value;
    data[LH_HFE_JOINT] = value;
    data[LH_KFE_JOINT] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   rf_haa_joint = "
    << map[RF_HAA_JOINT]
    << "   rf_hfe_joint = "
    << map[RF_HFE_JOINT]
    << "   rf_kfe_joint = "
    << map[RF_KFE_JOINT]
    << "   lf_haa_joint = "
    << map[LF_HAA_JOINT]
    << "   lf_hfe_joint = "
    << map[LF_HFE_JOINT]
    << "   lf_kfe_joint = "
    << map[LF_KFE_JOINT]
    << "   rh_haa_joint = "
    << map[RH_HAA_JOINT]
    << "   rh_hfe_joint = "
    << map[RH_HFE_JOINT]
    << "   rh_kfe_joint = "
    << map[RH_KFE_JOINT]
    << "   lh_haa_joint = "
    << map[LH_HAA_JOINT]
    << "   lh_hfe_joint = "
    << map[LH_HFE_JOINT]
    << "   lh_kfe_joint = "
    << map[LH_KFE_JOINT]
    ;
    return out;
}

}
}
#endif
