#ifndef IIT_ROBOT_POPI_DECLARATIONS_H_
#define IIT_ROBOT_POPI_DECLARATIONS_H_

#include "rbd_types.h"

namespace iit {
namespace popi {

static constexpr int JointSpaceDimension = 12;
static constexpr int jointsCount = 12;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 13;

typedef Matrix<12, 1> Column12d;
typedef Column12d JointState;

enum JointIdentifiers {
    RF_HAA_JOINT = 0
    , RF_HFE_JOINT
    , RF_KFE_JOINT
    , LF_HAA_JOINT
    , LF_HFE_JOINT
    , LF_KFE_JOINT
    , RH_HAA_JOINT
    , RH_HFE_JOINT
    , RH_KFE_JOINT
    , LH_HAA_JOINT
    , LH_HFE_JOINT
    , LH_KFE_JOINT
};

enum LinkIdentifiers {
    BASE = 0
    , EPAULEAVD
    , HJAMBEAVD
    , BJAMBEAVD
    , EPAULEAVG
    , HJAMBEAVG
    , BJAMBEAVG
    , EPAULEARD
    , HJAMBEARD
    , BJAMBEARD
    , EPAULEARG
    , HJAMBEARG
    , BJAMBEARG
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {RF_HAA_JOINT,RF_HFE_JOINT,RF_KFE_JOINT,LF_HAA_JOINT,LF_HFE_JOINT,LF_KFE_JOINT,RH_HAA_JOINT,RH_HFE_JOINT,RH_KFE_JOINT,LH_HAA_JOINT,LH_HFE_JOINT,LH_KFE_JOINT};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,EPAULEAVD,HJAMBEAVD,BJAMBEAVD,EPAULEAVG,HJAMBEAVG,BJAMBEAVG,EPAULEARD,HJAMBEARD,BJAMBEARD,EPAULEARG,HJAMBEARG,BJAMBEARG};

}
}
#endif
