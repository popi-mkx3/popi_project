#ifndef IIT_POPI_LINK_DATA_MAP_H_
#define IIT_POPI_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace popi {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE] = rhs[BASE];
    data[EPAULEAVD] = rhs[EPAULEAVD];
    data[HJAMBEAVD] = rhs[HJAMBEAVD];
    data[BJAMBEAVD] = rhs[BJAMBEAVD];
    data[EPAULEAVG] = rhs[EPAULEAVG];
    data[HJAMBEAVG] = rhs[HJAMBEAVG];
    data[BJAMBEAVG] = rhs[BJAMBEAVG];
    data[EPAULEARD] = rhs[EPAULEARD];
    data[HJAMBEARD] = rhs[HJAMBEARD];
    data[BJAMBEARD] = rhs[BJAMBEARD];
    data[EPAULEARG] = rhs[EPAULEARG];
    data[HJAMBEARG] = rhs[HJAMBEARG];
    data[BJAMBEARG] = rhs[BJAMBEARG];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE] = value;
    data[EPAULEAVD] = value;
    data[HJAMBEAVD] = value;
    data[BJAMBEAVD] = value;
    data[EPAULEAVG] = value;
    data[HJAMBEAVG] = value;
    data[BJAMBEAVG] = value;
    data[EPAULEARD] = value;
    data[HJAMBEARD] = value;
    data[BJAMBEARD] = value;
    data[EPAULEARG] = value;
    data[HJAMBEARG] = value;
    data[BJAMBEARG] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base = "
    << map[BASE]
    << "   EpauleAVD = "
    << map[EPAULEAVD]
    << "   HJambeAVD = "
    << map[HJAMBEAVD]
    << "   BJambeAVD = "
    << map[BJAMBEAVD]
    << "   EpauleAVG = "
    << map[EPAULEAVG]
    << "   HJambeAVG = "
    << map[HJAMBEAVG]
    << "   BJambeAVG = "
    << map[BJAMBEAVG]
    << "   EpauleARD = "
    << map[EPAULEARD]
    << "   HJambeARD = "
    << map[HJAMBEARD]
    << "   BJambeARD = "
    << map[BJAMBEARD]
    << "   EpauleARG = "
    << map[EPAULEARG]
    << "   HJambeARG = "
    << map[HJAMBEARG]
    << "   BJambeARG = "
    << map[BJAMBEARG]
    ;
    return out;
}

}
}
#endif
