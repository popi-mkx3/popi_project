#ifndef IIT_ROBOT_POPI_MODEL_CONSTANTS_H_
#define IIT_ROBOT_POPI_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace iit {
namespace popi {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_rf_haa_joint = 0.3305000066757202;
const Scalar ty_rf_haa_joint = -0.17499999701976776;
const Scalar tz_rf_haa_joint = -0.050999999046325684;
const Scalar ty_rf_hfe_joint = -0.10769999772310257;
const Scalar tx_rf_kfe_joint = -0.3519200086593628;
const Scalar ty_rf_kfe_joint = -0.041508998721838;
const Scalar tx_lf_haa_joint = 0.3305000066757202;
const Scalar ty_lf_haa_joint = 0.17499999701976776;
const Scalar tz_lf_haa_joint = -0.050999999046325684;
const Scalar ty_lf_hfe_joint = 0.10769999772310257;
const Scalar tx_lf_kfe_joint = -0.3519200086593628;
const Scalar ty_lf_kfe_joint = -0.041508998721838;
const Scalar tx_rh_haa_joint = -0.3305000066757202;
const Scalar ty_rh_haa_joint = -0.17499999701976776;
const Scalar tz_rh_haa_joint = -0.050999999046325684;
const Scalar ty_rh_hfe_joint = -0.10769999772310257;
const Scalar tx_rh_kfe_joint = -0.3519200086593628;
const Scalar ty_rh_kfe_joint = -0.041508998721838;
const Scalar tx_lh_haa_joint = -0.3305000066757202;
const Scalar ty_lh_haa_joint = 0.17499999701976776;
const Scalar tz_lh_haa_joint = -0.050999999046325684;
const Scalar ty_lh_hfe_joint = 0.10769999772310257;
const Scalar tx_lh_kfe_joint = -0.3519200086593628;
const Scalar ty_lh_kfe_joint = -0.041508998721838;
const Scalar tz_fr_base_COM = -0.03522999957203865;
const Scalar tx_fr_EpauleAVD_COM = -0.003539999946951866;
const Scalar ty_fr_EpauleAVD_COM = 0.043549999594688416;
const Scalar tz_fr_EpauleAVD_COM = 0.0013699999544769526;
const Scalar tx_fr_HJambeAVD_COM = -0.11331000179052353;
const Scalar ty_fr_HJambeAVD_COM = 0.017430000007152557;
const Scalar tz_fr_HJambeAVD_COM = -0.005119999870657921;
const Scalar tx_fr_BJambeAVD_COM = 0.11761912703514099;
const Scalar ty_fr_BJambeAVD_COM = 0.012387133203446865;
const Scalar tz_fr_BJambeAVD_COM = 5.79999983045705E-14;
const Scalar tx_fr_PiedAVD = 0.29603999853134155;
const Scalar ty_fr_PiedAVD = 0.014999999664723873;
const Scalar tx_fr_EpauleAVG_COM = 0.003539999946951866;
const Scalar ty_fr_EpauleAVG_COM = -0.043549999594688416;
const Scalar tz_fr_EpauleAVG_COM = -0.0013699999544769526;
const Scalar tx_fr_HJambeAVG_COM = -0.11388999968767166;
const Scalar ty_fr_HJambeAVG_COM = 0.00901000015437603;
const Scalar tz_fr_HJambeAVG_COM = 0.005549999885261059;
const Scalar tx_fr_BJambeAVG_COM = 0.11761912703514099;
const Scalar ty_fr_BJambeAVG_COM = 0.012387133203446865;
const Scalar tz_fr_BJambeAVG_COM = -1.7890899872696764E-10;
const Scalar tx_fr_PiedAVG = 0.29603999853134155;
const Scalar ty_fr_PiedAVG = 0.014999999664723873;
const Scalar tx_fr_EpauleARD_COM = -0.003539999946951866;
const Scalar ty_fr_EpauleARD_COM = 0.043549999594688416;
const Scalar tz_fr_EpauleARD_COM = -0.0013699999544769526;
const Scalar tx_fr_HJambeARD_COM = -0.11331000179052353;
const Scalar ty_fr_HJambeARD_COM = 0.017430000007152557;
const Scalar tz_fr_HJambeARD_COM = -0.005119999870657921;
const Scalar tx_fr_BJambeARD_COM = 0.11761912703514099;
const Scalar ty_fr_BJambeARD_COM = 0.012387133203446865;
const Scalar tx_fr_PiedARD = 0.29603999853134155;
const Scalar ty_fr_PiedARD = 0.014999999664723873;
const Scalar tx_fr_EpauleARG_COM = 0.003539999946951866;
const Scalar ty_fr_EpauleARG_COM = -0.043549999594688416;
const Scalar tz_fr_EpauleARG_COM = 0.0013699999544769526;
const Scalar tx_fr_HJambeARG_COM = -0.11388999968767166;
const Scalar ty_fr_HJambeARG_COM = 0.00901000015437603;
const Scalar tz_fr_HJambeARG_COM = 0.005549999885261059;
const Scalar tx_fr_BJambeARG_COM = 0.11761912703514099;
const Scalar ty_fr_BJambeARG_COM = 0.012387133203446865;
const Scalar tz_fr_BJambeARG_COM = -1.7893500570131948E-10;
const Scalar tx_fr_PiedARG = 0.29603999853134155;
const Scalar ty_fr_PiedARG = 0.014999999664723873;
const Scalar m_base = 22.93400001525879;
const Scalar ix_base = 0.38572999835014343;
const Scalar iy_base = 0.9069200158119202;
const Scalar iz_base = 1.2276999950408936;
const Scalar m_EpauleAVD = 6.775000095367432;
const Scalar ix_EpauleAVD = 0.028599999845027924;
const Scalar ixy_EpauleAVD = -5.999999848427251E-5;
const Scalar ixz_EpauleAVD = 1.8000000272877514E-4;
const Scalar iy_EpauleAVD = 0.01066999975591898;
const Scalar iyz_EpauleAVD = 7.50000006519258E-4;
const Scalar iz_EpauleAVD = 0.028929999098181725;
const Scalar m_HJambeAVD = 4.894999980926514;
const Scalar ix_HJambeAVD = 0.008990000002086163;
const Scalar ixy_HJambeAVD = -0.00215000007301569;
const Scalar ixz_HJambeAVD = -0.00646999990567565;
const Scalar iy_HJambeAVD = 0.060499999672174454;
const Scalar iyz_HJambeAVD = 4.299999854993075E-4;
const Scalar iz_HJambeAVD = 0.057029999792575836;
const Scalar m_BJambeAVD = 0.49002501368522644;
const Scalar ix_BJambeAVD = 1.0300027497578412E-4;
const Scalar ixy_BJambeAVD = 4.99999991225835E-15;
const Scalar ixz_BJambeAVD = 1.899883063742891E-4;
const Scalar iy_BJambeAVD = 0.0067988005466759205;
const Scalar iz_BJambeAVD = 0.006776800379157066;
const Scalar m_EpauleAVG = 6.775000095367432;
const Scalar ix_EpauleAVG = 0.02864000014960766;
const Scalar ixy_EpauleAVG = 5.999999848427251E-5;
const Scalar ixz_EpauleAVG = 1.8000000272877514E-4;
const Scalar iy_EpauleAVG = 0.01066999975591898;
const Scalar iyz_EpauleAVG = -7.50000006519258E-4;
const Scalar iz_EpauleAVG = 0.028929999098181725;
const Scalar m_HJambeAVG = 4.954999923706055;
const Scalar ix_HJambeAVG = 0.010320000350475311;
const Scalar ixy_HJambeAVG = 0.002259999979287386;
const Scalar ixz_HJambeAVG = -0.010110000148415565;
const Scalar iy_HJambeAVG = 0.06055999919772148;
const Scalar iyz_HJambeAVG = -2.500000118743628E-4;
const Scalar iz_HJambeAVG = 0.055959999561309814;
const Scalar m_BJambeAVG = 0.49002501368522644;
const Scalar ix_BJambeAVG = 1.0300027497578412E-4;
const Scalar ixy_BJambeAVG = -1.568599937440407E-11;
const Scalar ixz_BJambeAVG = 1.899883063742891E-4;
const Scalar iy_BJambeAVG = 0.0067988005466759205;
const Scalar iyz_BJambeAVG = 2.2900000580701463E-13;
const Scalar iz_BJambeAVG = 0.006776800379157066;
const Scalar m_EpauleARD = 6.775000095367432;
const Scalar ix_EpauleARD = 0.02864000014960766;
const Scalar ixy_EpauleARD = 5.999999848427251E-5;
const Scalar ixz_EpauleARD = -1.8000000636675395E-5;
const Scalar iy_EpauleARD = 0.01066999975591898;
const Scalar iyz_EpauleARD = 7.50000006519258E-4;
const Scalar iz_EpauleARD = 0.028929999098181725;
const Scalar m_HJambeARD = 4.894999980926514;
const Scalar ix_HJambeARD = 0.008990000002086163;
const Scalar ixy_HJambeARD = -0.00215000007301569;
const Scalar ixz_HJambeARD = -0.00646999990567565;
const Scalar iy_HJambeARD = 0.060499999672174454;
const Scalar iyz_HJambeARD = 4.299999854993075E-4;
const Scalar iz_HJambeARD = 0.057029999792575836;
const Scalar m_BJambeARD = 0.49002501368522644;
const Scalar ix_BJambeARD = 1.0300027497578412E-4;
const Scalar ixz_BJambeARD = 1.899883063742891E-4;
const Scalar iy_BJambeARD = 0.0067988005466759205;
const Scalar iz_BJambeARD = 0.006776800379157066;
const Scalar m_EpauleARG = 6.775000095367432;
const Scalar ix_EpauleARG = 0.02864000014960766;
const Scalar ixy_EpauleARG = -5.999999848427251E-5;
const Scalar ixz_EpauleARG = -1.8000000272877514E-4;
const Scalar iy_EpauleARG = 0.01066999975591898;
const Scalar iyz_EpauleARG = -7.50000006519258E-4;
const Scalar iz_EpauleARG = 0.028929999098181725;
const Scalar m_HJambeARG = 4.954999923706055;
const Scalar ix_HJambeARG = 0.010320000350475311;
const Scalar ixy_HJambeARG = 0.002259999979287386;
const Scalar ixz_HJambeARG = -0.010110000148415565;
const Scalar iy_HJambeARG = 0.06055999919772148;
const Scalar iyz_HJambeARG = -2.500000118743628E-4;
const Scalar iz_HJambeARG = 0.055959999561309814;
const Scalar m_BJambeARG = 0.49002501368522644;
const Scalar ix_BJambeARG = 1.0300027497578412E-4;
const Scalar ixy_BJambeARG = -1.568799951057187E-11;
const Scalar ixz_BJambeARG = 1.899883063742891E-4;
const Scalar iy_BJambeARG = 0.0067988005466759205;
const Scalar iyz_BJambeARG = 2.2900000580701463E-13;
const Scalar iz_BJambeARG = 0.006776800379157066;

}
}
#endif
