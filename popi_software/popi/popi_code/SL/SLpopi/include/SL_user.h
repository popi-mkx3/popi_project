#ifndef _SL_USER_POPI_H_
#define _SL_USER_POPI_H_

#include "SL.h"

/*! the robot name */
#define ROBOT_NAME "popi"

/*! links of the robot */
enum RobotLinks {
    PLACEHOLDER = 0,
    EPAULEAVD,
    HJAMBEAVD,
    BJAMBEAVD,
    EPAULEAVG,
    HJAMBEAVG,
    BJAMBEAVG,
    EPAULEARD,
    HJAMBEARD,
    BJAMBEARD,
    EPAULEARG,
    HJAMBEARG,
    BJAMBEARG,
    DUMMY_EE,
    N_ROBOT_LINKS
};

/*! endeffector information */
enum RobotEndeffectors {
  ENDEFF=1,
  N_ROBOT_ENDEFFECTORS
};

/*! vision variables */
enum VisionCameras {
  N_VISION_CAMERAS
};

enum ColorBlobs {
  BLOB1=1,
  N_COLOR_BLOBS
};

/*! define the DOFs of this robot */
enum RobotDOFs {
    BASE=0,
    RF_HAA_JOINT,
    RF_HFE_JOINT,
    RF_KFE_JOINT,
    LF_HAA_JOINT,
    LF_HFE_JOINT,
    LF_KFE_JOINT,
    RH_HAA_JOINT,
    RH_HFE_JOINT,
    RH_KFE_JOINT,
    LH_HAA_JOINT,
    LH_HFE_JOINT,
    LH_KFE_JOINT,
    N_ROBOT_DOFS
};

/*! define miscellenous sensors of this robot */
enum RobotMiscSensors {
    B_Q_0=1,
    B_Q_1,
    B_Q_2,
    B_Q_3,
    B_Qd_0,
    B_Qd_1,
    B_Qd_2,
    B_Qd_3,
    B_Qdd_0,
    B_Qdd_1,
    B_Qdd_2,
    B_Qdd_3,
    B_Ad_A,
    B_Ad_B,
    B_Ad_G,
    B_Add_A,
    B_Add_B,
    B_Add_G,
    B_X,
    B_Y,
    B_Z,
    B_Xd,
    B_Yd,
    B_Zd,
    B_Xdd,
    B_Ydd,
    B_Zdd,
    TIME_MOTOR,
    N_ROBOT_MISC_SENSORS
};

/*! number of degrees-of-freedom of robot */
#define N_DOFS (N_ROBOT_DOFS-1)

/*! N_DOFS + fake DOFS, needed for parameter estimation;
   fake DOFS come from creating endeffector information */
#define N_DOFS_EST (N_DOFS+31)

/*! N_DOFS to be excluded from parameter estimation (e.g., eye joints);
   these DOFS must be the last DOFS in the arrays */
#define N_DOFS_EST_SKIP 0

/*! number of links of the robot */
#define N_LINKS    (N_ROBOT_LINKS-1)

/*! number of miscellaneous sensors */
#define N_MISC_SENSORS   (N_ROBOT_MISC_SENSORS-1)

/*! number of endeffectors */
#define N_ENDEFFS  (N_ROBOT_ENDEFFECTORS-1)

/*! number of cameras used */
#define N_CAMERAS (N_VISION_CAMERAS-1)

/*! number of blobs that can be tracked by vision system */
#define MAX_BLOBS (N_COLOR_BLOBS-1)

/*! vision default post processing */
#define VISION_DEFAULT_PP "vision_default.pp"

/*! the servo rate used by the I/O with robot: this limits the
   servo rates of all other servos */
#define  SERVO_BASE_RATE 1000

/*! divisor to obtain task servo rate (task servo can run slower than
   base rate, but only in integer fractions */
#define  TASK_SERVO_RATIO   R1TO4
//! #define  TASK_SERVO_RATIO   R1TO1

/* settings for D/A debugging information -- see SL_oscilloscope.c */
#define   D2A_CM      1
#define   D2A_CT      2
#define   D2A_CV      3
#define   D2A_CR      4
#define   D2A_CS      5

#ifdef __cplusplus
extern "C" {
#endif

extern int    real_time_flag;
extern double force_biases[N_ENDEFFS+1][N_CART+1];

#ifdef __cplusplus
}
#endif
#endif  /* _SL_USER_POPI_H_ */
