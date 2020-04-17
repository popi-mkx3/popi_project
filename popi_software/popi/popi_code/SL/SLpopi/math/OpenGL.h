#define RAD2DEG (57.3)

static double x,y,z; // support vars

// The state of the base
glPushMatrix();
glTranslated((GLdouble)basec[0].x[1],(GLdouble)basec[0].x[2],(GLdouble)basec[0].x[3]);
glRotated((GLdouble)114.5916*ArcCos(baseo[0].q[1]),(GLdouble)baseo[0].q[2],(GLdouble)baseo[0].q[3],(GLdouble)baseo[0].q[4]);

// Joint rf_haa_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( tz_rf_haa_joint/(std::sqrt(tx_rf_haa_joint*tx_rf_haa_joint + ty_rf_haa_joint*ty_rf_haa_joint + tz_rf_haa_joint*tz_rf_haa_joint)) )), (GLdouble)-(ty_rf_haa_joint), (GLdouble)tx_rf_haa_joint, (GLdouble)0.0);
myDrawGLElement(::RF_HAA_JOINT, 0.37743377335937367, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_rf_haa_joint, (GLdouble)ty_rf_haa_joint, (GLdouble)tz_rf_haa_joint);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*-1.5707963705062866), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::RF_HAA_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint rf_hfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( 0.0/(std::sqrt(0.0*0.0 + ty_rf_hfe_joint*ty_rf_hfe_joint + 0.0*0.0)) )), (GLdouble)-(ty_rf_hfe_joint), (GLdouble)0.0, (GLdouble)0.0);
myDrawGLElement(::RF_HFE_JOINT, 0.10769999772310257, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.0, (GLdouble)ty_rf_hfe_joint, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*-1.5707963705062866), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::RF_HFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint rf_kfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( -0.0/(std::sqrt(tx_rf_kfe_joint*tx_rf_kfe_joint + ty_rf_kfe_joint*ty_rf_kfe_joint + -0.0*-0.0)) )), (GLdouble)-(ty_rf_kfe_joint), (GLdouble)tx_rf_kfe_joint, (GLdouble)0.0);
myDrawGLElement(::RF_KFE_JOINT, 0.3543595482976232, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_rf_kfe_joint, (GLdouble)ty_rf_kfe_joint, (GLdouble)-0.0);
glRotated((GLdouble)(RAD2DEG*-3.1415927410125732), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::RF_KFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Draw the end effector
glPushMatrix();
x = eff[1].x[_X_];
y = eff[1].x[_Y_];
z = eff[1].x[_Z_];
glRotated(
        (GLdouble)RAD2DEG*acos(z),(GLdouble)-y,(GLdouble)x,(GLdouble)0);
myDrawGLElement(101, (double)Sqrt(x*x + y*y + z*z), 0);
glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();
// Joint lf_haa_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( tz_lf_haa_joint/(std::sqrt(tx_lf_haa_joint*tx_lf_haa_joint + ty_lf_haa_joint*ty_lf_haa_joint + tz_lf_haa_joint*tz_lf_haa_joint)) )), (GLdouble)-(ty_lf_haa_joint), (GLdouble)tx_lf_haa_joint, (GLdouble)0.0);
myDrawGLElement(::LF_HAA_JOINT, 0.37743377335937367, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_lf_haa_joint, (GLdouble)ty_lf_haa_joint, (GLdouble)tz_lf_haa_joint);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::LF_HAA_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint lf_hfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( 0.0/(std::sqrt(0.0*0.0 + ty_lf_hfe_joint*ty_lf_hfe_joint + 0.0*0.0)) )), (GLdouble)-(ty_lf_hfe_joint), (GLdouble)0.0, (GLdouble)0.0);
myDrawGLElement(::LF_HFE_JOINT, 0.10769999772310257, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.0, (GLdouble)ty_lf_hfe_joint, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::LF_HFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint lf_kfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( -0.0/(std::sqrt(tx_lf_kfe_joint*tx_lf_kfe_joint + ty_lf_kfe_joint*ty_lf_kfe_joint + -0.0*-0.0)) )), (GLdouble)-(ty_lf_kfe_joint), (GLdouble)tx_lf_kfe_joint, (GLdouble)0.0);
myDrawGLElement(::LF_KFE_JOINT, 0.3543595482976232, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_lf_kfe_joint, (GLdouble)ty_lf_kfe_joint, (GLdouble)-0.0);
glRotated((GLdouble)(RAD2DEG*-3.1415927410125732), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::LF_KFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Draw the end effector
glPushMatrix();
x = eff[2].x[_X_];
y = eff[2].x[_Y_];
z = eff[2].x[_Z_];
glRotated(
        (GLdouble)RAD2DEG*acos(z),(GLdouble)-y,(GLdouble)x,(GLdouble)0);
myDrawGLElement(102, (double)Sqrt(x*x + y*y + z*z), 0);
glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();
// Joint rh_haa_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( tz_rh_haa_joint/(std::sqrt(tx_rh_haa_joint*tx_rh_haa_joint + ty_rh_haa_joint*ty_rh_haa_joint + tz_rh_haa_joint*tz_rh_haa_joint)) )), (GLdouble)-(ty_rh_haa_joint), (GLdouble)tx_rh_haa_joint, (GLdouble)0.0);
myDrawGLElement(::RH_HAA_JOINT, 0.37743377335937367, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_rh_haa_joint, (GLdouble)ty_rh_haa_joint, (GLdouble)tz_rh_haa_joint);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*-1.5707963705062866), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::RH_HAA_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint rh_hfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( 0.0/(std::sqrt(0.0*0.0 + ty_rh_hfe_joint*ty_rh_hfe_joint + 0.0*0.0)) )), (GLdouble)-(ty_rh_hfe_joint), (GLdouble)0.0, (GLdouble)0.0);
myDrawGLElement(::RH_HFE_JOINT, 0.10769999772310257, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.0, (GLdouble)ty_rh_hfe_joint, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*-1.5707963705062866), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::RH_HFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint rh_kfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( -0.0/(std::sqrt(tx_rh_kfe_joint*tx_rh_kfe_joint + ty_rh_kfe_joint*ty_rh_kfe_joint + -0.0*-0.0)) )), (GLdouble)-(ty_rh_kfe_joint), (GLdouble)tx_rh_kfe_joint, (GLdouble)0.0);
myDrawGLElement(::RH_KFE_JOINT, 0.3543595482976232, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_rh_kfe_joint, (GLdouble)ty_rh_kfe_joint, (GLdouble)-0.0);
glRotated((GLdouble)(RAD2DEG*-3.1415927410125732), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::RH_KFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Draw the end effector
glPushMatrix();
x = eff[3].x[_X_];
y = eff[3].x[_Y_];
z = eff[3].x[_Z_];
glRotated(
        (GLdouble)RAD2DEG*acos(z),(GLdouble)-y,(GLdouble)x,(GLdouble)0);
myDrawGLElement(103, (double)Sqrt(x*x + y*y + z*z), 0);
glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();
// Joint lh_haa_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( tz_lh_haa_joint/(std::sqrt(tx_lh_haa_joint*tx_lh_haa_joint + ty_lh_haa_joint*ty_lh_haa_joint + tz_lh_haa_joint*tz_lh_haa_joint)) )), (GLdouble)-(ty_lh_haa_joint), (GLdouble)tx_lh_haa_joint, (GLdouble)0.0);
myDrawGLElement(::LH_HAA_JOINT, 0.37743377335937367, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_lh_haa_joint, (GLdouble)ty_lh_haa_joint, (GLdouble)tz_lh_haa_joint);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::LH_HAA_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint lh_hfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( 0.0/(std::sqrt(0.0*0.0 + ty_lh_hfe_joint*ty_lh_hfe_joint + 0.0*0.0)) )), (GLdouble)-(ty_lh_hfe_joint), (GLdouble)0.0, (GLdouble)0.0);
myDrawGLElement(::LH_HFE_JOINT, 0.10769999772310257, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.0, (GLdouble)ty_lh_hfe_joint, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707963705062866), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::LH_HFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint lh_kfe_joint

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( -0.0/(std::sqrt(tx_lh_kfe_joint*tx_lh_kfe_joint + ty_lh_kfe_joint*ty_lh_kfe_joint + -0.0*-0.0)) )), (GLdouble)-(ty_lh_kfe_joint), (GLdouble)tx_lh_kfe_joint, (GLdouble)0.0);
myDrawGLElement(::LH_KFE_JOINT, 0.3543595482976232, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)tx_lh_kfe_joint, (GLdouble)ty_lh_kfe_joint, (GLdouble)-0.0);
glRotated((GLdouble)(RAD2DEG*-3.1415927410125732), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::LH_KFE_JOINT].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Draw the end effector
glPushMatrix();
x = eff[4].x[_X_];
y = eff[4].x[_Y_];
z = eff[4].x[_Z_];
glRotated(
        (GLdouble)RAD2DEG*acos(z),(GLdouble)-y,(GLdouble)x,(GLdouble)0);
myDrawGLElement(104, (double)Sqrt(x*x + y*y + z*z), 0);
glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();

// pops the first matrix related to the state of the base
glPopMatrix();
