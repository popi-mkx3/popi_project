#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_dynamics.h" // only because of setDefaultEndeffector()


char joint_names[][20]= {
    {"BASE"}
    ,{"rf_haa_joint"}
    ,{"rf_hfe_joint"}
    ,{"rf_kfe_joint"}
    ,{"lf_haa_joint"}
    ,{"lf_hfe_joint"}
    ,{"lf_kfe_joint"}
    ,{"rh_haa_joint"}
    ,{"rh_hfe_joint"}
    ,{"rh_kfe_joint"}
    ,{"lh_haa_joint"}
    ,{"lh_hfe_joint"}
    ,{"lh_kfe_joint"}
};

// TODO names for the end-effectors
char cart_names[][20]= {
    {"dummy"},
    {"ENDEFF"}
};

// TODO names for the dummy links for the end-effectors
char link_names[][20]= {
    {"BASE"},
    {"LNK_EpauleAVD"},
    {"LNK_HJambeAVD"},
    {"LNK_BJambeAVD"},
    {"LNK_EpauleAVG"},
    {"LNK_HJambeAVG"},
    {"LNK_BJambeAVG"},
    {"LNK_EpauleARD"},
    {"LNK_HJambeARD"},
    {"LNK_BJambeARD"},
    {"LNK_EpauleARG"},
    {"LNK_HJambeARG"},
    {"LNK_BJambeARG"},
    {"LNK_dummy_ee"}
};
// don't really know what this is for, but it is required for linking...
char blob_names[][20]= {
  {"dummy"},
  {"BLOB1"},
  {"BLOB2"},
  {"BLOB3"},
  {"BLOB4"},
  {"BLOB5"},
  {"BLOB6"}
};

char misc_sensor_names[][20]= {
    {"dummy"},
    {"B_Q_0"},
    {"B_Q_1"},
    {"B_Q_2"},
    {"B_Q_3"},
    {"B_Qd_0"},
    {"B_Qd_1"},
    {"B_Qd_2"},
    {"B_Qd_3"},
    {"B_Qdd_0"},
    {"B_Qdd_1"},
    {"B_Qdd_2"},
    {"B_Qdd_3"},
    {"B_Ad_A"},
    {"B_Ad_B"},
    {"B_Ad_G"},
    {"B_Add_A"},
    {"B_Add_B"},
    {"B_Add_G"},
    {"B_X"},
    {"B_Y"},
    {"B_Z"},
    {"B_Xd"},
    {"B_Yd"},
    {"B_Zd"},
    {"B_Xdd"},
    {"B_Ydd"},
    {"B_Zdd"},
    {"TIME_MOTOR"}
};

int link2endeffmap[] = {0,DUMMY_EE}; // TODO fix
double test_goal[N_CART+1];
int    no_user_interaction_flag=FALSE;
int    real_time_flag=FALSE;
char   initial_task_name[100];

/* the following include must be the last line of the variable declaration section */
#include "SL_user_common.h"   /* do not erase!!! */


void setDefaultEndeffector(void)
{
    int i;
    for (i=1; i<=N_ENDEFFS; ++i) {
        endeff[i].m       = 0.0;
        endeff[i].mcm[_X_]= 0.0;
        endeff[i].mcm[_Y_]= 0.0;
        endeff[i].mcm[_Z_]= 0.0;
        endeff[i].x[_X_]  = 0.0;
        endeff[i].x[_Y_]  = 0.0;
        endeff[i].x[_Z_]  = 0.0;
        endeff[i].a[_A_]  = 0.0;
        endeff[i].a[_B_]  = 0.0;
        endeff[i].a[_G_]  = 0.0;
    }
}

void setRealRobotOptions(void)
{
    if (!real_robot_flag) {
        sprintf(config_files[CONFIGFILES],"ConfigFilesSim.cf");
    } else {
        sprintf(config_files[CONFIGFILES],"ConfigFiles.cf");
    }

    // update the config file names
    read_config_files(config_files[CONFIGFILES]);
}
