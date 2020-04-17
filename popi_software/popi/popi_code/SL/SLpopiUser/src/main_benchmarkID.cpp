#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdlib>

#include "SL.h"
#include "SL_user.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"

using namespace std;

static void fillState(SL_DJstate* desiredState);
static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject);

/* This main is supposed to be used to test the inverse dynamics routines */
int main(int argc, char**argv)
{
    if(argc < 2) {
        cerr << "Please provide the number of tests to perform" << endl;
        return -1;
    }
    int numOfTests = std::atoi(argv[1]);
    double sl[numOfTests];
    int iterations[numOfTests];

    double t0, duration, sl_total;
    sl_total = 0;

    int t=0,i=0;

    SL_Jstate currentState[N_ROBOT_DOFS];
    SL_DJstate desiredState[N_ROBOT_DOFS];
    SL_endeff endeffector[N_ROBOT_ENDEFFECTORS];
    SL_Cstate basePosition;
    SL_quat baseOrient;

    init_kinematics();
    if( ! init_dynamics() ) {
        cerr << "Error in init_dynamics()" << endl;
        exit(-1);
    }

    for(i=0; i<N_ROBOT_DOFS; i++) {
        currentState[i].th   = 0;
        currentState[i].thd  = 0;
        currentState[i].thdd = 0;

        desiredState[i].th   = 0;
        desiredState[i].thd  = 0;
        desiredState[i].thdd = 0;

        desiredState[i].uff  = 0;
        desiredState[i].uex  = 0;
    }

    // Zeroes out every end effector:
    for(int e=1; e<=N_ENDEFFS; e++) {
        endeffector[e].m = 0;
        for(i=0; i<=N_CART; i++) {
            endeffector[e].x[i]   = 0;
            endeffector[e].a[i]  = 0;
        }
    }


    for(i=0; i<=N_CART; i++) {
        basePosition.x[i]   = 0;
        basePosition.xd[i]  = 0;
        basePosition.xdd[i] = 0;
    }
    baseOrient.q[_Q0_] = 1;
    baseOrient.q[_Q1_] = 0;
    baseOrient.q[_Q2_] = 0;
    baseOrient.q[_Q3_] = 0;

    // This restores the default end effector parameters (ie non zero values as opposed
    //  to what we did above): for some reason this makes inverse dynamics much slower (???)
    //setDefaultEndeffector();

    std::srand(std::time(NULL)); // initialize random number generator

    /*
    // Prints numerical results, for comparison
    fillState(desiredState);
    SL_InvDynNE(NULL, desiredState, endeffector, &basePosition, &baseOrient);
    cout << "SL:" << endl
    << desiredState[rf_haa_joint].uff << endl
    << desiredState[rf_hfe_joint].uff << endl
    << desiredState[rf_kfe_joint].uff << endl
    << desiredState[lf_haa_joint].uff << endl
    << desiredState[lf_hfe_joint].uff << endl
    << desiredState[lf_kfe_joint].uff << endl
    << desiredState[rh_haa_joint].uff << endl
    << desiredState[rh_hfe_joint].uff << endl
    << desiredState[rh_kfe_joint].uff << endl
    << desiredState[lh_haa_joint].uff << endl
    << desiredState[lh_hfe_joint].uff << endl
    << desiredState[lh_kfe_joint].uff << endl
        ;
   return 1;
   //*/

    int numOfIterations = 1;
    for(t=0; t<numOfTests; t++) {
        sl_total = 0;
        numOfIterations = numOfIterations * 10;
        iterations[t] = numOfIterations;

        for(i=0; i<numOfIterations; i++) {
            fillState(desiredState);

            t0 = std::clock();
            SL_InvDynNE(NULL, desiredState, endeffector, &basePosition, &baseOrient);
            duration = std::clock() - t0;
            sl_total += duration;
        }
        sl[t] = sl_total/CLOCKS_PER_SEC;
    }


   for(int t=0; t<numOfTests; t++) {
        cout << sl[t] << endl;
    }
    matlabLog(numOfTests, iterations, sl, "inv_dyn");

    return TRUE;
}


void fillState(SL_DJstate* desiredState) {
    static const double max = 12.3;
    for(int i=0; i<N_ROBOT_DOFS; i++) {
        desiredState[i].th   = ( ((double)std::rand()) / RAND_MAX) * max;
        desiredState[i].thd  = ( ((double)std::rand()) / RAND_MAX) * max;
        desiredState[i].thdd = ( ((double)std::rand()) / RAND_MAX) * max;
    }
}

static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject) {
    std::string fileName = "popi_" + subject + "_speed_test_data.m";
    ofstream out(fileName.c_str());
    out << "popi_test.robot       = 'popi';" << std::endl;
    out << "popi_test.description = 'test of the speed of the calculation of: " << subject << "';" << std::endl;
    out << "popi_test.software    = 'SL';" << std::endl;

    // Current date/time based on current system
    time_t now = std::time(0);
    tm* localtm = std::localtime(&now); // Convert now to tm struct for local timezone
    char timeStr[64];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d  %X",localtm);
    out << "popi_test.date = '" << timeStr << "';" << std::endl;

    out << "popi_test.iterations = [";
    for(int t=0; t<numOfTests; t++) {
        out << " " << iterations[t];
    }
    out << "];" << endl;
    out << "popi_test.times = [";
    for(int t=0; t<numOfTests; t++) {
        out << " " << tests[t];
    }
    out << "];" << endl;
    out.close();
}
