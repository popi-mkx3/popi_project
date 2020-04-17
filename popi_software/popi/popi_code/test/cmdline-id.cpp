#include <iit/robcogen/test/cmdline_id.h>
#include <iit/robots/popi/traits.h>

using namespace iit;

/**
 * This program calls the generated implementation of Inverse Dynamics, and
 * prints the result (i.e. the joint forces) on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are 36
 * arguments, for the position, velocity and acceleration of each joint of
 * the robot. Group the arguments by type, not by joint.
 */
int main(int argc, char** argv)
{
    robcogen::test::cmdline_id_fb< popi::Traits >(argc, argv);
    return 0;
}
