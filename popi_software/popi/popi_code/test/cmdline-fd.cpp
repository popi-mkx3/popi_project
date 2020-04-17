#include <iit/robcogen/test/cmdline_fd.h>
#include <iit/robots/popi/traits.h>

using namespace iit;

/**
 * This program calls the generated implementation of Forward Dynamics, and
 * prints the result (i.e. the joint acceleration) on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are 36
 * arguments, for the position, velocity and joint force of each joint of
 * the robot. Group the arguments by type, not by joint.
 */
int main(int argc, char** argv)
{
    robcogen::test::cmdline_fd_fb< popi::Traits >(argc, argv);
    return 0;
}
