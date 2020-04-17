#include <iit/robcogen/test/cmdline_jsim.h>
#include <iit/robots/popi/traits.h>

using namespace iit;

/**
 * This program calls the generated implementation of the algorithm to calculate
 * the Joint Space Inertia Matrix, and prints it on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are
 * 12 arguments, for the position status of each joint of
 * the robot.
 */
int main(int argc, char** argv)
{
    robcogen::test::cmdline_jsim< popi::Traits >(argc, argv);
    return 0;
}
