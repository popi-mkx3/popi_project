#!/usr/bin/octave -qf

## This Octave program demonstrates how to use the test functions shipped with
## RobCoGen, to test the generated code (both Octave and C++). Please refer to
## '<robcogen root>/octave-tests/readme.md' for more information.

## Set up the workspace first:
run octave-startup.m


printf('\nTest of the generated Octave code for the Joint Space Inertia Matrix\n');
[me.res roy.res] = test_jsim_fb(roy.model, me);

printf('\nTest of the generated Octave code for Inverse Dynamics\n');
[me.res roy.res] = test_id_fb(roy.model, me);

printf('\nTest of the generated Octave code for Forward Dynamics\n');
[me.res roy.res] = test_fd_fb(roy.model, me);


printf('\nTest of the generated C++ code for the Joint Space Inertia Matrix\n');
[me.res roy.res] = testcpp_jsim_fb(roy.model, exe_im);

printf('\nTest of the generated C++ code for Inverse Dynamics\n');
[me.res roy.res] = testcpp_id_fb(roy.model, exe_id);

printf('\nTest of the generated C++ code for Forward Dynamics\n');
[me.res roy.res] = testcpp_fd_fb(roy.model, exe_fd);
