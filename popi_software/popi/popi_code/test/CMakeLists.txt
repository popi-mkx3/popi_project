cmake_minimum_required(VERSION 2.8)
project(robcogen-popi-cpptests)

set(EIGEN_ROOT $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")
set(CMAKE_CXX_FLAGS "-g -Wall -O3 -D EIGEN_NO_DEBUG -std=c++11")

# Include directories
include_directories(${EIGEN_ROOT})

# Optional CppAD
#
set(CPPAD off CACHE BOOL "Enable tests of CppAD traits")
if(CPPAD)
    set(CPPAD_ROOT $ENV{CPPAD_ROOT}   CACHE PATH "Path to CppAD headers")
    include_directories(${CPPAD_ROOT})
endif(CPPAD)

# The binaries
add_executable(fd   cmdline-fd.cpp)
add_executable(id   cmdline-id.cpp)
add_executable(jsim cmdline-jsim.cpp)
add_executable(cons dyn-consistency.cpp)

target_link_libraries(fd   iitgenpopi)
target_link_libraries(id   iitgenpopi)
target_link_libraries(jsim iitgenpopi)
target_link_libraries(cons iitgenpopi)
