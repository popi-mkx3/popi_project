# Auto-generated CMake file
#
#
# Project configuration
cmake_minimum_required(VERSION 2.8)
project(SLpopiUser)
set(EXECUTABLE_OUTPUT_PATH ..)

set(EIGEN_ROOT   $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")

set(GENCODELIB iitgenpopi)

set(COMMON_LIBS popicommon SLcommon utilities)

# Include directories
include_directories(${SL_ROOT}/utilities/include)
include_directories(${SL_ROOT}/SL/include)
include_directories(${SL_ROOT}/${ROBOT}/math)
include_directories(${SL_ROOT}/${ROBOT}/include)
include_directories(include)

include_directories(${EIGEN_ROOT})

set(TASK_SOURCES
    ${SOURCE_SL_TASK}
    ${SOURCE_TASK}
    src/initUserTasks.cpp)

set(OPENGL_SOURCES
    ${SOURCE_SL_OPENGL}
    ${SOURCE_OPENGL}
    src/initUserGraphics.cpp)

if (XENO)

    add_executable(xenotask ${TASK_SOURCES})
    target_link_libraries(xenotask
        ${COMMON_LIBS}
        ${XENOTASK_LIBS}
        ${GENCODELIB})

    add_executable(xenoopengl ${OPENGL_SOURCES})
    target_link_libraries(xenoopengl
        ${COMMON_LIBS}
        ${XENOOPENGL_LIBS}
        ${GENCODELIB})

else (XENO)
    # Executables:

    add_executable(xtask ${TASK_SOURCES})
    target_link_libraries(xtask
        ${COMMON_LIBS}
        ${XTASK_LIBS}
        ${GENCODELIB})

    add_executable(xopengl ${OPENGL_SOURCES})
    target_link_libraries(xopengl
        ${COMMON_LIBS}
        ${XOPENGL_LIBS}
        ${GENCODELIB})

    add_executable(xsim
        ${SOURCE_SL_SIM}
        ${SOURCE_SIM}
        src/initUserSimulation.cpp)

    target_link_libraries(xsim
        ${COMMON_LIBS}
        ${XSIM_LIBS}
        ${GENCODELIB})
endif(XENO)
