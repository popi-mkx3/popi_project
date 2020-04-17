# Project configuration
cmake_minimum_required(VERSION 2.8)
project(SLpopi)
set(EXECUTABLE_OUTPUT_PATH ..)

set(EIGEN_ROOT   $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")
set(GENCODELIB iitgenpopi)

# Include directories
include_directories(${SL_ROOT}/utilities/include)
include_directories(${SL_ROOT}/SL/include)
include_directories(math)
include_directories(include)

include_directories(${EIGEN_ROOT})

# The source files directory
set(SOURCE_DIR ${SL_ROOT}/${ROBOT}/src)

set(SOURCES_COMMON
    ${SOURCE_SL_COMMON}
    ${SOURCE_DIR}/robcogen_globals.cpp
    ${SOURCE_DIR}/SL_user_commands.cpp
    ${SOURCE_DIR}/SL_user_common.cpp
    ${SOURCE_DIR}/SL_dynamics.cpp
    ${SOURCE_DIR}/SL_kinematics.cpp
    )

if (XENO)
  # Add extra dependencies for Xenomai
  include_directories(${IIT_IO_ROOT})

  # Add library
  add_library(popicommon STATIC ${SOURCES_COMMON})

  # Add executable
  add_executable(xenomotor
    ${SOURCE_SL_MOTOR}
    src/SL_user_motor.cpp
    src/SL_user_sensor_proc_xeno.cpp)

  target_link_libraries(xenomotor
    popicommon SLcommon utilities iitio
    ${XENOMOTOR_LIBS}  ${GENCODELIB})

  # Add source files lists
  set(SOURCE_TASK
    ${SOURCE_DIR}/SL_user_task_xeno.cpp
    PARENT_SCOPE)

  set(SOURCE_OPENGL
    ${SOURCE_DIR}/SL_user_openGL.cpp
    PARENT_SCOPE)

  set(SOURCE_SIM
    ${SOURCE_DIR}/SL_user_simulation.cpp
    PARENT_SCOPE)

else (XENO)
    # Add library
    add_library(popicommon STATIC ${SOURCES_COMMON})

    add_executable(xmotor
        ${SOURCE_SL_MOTOR}
        src/SL_user_motor.cpp
        src/SL_user_sensor_proc_unix.cpp)

    target_link_libraries(xmotor
        popicommon SLcommon utilities
        ${XMOTOR_LIBS} ${GENCODELIB})

  if (ROS)
    # Add executable
    add_executable(xros
      ${SOURCE_SL_ROS}
      src/SL_user_ros.cpp)

    target_link_libraries(xros
      popicommon SLcommon utilities
      ${ROSPACK_LIBS}
      ${XROS_LIBS})
  endif (ROS)

  # Add source files lists
  set(SOURCE_TASK
    ${SOURCE_DIR}/SL_user_task.cpp
    PARENT_SCOPE)

  set(SOURCE_OPENGL
    ${SOURCE_DIR}/SL_user_openGL.cpp
    PARENT_SCOPE)

  set(SOURCE_SIM
    ${SOURCE_DIR}/SL_user_simulation.cpp
    PARENT_SCOPE)
endif (XENO)
