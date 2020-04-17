include_directories(
  include/
)

# Popi Hardware Interface
add_library(popi_hw_interface
  src/popi_hw_interface.cpp
)
target_link_libraries(popi_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(popi_hw_main src/popi_hw_main.cpp)
target_link_libraries(popi_hw_main
  popi_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    popi_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    popi_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/popi_hw_interface/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
