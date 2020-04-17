cmake_minimum_required(VERSION 2.8.3)
project(popi_robot)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY
   msg
   FILES
   Float64Array.msg
   CmdPosArray.msg
   Potards.msg
   RouesCodeuses.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  Single_sensor_read.srv
  Multi_sensor_read.srv
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
 
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_executable(left_multi_sensor_client src/left_multi_sensor_client.cpp)
target_link_libraries(left_multi_sensor_client ${catkin_LIBRARIES})
add_dependencies(left_multi_sensor_client popi_robot_gencpp)

add_executable(right_single_sensor_client src/right_single_sensor_client.cpp)
target_link_libraries(right_single_sensor_client ${catkin_LIBRARIES})
add_dependencies(right_single_sensor_client popi_robot_gencpp)

add_executable(right_multi_sensor_client src/right_multi_sensor_client.cpp)
target_link_libraries(right_multi_sensor_client ${catkin_LIBRARIES})
add_dependencies(right_multi_sensor_client popi_robot_gencpp)

add_executable(manette_yannis src/manette_yannis.cpp)
target_link_libraries(manette_yannis ${catkin_LIBRARIES})
add_dependencies(manette_yannis popi_robot_gencpp) ## ${catkin_EXPORTED_TARGETS})

add_executable(manette_vitesse src/manette_vitesse.cpp)
target_link_libraries(manette_vitesse ${catkin_LIBRARIES})
add_dependencies(manette_vitesse popi_robot_gencpp) ## ${catkin_EXPORTED_TARGETS})

add_executable(manette src/manette.cpp)
target_link_libraries(manette ${catkin_LIBRARIES})
add_dependencies(manette popi_robot_gencpp) ## ${catkin_EXPORTED_TARGETS})