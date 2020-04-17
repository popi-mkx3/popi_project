cmake_minimum_required(VERSION 2.8.3)
project(popi_description)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    std_msgs
    tf
    gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)
