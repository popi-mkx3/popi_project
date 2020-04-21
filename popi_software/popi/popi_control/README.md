## Credits
<br>
<b>
Please check the original repository of this folder [here](https://github.com/PickNikRobotics/ros_control_boilerplate) ! 
</b>
<br>
<br>
<br>
<br>
<br>
<br>


# ROS Control Boilerplate

Simple simulation interface and template for setting up a hardware interface for ros_control. The idea is you take this as a starting point for creating your hardware interfaces, and it is needed because [ros_control documentation](http://wiki.ros.org/ros_control) is sparse. This boilerplate demonstrates:

 - Creating a hardware_interface for multiple joints for use with ros_control
 - Position Trajectory Controller
 - Control of 2 joints of the simple robot "Popi" pictured below
 - Loading configurations with roslaunch and yaml files
 - Generating a random trajectory and sending it over an actionlib interface
 - Partial support of joint mode switching (needs to be improved)
 - Joint limits
 - Pass-through non-physics based robot simulator
 - Visualization in Rviz

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder

<a href='https://ko-fi.com/A7182AMW' target='_blank'><img height='36' style='border:0px;height:36px;' src='https://az743702.vo.msecnd.net/cdn/kofi2.png?v=0' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>

 * [![Build Status](https://travis-ci.org/davetcoleman/popi_control.svg)](https://travis-ci.org/davetcoleman/popi_control) Travis CI
 * [![Devel Job Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-popi_control)](http://jenkins.ros.org/job/devel-indigo-popi_control) Devel Job Status
 * [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-ros-control-boilerplate_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-ros-control-boilerplate_binarydeb_trusty_amd64/) AMD64 Debian Job Status

<img src="https://raw.githubusercontent.com/davetcoleman/popi_control/jade-devel/resources/screenshot.png"/>


## Video Demo

See [YouTube](https://www.youtube.com/watch?v=Tpj2tx9uZ-o) for a very modest video demo.

## Install

This package depends on [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) for its ``popi_description`` package, but you must add it to your catkin workspace by source:

    git clone https://github.com/ros-simulation/gazebo_ros_demos.git

Then, either install this package from source so you can develop off of it, or install from debian:

    sudo apt-get install ros-indigo-ros-control-boilerplate

## Run Simulation Demo

This package is setup to run the "Popi" two joint revolute-revolute robot demo. This "template package" is located in the popi_control as a subfolder that you can easily rename and reuse. To run its ros_control non-physics-based simulated hardware interface, run:

    roslaunch popi_control popi_simulation.launch

To visualize its published ``/tf`` coordinate transforms in Rviz run:

    roslaunch popi_control popi_visualize.launch

To send a random, dummy trajectory to execute, run:

    roslaunch popi_control popi_test_trajectory.launch

## Customize

To test this as a simulation interface for your robot, you can quickly rename the subfolder package into the name of your robot using the following commands:

```
function findreplace() {
    grep -lr -e "$1" * | xargs sed -i "s/$1/$2/g" ;
}

function findreplacefilename() {
    find . -depth -name "*$1*" -exec bash -c 'for f; do base=${f##*/}; mv -- "$f" "${f%/*}/${base//'$1'/'$2'}"; done' _ {} +
}

findreplacefilename popi myrobot
findreplace popi myrobot
findreplace Popi MyRobot
findreplace POPI MYROBOT
```

Then add the necessary code to communicate with your robot via USB/serial/ethernet/etc in the file ``myrobot_hw_interface.cpp``.

## Setting an Initial Position, Using with MoveIt!

If you need your robot to startup at a particular position in simulation, or you would like to use this funcitonality to simulate your robot with MoveIt!, see the downstream package (it depends on this package) [moveit_sim_controller](https://github.com/davetcoleman/moveit_sim_controller)

## Other Helper Tools

### Recording to CSV

Write the commands from a trajectory controller to csv file

    rosrun popi_control controller_to_csv SAVE_TO_FILE_PATH CONTROLLER_STATE_TOPIC TIME_TO_RECORD

### Commanding from CSV

Read from csv file and execute on robot

    rosrun popi_control csv_to_controller READ_FROM_FILE_PATH CONTROLLER_STATE_TOPIC TIME_TO_RECORD

### Commanding from Keyboard

Joint-level teleop from a keyboard (TODO: remove had coded topic names)

    rosrun popi_control keyboard_teleop

## Limitations

 - Does not implement estops, transmissions, or other fancy new features of ros_control
 - Does not have any hard realtime code, this depends largely on your platform, kernel, OS, etc
 - Only position control is fully implemented, though some code is in place for velocity control

## Contribute

Please add features, make corrections, and address the limitations above, thanks!
