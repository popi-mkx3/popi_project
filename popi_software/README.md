# popi_software

<img align="center" src="https://i.imgur.com/ONQVNOU.png" width="100%"/>
<br>
<br>

POPI is an entirely open-source quadruped robot. You will find here all the source code used on POPI. To have a look on the relevant technical material, you can see our other repository [popi_docs]. These docs unfortunately are in French for now, but mechanical drawings should still speak to anyone. Let us know if you ever need a translation and we'll do our best !  
The code is based on [ROS]. You can run it without the robot, develop your own code and try it in a [Gazebo] simulation.
<br>
<br>

:heavy_check_mark: Trajectory generation and optimization based on [Towr].  
:heavy_check_mark: Easy recording with [rqt_bag].  
:heavy_check_mark: Actuators' control based on [ROS control boilerplate].  
:heavy_check_mark: Available simulation on Gazebo.  
<br>
<br>

<p align="center">
  <a href="#soft">POPI software</a> •
  <a href="#install-dependecies">Install dependencies</a> •
  <a href="#run">Run</a> •
  <a href="#contribute">Contribute</a>
</p>

<p align="center">
	<img src="https://i.imgur.com/zi1gDp3.gif" />
</p>

## <a name="soft"></a> POPI software
The brains of POPI are split between 3 single-board computers (one Raspberry Pi 3B+ and two BeagleBone Black) and one (offboard) user computer. It can seem daunting at first, but it allows for a more distributed work, and endows POPI with a computational power that is sufficient even for its future developments. Moreover the source code is entirely based on [ROS], which makes it really easy to run code across multiple machines.

Each computer has a special use, as shown on the diagram below.
<p align="center">
  <img src="https://i.imgur.com/3t14bqz.png" /> 
</p>

If you build your own POPI, you will need to download the [flash images] for the 3 SD cards. <b>Even if you don't plan on building POPI, you can still develop your own code or your own walking trajectories and try it on a virtual POPI in a [Gazebo] simulation !</b>

The instructions to download the source code and use it are available in the [popi_software] folder. Of course you will also find there more explanations about how it works to help you getting started.
<br>
<br> 

## <a name="install-dependecies"></a> Install dependencies
  
The environment we used is Ubuntu 18.04 with [ROS Melodic].
<br>

* First, install [ROS]:
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt-get update
   sudo apt-get install ros-melodic-desktop-full
   ```
	Do the following to get the latest version of Gazebo 9 (we used the 9.12.0):
   ```bash
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   ```
  
	Then update your packages:
   ```bash
   sudo apt-get update
   sudo apt-get upgrade
   ```
   Finally, install the other dependencies:
   ```bash
   sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-robot-controllers ros-melodic-robot-state-publisher ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-rosparam-shortcuts ros-melodic-joy git cmake libeigen3-dev coinor-libipopt-dev libncurses5-dev libgflags-dev libboost-all-dev xterm
   ```
   If you encounter any problem or would like any details about these dependencies, you will find [here](https://github.com/popi-mkx3/popi_ros/blob/master/CONFIGURATION.md) a list of the versions of the packages we had installed, aswell with links to their respective websites.
<br>

* Create the workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```
* Either download the whole repository:
   ```bash
   git clone https://github.com/popi-mkx3/popi_project.git
   ```
* Or only the popi_software folder:
   ```bash
   git init
   git remode add -f origin https://github.com/popi-mkx3/popi_ros.git
   git config core.sparseCheckout true
   echo "popi_software" >> .git/info/sparse-checkout
   git pull origin master
   ```
<br>

* Build the workspace:
   ```bash
   cd ~/catkin_ws
   source /opt/ros/melodic/setup.bash
   catkin_make_isolated
   ```
<br>

* Configuration:  
   Copy the following command to add these four lines to your `~/.bashrc`: 
   ```bash
   echo "source /opt/ros/melodic/setup.bash
   source ~/catkin_ws/devel_isolated/setup.bash
   #export ROS_IP=192.168.7.4
   #export ROS_MASTER_URI=http://192.168.7.1:11311" >> ~/.bashrc

   ```
	Leave the two last lines commented if you're just using the simulation.
<br>
<br>
 
## Run
<br>
<br>

## Contribute
The whole point of making this project fully open-source is to have anyone who is interested contribute to POPI ! Whether it includes documentation translations, new functionalities, bug fixes or code improvements, we'll be glad to receive your pull request !  
We're fully aware we still have a long road to go before POPI becomes an impressive robot, and we'll be happy to take anyone with us onboard.  
You can see here the list of [contributors](https://github.com/popi-mkx3/popi_ros/graphs/contributors) who participated in this project.
<br>
<br>

## <a name="team"></a> Meet the team !

<p align="center"><b> Project Leader </b></p>
<p align="center">
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/clément-thomaso-6b9ab910b/"><img src="https://i.imgur.com/Q7O4i6e.png" width="10%" alt="Clément Thomaso"/></a>
</p>

<p align="center"><b> Mechanics </b> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <b> Electronics </b> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <b> Programming </b></p>

<p align="center">
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/rémi-combacal-16032214a/"><img src="https://i.imgur.com/Dx2GRi4.png" width="10%" alt="Rémi Combacal"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/anaïs-gutton-655383a6/"><img src="https://i.imgur.com/yyJLZzI.png" width="10%" alt="Anaïs Aharram-Gutton"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/olivier-peres/"><img src="https://i.imgur.com/6CifAs4.png" width="10%" alt="Olivier Peres"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/guillaume-rougé-913a10108/"><img src="https://i.imgur.com/d3qdMGd.png" width="10%" alt="Guillaume Rougé"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/clémence-graton-a02200174/"><img src="https://i.imgur.com/XqYmJ73.png" width="10%" alt="Clémence Graton"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/jean-pelloux-prayer-b38a32143/"><img src="https://i.imgur.com/WhTJOjs.png" width="10%" alt="Jean Pelloux-Prayer"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/yannis-oddon-442717128/"><img src="https://i.imgur.com/5yZow0U.png" width="10%" alt="Yannis Oddon"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/karla-brottet-69440/"><img src="https://i.imgur.com/ORwYYx3.png" width="10%" alt="Karla Brottet"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/lucas-labarussiat/"><img src="https://i.imgur.com/pJAIIhz.png" width="10%" alt="Lucas Labarussiat"/></a>
</p>
<br>
<br>

<img src="https://i.imgur.com/h6RkNK1.jpg" height="151"/> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <img src="https://i.imgur.com/MZJbr31.png" height="151"/> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <img src="https://i.imgur.com/P2nOGKx.jpg" height="151"/>

[popi_docs]: https://github.com/popi-mkx3/popi_docs
[ROS]: https://www.ros.org/
[Towr]: https://github.com/ethz-adrl/towr
[rqt_bag]: http://wiki.ros.org/rqt_bag
[xpp]: http://wiki.ros.org/xpp
[RViz]: http://wiki.ros.org/rviz
[URDF]: http://wiki.ros.org/urdf
[Gazebo]: http://gazebosim.org/
[rqt_gui]: http://wiki.ros.org/rqt_gui
[ROS Melodic]: http://wiki.ros.org/melodic/Installation/Ubuntu

[BalenaEtcher]: https://www.minimachines.net/tutos/etcher-flasher-image-usb-50144
[ROS control boilerplate]: https://github.com/PickNikRobotics/ros_control_boilerplate

[Alexander W. Winkler]: https://www.alex-winkler.com
[Dave Coleman]: http://dav.ee/
[TiMOTION]: https://www.timotion.com/fr
[IMT Mines d'Alès]: https://www.mines-ales.fr/

[Clément]: https://www.linkedin.com/in/clément-thomaso-6b9ab910b/
[Rémi]: https://www.linkedin.com/in/rémi-combacal-16032214a/
[Anaïs]: https://www.linkedin.com/in/anaïs-gutton-655383a6/
[Olivier]: https://www.linkedin.com/in/olivier-peres/
[Guillaume]: https://www.linkedin.com/in/guillaume-rougé-913a10108/
[Clémence]: https://www.linkedin.com/in/clémence-graton-a02200174/
[Jean]: https://www.linkedin.com/in/jean-pelloux-prayer-b38a32143/
[Yannis]: https://www.linkedin.com/in/yannis-oddon-442717128/
[Karla]: https://www.linkedin.com/in/karla-brottet-69440/
[Lucas]: https://www.linkedin.com/in/lucas-labarussiat/