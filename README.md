# POPI

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
  <a href="#contribute">Contribute</a> •
  <a href="#publications">Publications</a> •
  <a href="#team">Meet the team</a> •
  <a href="#acknowledgments">Acknowledgments</a>
</p>

<p align="center">
	<img src="https://i.imgur.com/s29RHe6.gif" />
</p>

## Contribute
The whole point of making this project fully open-source is to have anyone who is interested contribute to POPI ! Whether it includes documentation translations, new functionalities, bug fixes or code improvements, we'll be glad to receive your pull request !  
We're fully aware we still have a long road to go before POPI becomes an impressive robot, and we'll be happy to take anyone with us onboard.  
You can see here the list of [contributors](https://github.com/popi-mkx3/popi_ros/graphs/contributors) who participated in this project.
<br>
<br>

## Publications
<br>
<br>

## <a name="team"></a> Meet the team !

<p align="center"><b> Project Leader </b></p>
<p align="center">
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/clément-thomaso-6b9ab910b/"><img src="https://i.imgur.com/3dUgaow.png" width="10%" alt="Clément Thomaso"/></a>
</p>

<p align="center"><b> Mechanics </b> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <b> Electronics </b> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <b> Programming </b></p>

<p align="center">
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/rémi-combacal-16032214a/"><img src="https://i.imgur.com/ZVTtgik.png" width="10%" alt="Rémi Combacal"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/anaïs-gutton-655383a6/"><img src="https://i.imgur.com/ff2XJUW.png" width="10%" alt="Anaïs Aharram-Gutton"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/olivier-peres/"><img src="https://i.imgur.com/SLJLxvD.png" width="10%" alt="Olivier Peres"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/guillaume-rougé-913a10108/"><img src="https://i.imgur.com/kFxLFVK.png" width="10%" alt="Guillaume Rougé"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/clémence-graton-a02200174/"><img src="https://i.imgur.com/OOfYG0X.png" width="10%" alt="Clémence Graton"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/jean-pelloux-prayer-b38a32143/"><img src="https://i.imgur.com/4oOODpr.png" width="10%" alt="Jean Pelloux-Prayer"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/yannis-oddon-442717128/"><img src="https://i.imgur.com/YUDleYD.png" width="10%" alt="Yannis Oddon"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/karla-brottet-69440/"><img src="https://i.imgur.com/4GvkygH.png" width="10%" alt="Karla Brottet"/></a>
  <a target="_blank" rel="noopener noreferrer" href="https://www.linkedin.com/in/lucas-labarussiat/"><img src="https://i.imgur.com/eMQNCa7.png" width="10%" alt="Lucas Labarussiat"/></a>
</p>

<br>
<br>

## Acknowledgments
First we would like to thank deeply [Alexander W. Winkler] and [Dave Coleman] for their respective work on [Towr] and [ROS control boilerplate].  
Then let us thank [TiMOTION] who gave us some precious advice along with our knee actuators. Without their contribution, POPI wouldn't be able to lift his legs !  
Finally we of course want to express our sincere gratitude to all our teachers from [IMT Mines d'Alès] who gave us the opportunity to undertake this project and were there to lend us a hand when we needed it the most.
<br>
<br>

<img src="https://i.imgur.com/Qv3iWwq.jpg" height="151"/> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <img src="https://i.imgur.com/5oggd8Z.png" height="151"/> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <img src="https://i.imgur.com/TJwQi9n.jpg" height="151"/>

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