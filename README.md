# POPI

<img align="center" src="https://i.imgur.com/ftqUFxM.png" width="100%"/>
<br>
<br>

## News : Walko
Below are the links to the follow up of the POPI project, on a new quadruped. Our whole work will be once again open source and we should start releasing it in September 2021. <br>
The goal is still the same : allowing everybody to build their own quad. Unlike POPI, our new quad will be 100% 3d-printed to make this goal closer, with components available to purchase online. We will provide as soon as possible a BOM and links to the manufacturers' websites (we are not resellers). <br>
According to our current estimations, building your own quad will cost around 2,7k€ altogether (without the camera module though), which is still expensive but rather cheap when considering the size of the robot and the price of similar-sized robots.
<br>
We aim to provide beginner-friendly step-by-step instructions along the road, and to keep improving the tech as well.
<br>
<br>
If you want to stay tuned about this : <br>
[Website](https://walko-robotics.com) <br>
[GitLab](https://gitlab.com/walko-robotics/doge/software) and [GitHub](https://github.com/Walko-Robotics) - Please keep in mind that the GitHub repo will be a mirror of the GitLab one from now on. <br>
[Instagram](https://www.instagram.com/walko_adventures/) <br>
[Discord](dsc.gg/walko) <br>
[YouTube](https://www.youtube.com/c/WalkoRobotics) <br> <br>

## Overview of the project
POPI is an entirely open-source quadruped robot. Though not yet extremely agile, <a href="#team">we</a> meant it to be accessible to anyone interested in robotics and we hope it becomes a friendly collaborative platform for educational or research purposes. You will hence find in this repository everything there is to know about POPI. If you want to know about every details, a good way would be to have a look at the user manual in the [popi_reports] folder.

This project initially started as a student work. As a team of 10 not-yet engineers, we designed and built it from scratch with a 4k€ budget over a six-month-long period during which we still had to take other classes and exams. As such, we are proud of what we achieved but we do know POPI couldn't compete with other quadrupeds as-is. Some of us are still working on this prototype on our free time to improve it, and we will share here everything. In the same time we are also rethinking the whole design to create a new, lighter and mostly 3D-printed POPI.

We are actually looking for some fundings to get this new version up and running and are interested in any opportunity we could have to continue our work. If you have any idea about this or would like to discuss some kind of partnership, please let us know by sending us an <a href = "mailto: popi.mkx3@gmail.com">email</a>.

Otherwise, you can still have a look at what we've done so far on this first prototype. Please keep in mind we inevitably made some mistakes considering it is our first robot and we had limited time and budget. We'll be more than happy to get some advice and discuss any part of our work with you.

#### What is available
:heavy_check_mark: User manual  
:heavy_check_mark: Bill of materials  
:heavy_check_mark: Mechanical drawings and CAD files  
:heavy_check_mark: Electrical drawings  
:heavy_check_mark: Source code (ROS and C++)  
:heavy_check_mark: Cool 3D-renders and wallpapers  
<br>
<br>

<p align="center">
  <a href="#build">Build your own POPI</a> •
  <a href="#contribute">Contribute</a> •
  <a href="#alternative">Other open-source quadrupeds</a> •
  <a href="#team">Meet the team</a> •
  <a href="#acknowledgments">Acknowledgments</a>
</p>

<p align="center">
<a target="_blank" rel="noopener noreferrer" href="https://www.youtube.com/watch?v=yvZBiybOPDk" title="Watch the teaser on YouTube"><img src="https://i.imgur.com/u5yCj2k.gif" alt="Watch the teaser on YouTube"/></a>
</p>

## <a name="build"></a> Build your own POPI
We really hope you build your own POPI, and if that's the case and you ever need our help, we'll be happy to answer all your questions. To get you going you will find all the CAD files and mechanical drawings in the [popi_mechanics] folder, along with information about POPI's dimensions, its actuators' specifications and the machines we used to make its parts.

In [popi_electronics] you will learn more about POPI's electrical needs, its wiring maps and the electronic supplies. You can also check the whole bill of materials [here](https://github.com/popi-mkx3/popi_project/blob/master/popi_reports/POPI_bill_of_materials.xlsx).

The source code is entirely based on [ROS]. <b>Even if you don't plan on building POPI, you can still develop your own code or your own walking trajectories and try it on a virtual POPI !</b> The instructions to download the source code and use it are available in the [popi_software] folder. Of course you will also find there more explanations about how it works to help you getting started.
<br>
<br>

## Contribute
In times when [ICRA] organizes workshops called *["Towards real-world deployment of legged robots"](https://www.youtube.com/watch?v=nKZdYw9eXY0)*, and when big shots like MIT and Boston Dynamics [loan](http://news.mit.edu/2019/mit-mini-cheetah-first-four-legged-robot-to-backflip-0304) or [sell](https://spectrum.ieee.org/automaton/robotics/industrial-robots/boston-dynamics-spot-robot-dog-goes-on-sale) their robots, we hope our modest open-source alternative can contribute to spread this technology.

The whole point of making this project fully open-source is to have anyone who is interested contribute to POPI ! Whether it includes documentation translations, new functionalities, bug fixes or code improvements, we'll be glad to receive your pull request.

We're fully aware we still have a long road to go before POPI becomes a more autonomous robot, and we'll be happy to take anyone with us onboard. You can see here the list of [contributors](https://github.com/popi-mkx3/popi_project/graphs/contributors) who participated in this project.
<br>
<br>

## <a name="alternative"></a> Other open-source quadrupeds
Other open-source quadruped robots already exist ! If you are interested, you can check out the [Stanford Doggo Project] or the [Open Dynamic Robot Initiative] which are really cool and more functional at this point in time.

If you know of other advanced open-source robots do not hesitate to contact us, we will be really happy to have a look at other projects.
<br>
<br>

## <a name="team"></a> Meet the team !
POPI was initially designed as part of our mechatronical engineering degree at IMT Mines d'Alès in France. Here is the team who started this project.
If you want to contact us you can send us an email at popi.mkx3@gmail.com.

<br>
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

## Acknowledgments
First we would like to thank deeply [Alexander W. Winkler] and [Dave Coleman] for their respective work on [Towr] and [ROS control boilerplate].  
Then let us thank [TiMOTION] who gave us some precious advice along with our knee actuators. Without their contribution, POPI wouldn't be able to lift his legs !  
Finally we of course want to express our sincere gratitude to all our teachers from [IMT Mines d'Alès] who gave us the opportunity to undertake this project and were there to lend us a hand when we needed it the most.
<br>
<br>

<img src="https://i.imgur.com/h6RkNK1.jpg" height="135"/> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <img src="https://i.imgur.com/MZJbr31.png" height="135"/> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <img src="https://i.imgur.com/P2nOGKx.jpg" height="135"/>

[popi_reports]: https://github.com/popi-mkx3/popi_project/tree/master/popi_reports
[popi_mechanics]: https://github.com/popi-mkx3/popi_project/tree/master/popi_mechanics
[popi_electronics]: https://github.com/popi-mkx3/popi_project/tree/master/popi_electronics
[popi_software]: https://github.com/popi-mkx3/popi_project/tree/master/popi_software

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
[ICRA]: https://www.icra2020.org/
[Stanford Doggo Project]: https://github.com/Nate711/StanfordDoggoProject/blob/master/README.md
[Open Dynamic Robot Initiative]: https://open-dynamic-robot-initiative.github.io/
[YouTube channel]: https://www.youtube.com/channel/UCaCy-1MX6SoqdtjZH4uU_TA

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
