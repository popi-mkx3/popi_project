# popi_electronics

<img align="center" src="https://i.imgur.com/HhxkuNx.jpg" width="100%"/>
<br>
<br>

You will find here everything you need to understand all the electronics necessary to make POPI walk. Hence, all the [wiring diagrams] and the [pin planning] of our onboard-computers are available in this folder. These files unfortunately are in French for now, but the diagrams should still speak to anyone. Let us know if you ever need a translation and we'll do our best ! To get an overview of POPI's electronic design, please read the following information.

#### What is available
:heavy_check_mark: Design made with [easyEDA]  
:heavy_check_mark: Creation of a custom [encoder]  
:heavy_check_mark: Creation of a custom [current sensor PCB]  
:heavy_check_mark: Mapping of all the card to respect the symmetry  
:heavy_check_mark: PCB manufacturing on a PCB milling machine
<br>
<br>

<p align="center">
  <a href="#general-description">General description</a> •
  <a href="#onboard-computers">Onboard computers</a> •
  <a href="#sensors">Sensors</a> •
  <a href="#power-supply">Power supply</a> •
  <a href="#lets-look-further">Let's look further</a> •
  <a href="#contribute">Contribute</a> •
  <a href="#team">Meet the team</a>
</p>

<p align="center">
	<img src="https://i.imgur.com/C25Hcgm.gif" width="70%" />
</p>

## <a name="general-description"></a> General description
Most of the electronics of the robot is located on the main electronic board underneath the plexiglass carter. All the parts are spread and layered evenly to ease the accessibility and the maintenance. As you can see below, the main electronic board is divided in 3 areas : computing, commanding and supplying.  
<p align="center">
	<img src="https://i.imgur.com/N276beb.png" width="80%" />
</p>

All the details of the parts is given in the [electronic nomenclature] and a link to a retailer can be found in the [bills of material] in the [popi_report] repository. 
<br>
<br>

## <a name="onboard-computers"></a> Onboard computers
POPI is embedded with 3 onboard computers. The main one is a [Raspberry Pi 3B+] and is used to host the controllers and compute the set point of each joint. It communicates with two [BeagleBone Black] (BBB) computers, each one managing the sensors and actuators of two legs (right legs for one BBB and left legs for the other). 

<p align="center">
	<img src="https://i.imgur.com/J5PzmXd.png" width="100%" /> 
</p>
<br>
<br>

## <a name="sensors"></a> Sensors
In order to know the position of each joint, POPI is embedded with two kind of sensors. The knee and hip rolling joints positions are measured by potentiometers. For the hip pitching joints, we designed a custom [encoder] and its control PCB whose plan is available in this folder. 
<p align="center">
	<img src="https://i.imgur.com/G7UEYgz.png" width="60%"/> 
</p>

We also designed a [current sensor PCB] to be able to know the consumption of the motors. All the details of this part are given in here.  
<br>
<br>
 
## <a name="power-supply"></a> Power supply
POPI is actually powered by two 24V power supplies that are located outside the robot on a [power supply board] we designed. Thus, POPI must be hooked up with an umbilicus cable that will provide the power. Details about this power supply board are given in the [user manual]. 
<p align="center">
	<img src="https://i.imgur.com/L8llYIF.png" width="70%"/> 
</p>

## <a name="lets-look-further"></a> Let's look further
This design remains a proof of concept, designed and built in a few months by students relatively new to this field. We now have to improve what we've done and take into account the problems we encountered to achieve a truly functional V2.
  
#### Thoughts for a future version:
* Try looking for more power efficient motors to reduce the size of the ombilicus cable and even maybe to design an onboard battery. 
* Be more careful of ECM influence on the quality of the sensor signals. Try having all the high power beneath the main electronic board and isolate better the two zones. Think about shielding the important data cables. 
* Try having always absolute sensors or at least add limit switches to the system. 
* Unfortunately we had not enough analog input to receive all the current sensor data in the same time. Try adding a multiplexer or an ADC on the current sensor PCB. 
* We had to add 100nF capacitors just before the BeagleBone to filter the analog signal of our potentiometers. Try designing a proper integration board for those capacitors.
<br>
<br>

## Contribute
The whole point of making this project fully open-source is to have anyone who is interested contribute to POPI ! Whether it includes documentation translations, new functionalities, bug fixes or code improvements, we'll be glad to receive your pull request.

We're fully aware we still have a long road to go before POPI becomes a more autonomous robot, and we'll be happy to take anyone with us onboard. You can see here the list of [contributors](https://github.com/popi-mkx3/popi_project/graphs/contributors) who participated in this project.
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


[wiring diagrams]: https://github.com/popi-mkx3/popi_project/blob/master/popi_electronics/POPI_PlanElec_General.pdf
[pin planning]: https://github.com/popi-mkx3/popi_project/blob/master/popi_electronics/POPI_PinPlanning.xlsx
[easyEDA]: https://easyeda.com/fr
[encoder]:https://github.com/popi-mkx3/popi_project/blob/master/popi_electronics/POPI_PlanElec_Carte_roue_codeuse.pdf
[current sensor PCB]: https://github.com/popi-mkx3/popi_project/blob/master/popi_electronics/POPI_PlanElec_Capteur_courant.pdf
[electronic nomenclature]: https://github.com/popi-mkx3/popi_project/blob/master/popi_electronics/Nomenclature.xlsx
[bills of material]: https://github.com/popi-mkx3/popi_project/blob/master/popi_reports/POPI_bill_of_materials.xlsx
[Raspberry Pi 3B+]: https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/
[BeagleBone Black]: https://beagleboard.org/black
[power supply board]: https://github.com/popi-mkx3/popi_project/blob/master/popi_electronics/Schematic_POPI-Platine-dalimentation_Sheet_1.pdf
[user manual]: https://github.com/popi-mkx3/popi_project/blob/master/popi_reports/POPI_user_manual.pdf
[popi_reports]: https://github.com/popi-mkx3/popi_project/tree/master/popi_reports


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