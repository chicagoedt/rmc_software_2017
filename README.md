RMC_Software [![Build Status](http://jenkins.chicagoedt.org/job/Software_RMC_Upstream/badge/icon)](http://jenkins.chicagoedt.org/job/Software_RMC_Upstream/) 
============

Software repository for Robotic Mining Competition.

Instructions
------------
1) Fork this repo if you already havent

2)  git clone https://github.com/yourGithubName/Software_RMC.git src   (dont forget the 'src' at the end)

3) cd src

4) source installEDT.bash

5) Your RMC workspace is now setup along with anything else you will need to develop for RMC. 

6) roslaunch rmc_simulation gazebo.launch       <--- Try this to get gazebo up and running with RMC model


==========
launch files to run all the system:
1. radxa-1:
    * rmc_2dnav control.launch
2. radxa-2:
    * teensy launch file thingie
3. jetson-3:
    * vision pointgrey