rmc_software 
============

Core software repository for Chicago EDT Robotics NASA Robotic Mining Competition.

Instructions
------------

Create a fresh workspace for RMC, called rmc_ws
`cd ~/ && mkdir -p rmc_ws/src && cd rmc_ws/src `

(Core Components) `git clone https://github.com/chicagoedt/rmc_software.git`   

(Simulator) `git clone https://github.com/chicagoedt/rmc_simulation.git`   

rosdep is used to make sure various dependencies that ROS packages have specified in each of their own package.xml files are installed on your machine. If rosdep cant find them installed on your system, it will install them for you.
`rosdep update`
`rosdep install --from-paths ./ --ignore-src --rosdistro indigo -y`

Go to the catkin workspace
`cd ~/rmc_ws`

Lets build it
`catkin_make`

If you get any "cannot locate/find package 'foo' ", you can manually install it 

6) roslaunch rmc_simulation gazebo.launch       <--- Try this to get gazebo up and running with RMC model
launch files to run all the system:
1. radxa-1:
    * rmc_2dnav control.launch
2. radxa-2:
    * teensy launch file thingie
3. jetson-3:
    * vision pointgrey

Qt Mission Control 
------------------
1) In order to build Mission Control Qt application please go to
    http://www.qt.io/download-open-source/ and download Qt 5.5 or later

2. After starting QT Creator open mission_control.pro project file.
    This will create mission_control.pro.user file. DO not chekc this file.
    This file is your local machine settings.

