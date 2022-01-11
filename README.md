# FRCGazebo
Gazebo Simulation support on Windows

Overview

The Gazebo Simulator (also called Ignition) provides the ability to test robot behavior in an environment that includes real-world emulation of physical properties like gravity, friction, momentum, inertia, collisions etc. In past years this test-bed was supported by First but was dropped more recently, presumbably because it was only fully provided on Linux platforms which were difficult to come by for most FRC teams. 

Recently however, a number of factors have combined to open up the possibility of running Gazebo simulations natively on Windows. Most notably:
1) A simple python installation of gazebo-11 for Windows available from conda-forge
2) wpilib's new HAL-based "Simulate" option which (critically) provides a driver-station selection box that can be used to switch between disabled,auto and teleop modes

Gazebo Installation

I. You will need to first download and install the following applications:
1) miniconda3
   https://docs.conda.io/en/latest/miniconda.html
   
2) git-bash for Windows
   https://gitforwindows.org/
   
II. Install Gazebo-11 from conda-forge

1) open a git-bash command Window and enter the following:
2) $ conda install gazebo -c conda-forge
   (The installation usually takes about 10-15 minutes)
3) That's it ! to test just enter the following in the bash shell

   gazebo --verbose
   
   If all goes well you should see the Gazebo application on the desktop displaying an empty world
   
FRCGazebo Installation

This Git repository contains a number of tools that will allow you to connect a robot program to Gazebo in simulation mode.
To Install just clone it into a chosen Windows directory by entering the following in a bash shell

1) git clone https://github.com/FRCTeam159/FRCGazebo
2) Create a ".bashrc" text file that contains environment variables etc. that will allow you to access the models and plugins provided. 
    example:
   
     export MY_GAZEBO=/c/Users/Alpiner/Robotics/FRCTeam159Repos/FRCGazebo
     
     export GAZEBO_LIBRARY=/c/Users/Alpiner/miniconda3/Library
     
     export GAZEBO_TOOLS=$GAZEBO_LIBRARY/share/gazebo-11
     
     export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11345}
     
     export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
     
     export GAZEBO_RESOURCE_PATH=$GAZEBO_TOOLS:$MY_GAZEBO/worlds
     
     export GAZEBO_PLUGIN_PATH=$GAZEBO_TOOLS/plugins:$MY_GAZEBO/plugins
     
     export GAZEBO_MODEL_PATH=$GAZEBO_TOOLS/models:$MY_GAZEBO/models
     
     export OGRE_RESOURCE_PATH=$GAZEBO_LIBRARY/bin
  
     export PATH=.:MY_GAZEBO/bin:$MY_GAZEBO/wpilib/libs:$PATH
  
 3) Set "MY_GAZEBO" to the directory that this repo was cloned to and "GAZEBO_LIBRARY" to where miniconda was installed

