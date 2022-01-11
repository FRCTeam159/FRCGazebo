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
   $ conda install gazebo -c conda-forge
   (The installation usually takes about 10-15 minutes)
2) That's it ! to test just enter the following in the bash shell

   gazebo --verbose
   
   If all goes well you should see the Gazebo application on the desktop displaying an empty world
   
Interfacing an application to Gazebo (problems and solutions)

With Gazebo running natively it's possible to develop a communications interface to other Windows applications (such as a robot program) using a variety of approaches, although there are some problems that may be incountered that were not seen in Linux. As a head's up I've itemized some of these issues which largely dictated the scheme that was followed here (which will be briefly descibed below)

(I) Gazebo uses a method to communicate between the main application and dynamically loaded libraries (called plugins) that's based on Google "protobuf" protocol. When Gazebo was supported by First in the past a set of custom Message types were used for this which require them to be "registered" with the protobuf manager service before they can be used to transfer data to and from Gazebo. It's still possible to compile these message types using c++ code created by an application called "protoc" but at least in the environment currently provided by the python installation when libraries are built using them a number of problems were observed:

1) After the first such plugin is encountered in a Gazebo model's xml-based "sdf" file any othe plugin (only of a different type) that is included fails to load and an error is generated.
2) Even without this error (e.g. only a single plugin is used) a client application that tries to communicate to the plugin will get another fatal error that the contained message type couldn't be registered with the protobuf service (according to comments on the web this is a known issue that may or may not have been fixed in more recent libraries)

One solution to get around these problems is to only use the built in message types that are supported by the main application. Since these are already compiled in (and not dynamically loaded) they don't seem to require registration with the protobuf service and don't generate these errors when used in custom built plugins

(II) An attempt was made to use Java's JNI protocol to communicate directly between a Java program and an application built with c++ that made calls using the Gazebo library API but many of those functions genereated a "library not found" error (possibly for a dependent library) at runtime. These kinds of failures are very difficult to track down since no clue is provided on what library caused the error (I suspect it may be zlib.ll since that is one library that is present in the bin directory used by java jni and the Library/bin directory installed by anaconda).

The work-around adopted was to use Wpilib's "network tables" API to pass data between the robot application and a c++ based Gazebo interface application, which contains direct calls the the Gazebo library routines. (Looking back this approach is probably better anyway for a variety of reasons) 

(III) 

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

