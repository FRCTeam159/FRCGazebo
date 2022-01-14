# FRCGazebo
<em><b>Gazebo Simulation support on Windows</b></em>

<h1>Overview</h1>

The Gazebo Simulator (also called Ignition) provides the ability to test robot behavior in an environment that includes real-world emulation of physical properties like gravity, friction, momentum, inertia, collisions etc. In past years this test-bed was supported by First but was dropped more recently, presumbably because it was only fully provided on Linux platforms which were difficult to come by for most FRC teams. 

Recently however, a number of factors have combined to open up the possibility of running Gazebo simulations natively on Windows. Most notably:
1) A simple python installation of gazebo-11 for Windows available from conda-forge
2) wpilib's new "Simulate on Desktop" option which provides a driver-station selection box that can be used to switch between disabled,auto and teleop modes

<h2>Hardware Requirements</h2>
Testing has only been done on a single Windows laptop (a Dell Inspison 7000 series with an Nvidia GTX 960M graphics card) but any similarly equiped laptop or desktop computer would probably be adequate.

<h2>Software Requirements</h2>
Support is currenly provided for VSCode projects written in the Java programming lanquage targeting the 2022.1.1 FRC release

<h2>Gazebo Installation</h2>

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
   

<h2>FRCGazebo Installation</h2>

This Git repository contains a number of tools that should allow you to connect a robot program to Gazebo in simulation mode. It should be noted however that this is just one approcah that can be followed and no promises are made that it will work as well for you withoutt some tweeking (as very limited testing has been done as of this writing !)

In any case, to Install these tools just clone the current repository into a Windows folder of your choosing by entering the following in a bash shell:

$ git clone https://github.com/FRCTeam159/FRCGazebo

<h3> configure a "bash" environment to use the libraries, models and other utilities provided</h3>
   Create a ".bashrc" file (text) in your home directory that contains environment variables etc. that will allow you to more easilly access the models and plugins provided by this project. 
<pre>
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
Note: set "MY_GAZEBO" to the directory that this repo was cloned to and "GAZEBO_LIBRARY" 
      to where your miniconda3 was installed
</pre>
<h3> Project Directory Structure </h3>
<h4>FRCGazebo project (git) root</h4>
<dl>
<dt>bin</dt><dd>executables</dd> 
<dt>docs</dt><dd>documents</dd>
<dt>gzsim</dt><dd>build and test</dd>
<dt>models</dt><dd>gazebo models</dd>
<dt>plugins</dt><dd>gazebo plugins (build by project)</dd>
<dt>worlds</dt><dd>gazebo worlds</dd>
<dt>wpilib</dt><dd>wpilib c++ includes and libraries</dd>
</dl>

<h2> Testing </h2>
As of this writing testing and Gazebo support has only been provided for FRC robot programs that use the Java programming lanquage. However, migration to c++ applications should be fairly straightforward since most of the communication is done though NetworkTables which is supported in both environments. A number of Java VSCode projects can be found in the gzsim/tests directory but only one of these will be chosen as a testing example
<h3>SimDriveTest</h3>
This project uses a simple command-based program to demonstate Gazebo teleop and autonomous operation in simulation mode. The robot model is an 8-wheel differential drive "tankbot" which can be deployed in an "empty world", the current 2022 "RapidReact" arena or the 2018 "PowerUp" field. In addition to basic Gazebo simulation the project also demonstates the use of First's new odometry and kinematics feature, and trajectory generation and following in autonomous mode.

<h2> Implementation Details
<h3> Interfacing a FRC Robot application to Gazebo (general issues)</h3>

With Gazebo running natively it's possible to develop a communications interface to other Windows applications (such as a robot program) using a variety of approaches, although there are some problems that may be incountered that were not seen in Linux. As a head's up I've itemized some of these issues which largely dictated the scheme that was eventually followed here (which will be briefly descibed below)

<h4> (I)Protobuf Messaging</h3> Gazebo uses a method to communicate between the main application and dynamically loaded libraries (called plugins) that's based on Google "protobuf" protocol. When Gazebo was supported by First in the past a set of custom Message types were used for this which require them to be "registered" with the protobuf manager service before they can be used to transfer data to and from Gazebo. It's still possible to compile these message types using c++ code created by an application called "protoc" but at least in the environment currently provided by the python installation when libraries are built using them a number of problems were observed:

1) After the first such plugin is encountered in a Gazebo model's xml-based "sdf" file any othe plugin (only of a different type) that is included fails to load and an error is generated.
2) Even without this error (e.g. only a single plugin is used) a client application that tries to communicate to the plugin will get another fatal error that the contained message type couldn't be registered with the protobuf service (according to comments on the web this is a known issue that may or may not have been fixed in more recent libraries)

One solution to get around these problems is to only use the built in message types that are supported by the main application. Since these are already compiled in (and not dynamically loaded) they don't seem to require registration with the protobuf service and don't generate these errors when used in custom built plugins

<h5>(II) Java JNI</h3>An attempt was made to use Java's JNI protocol to communicate directly between a Java program and an application built with c++ that made calls using the Gazebo library API but many of those functions genereated a "library not found" error (possibly for a dependent library) at runtime. These kinds of failures are very difficult to track down since no clue is provided on what library caused the error (I suspect it may be zlib.ll since that is one library that is present in the bin directory used by java jni and the Library/bin directory installed by anaconda).

The work-around adopted was to use Wpilib's "network tables" API to pass data between the robot application and a c++ based Gazebo interface application, which contains direct calls the the Gazebo library routines. (Looking back this approach is probably better anyway for a variety of reasons) 

<h4>(III) Gazebo Cameras</h3>Gazebo camera sensors and Mjpeg servers aren't well supported in Windows.
The version of Gazebo supplied by conda-forge was apparently build with support for only the ".png" image format. This causes the "save snapshot" and "record movie" features to fail in the main application and also when using the camera sensor class in the robot's sdf file (with the save to file option enabled).

It is possible get around this problem and to generate the expected ".jpg" files using the "freeimage" library (present in the anaconda environment) but the general purpose "mjpeg-streamer" application that's available in Linux isn't supported in a useful form in Windows. An "MJPEG" server is used to publish a video stream or a set of jpeg images to an httml client such as a web page or an "mjpeg video window" in First's smartdashboard application

The solution to this problem was to use Wpilib's Camera Server API (cscore) instead. This has nice advantage in that it allows images to be sent directly from memory to a cs::MjpegServer class without having to be written out to an intermediate jpeg file at all.
   
<h3> Project Implentation Method</h3>

<h2> Building This Project
