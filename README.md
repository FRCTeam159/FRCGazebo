# FRCGazebo
<em><b>Gazebo Simulation support on Windows</b></em>

<h1>Overview</h1>

The Gazebo Simulator provides the ability to test robot behavior in an environment that includes real-world emulation of physical properties like gravity, friction, momentum, inertia, collisions etc. In past years this test-bed was supported by First but was dropped more recently, presumbably because it was only fully provided on Linux platforms which were difficult to come by for most FRC teams. 

Recently however, a number of factors have combined to open up the possibility of running Gazebo simulations natively on Windows. Most notably:
1) A simple python installation of gazebo-11 for Windows available from conda-forge
2) wpilib's new "Simulate on Desktop" option which provides a driver-station selection box that can be used to switch between disabled,auto and teleop modes

<h2>Hardware Requirements</h2>
Testing has only been done on a Windows 10 laptop (a Dell Inspison 7000 series with a Nvidia GTX 960M graphics card) and a Windows 11 desktop (2070 Nvidia gpu) but any similarly equiped laptop or desktop computer will probably be adequate.

<h2>Software Requirements</h2>
Run-time support is currenly provided for VSCode projects written in the Java programming lanquage targeting the 2022.1.1 FRC release. In order to build the non-java components of this project you will also need to download and install Visual Studio Community edition 2015-2019 (with c++ support)

<h2>Gazebo Installation</h2>

I. You will need to first download and install the following applications:
1) miniconda3
   https://docs.conda.io/en/latest/miniconda.html
   <ul><li>note: ok to accept the option to add miniconda to path (may get Ogre installation errors otherwise)</li></ul>
   
2) git-bash for Windows
   https://gitforwindows.org/
   
II. Install Gazebo-11 from conda-forge

1) open a git-bash command Window and enter the following:

   $ conda install gazebo -c conda-forge
   (The installation usually takes about 10-15 minutes)
   
2) That's it ! to test just enter the following in the bash shell *

   gazebo --verbose
   
   If all goes well you should see the Gazebo application on the desktop displaying an empty world
   
<h4>* Bug 2/4/22</h4>
 <ul><li>As of at least this date the conda-forge gazebo installation fails to allow gazebo to run (just fails silently with no error message)
   <li> A work-around (until the problem is fixed) is to use a conda environment file to install a known-to-work version which is available in the "docs" directory of this project (base-env.txt)
      <li> After installing miniconda3 to install gazebo in the base directory download the file and the enter from a command shell: conda install --name base --file base-env.txt
         </ul>
This repository contains a number of tools that should allow you to connect a robot program to Gazebo in simulation mode. It should be noted however that this is just one approach that can be followed and no promises are made that it will work for you without at least a bit of tweeking !

In any case, to Install just clone the current repository into a Windows folder of your choosing by entering the following in a bash shell:

$ git clone https://github.com/FRCTeam159/FRCGazebo

<h3> configure a "bash" environment to use the libraries, models and other utilities provided</h3>
   Create a ".bashrc" file (text) in your home directory that contains environment variables etc. that will allow you to more easilly access the models and plugins provided by this project (an example .bashrc file is provided in the "docs" directory)
<pre>
export MY_GAZEBO=$HOME/Robotics/FRCTeam159Repos/FRCGazebo
export GAZEBO_LIBRARY=$HOME/miniconda3/Library
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
<dt>plugins</dt><dd>gazebo plugins (built by project)</dd>
<dt>worlds</dt><dd>gazebo worlds</dd>
<dt>wpilib</dt><dd>wpilib c++ includes and libraries</dd>
</dl>

<h2> Testing </h2>
As of this writing testing and Gazebo support has only been provided for FRC robot programs that use the Java programming lanquage. However, migration to c++ applications should be fairly straightforward since most of the communication is done though NetworkTables which is supported in both environments. A number of Java VSCode projects can be found in the gzsim/tests directory but only one of these will be chosen as a testing example
<h3>SimDriveTest</h3>
This project uses a simple command-based program to demonstate Gazebo teleop and autonomous operation in simulation mode. The robot model is an 8-wheel differential drive "tankbot" which can be deployed in an "empty world", the current 2022 "RapidReact" arena or the 2018 "PowerUp" field. In addition to basic Gazebo simulation the project also demonstates the use of First's new odometry and kinematics feature, and trajectory generation and following in autonomous mode.

<h3>Setup</h3>
<ol>
<li>In vscode, open the SimDriveTest project in gztest/tests
<li>Change the default terminal in vscode to "Git Bash"
  <ul>
  <li>in the lower right corner of screen press the down carot (next to the + sign)
  <li>at The bottom of the popup choose "Select Default Profile"
  <li>select "Git Bash" from list
  </ul>
<li>Open a gitbash terminal
  <ul>
    <li>menu/terminal/new terminal
  </ul>
<li>In the terminal enter: gzrun
   <ul>
   <li> gzrun is a script (text file) where you can select the "world" file used in the simulation (e.g. tank8-2022-field.world)
   <li> world files are located in the worlds folder in the repostory base directory (root)
   <li> after opening Gazebo with the selected world file gzrun starts the program GzProgram.exe (in [root]/bin) that establishes a communication link between NetworkTables and the Gazebo application
   </ul>
<li>Open a second terminal in vscode
   <ul>
   <li>in the terminal enter "simulate"
   <li>"simulate" (in [root]/bin) is just a shortcut for "gradlew simulateJavaRelease" (you can optionally select "Simulate Robot Code" from the (W) pulldown menu)
   <li>simulate starts the robot program in simulation mode
   </ul>
<li>Open "SmartDashboard"
   <ul>
      <li>in the (W) pulldown select "Start Tool"-> SmartDashboard
   </ul>
<li>If all goes well you should see a a screen similar to the one in : docs/ScreenShot-FRCGazebo-windows-laptop.PNG
</ol>
<h3>Running the program</h3>
 <h4> Teleop Mode</h4>
   <ol>
      <li>in the "Robot Simulation" window select "Teleoperated" from the "state" selection list
      <li>you shlould be able to drive the test bot around in the simulator using a standard game controller
      <li>you can reset the initial scene by pressing the reset checkbox in SmartDashboard or "Edit->Reset Model Poses" from the Gazebo menu bar
   </ol>
 <h4>Autonomous Mode</h4>
   <ol>
     <li>you can choose a pre-programmed test path from the selection list on the right side of the smartdashboard window (Scurve, Hooked etc.)
     <li>these paths are defined in the "Trajectories.java" file in the java project's subsystem directory
     <li>before running an autonomous routine press the "reset" checkbox to reset the model poses"
     <li>to start a run choose "Autonomous" from the robot state panel in the Robot Simulation window
   </ol>
   <h4>Trajectory Plot options</h4>
   <ol>
   <li> In a command shell start the plot server in the background by executing the script ("start_plot_server") located in the project bin directory
   <li> to plot certain features of an autonomous run choose one on the options from the selections on the right side of the smartdashboard window (Plot Distance etc.)
   <li> plotting support and other utilities is provided by the java files in [root]/gztest/java/utils directory.
   </ol>
 <h4>Recording a video</h4>
   <ol>
   <li>the display window showing in smartdashboard is generated by a MJPEGserver running from a camera plugin in the gazebo world file
   <li>the contents of this window can be captured into a video file (avi format) by pressing the "record" checkbox in the smartdashboard window
   <li>by default the video file is written to test_0.avi in a directory called "tmp" in the java project directory (note: make this directory first if it doesn't yet exist)
   <li>stop the recording by unchecking the record checkbox in smartdashboard
   <li>you can generate multiple videos by repeating the record check->uncheck cycle (subsequent videos are created in tmp/test_1.avi, tmp/test_2.avi etc.)
   </ol>

<h2> Implementation Details
<h3> Interfacing a FRC Robot application to Gazebo (general issues)</h3>

With Gazebo running natively it's possible to develop a communications interface to other Windows applications (such as a robot program) using a variety of approaches, although there are some problems that may be incountered that were not seen in Linux. As a head's up I've itemized some of these issues which largely dictated the scheme that was eventually followed here (which will be briefly descibed below)

<h4> (I) Protobuf Messaging</h3> Gazebo uses a method to communicate between the main application and dynamically loaded libraries (called plugins) that's based on Google "protobuf" protocol. When Gazebo was supported by First in the past a set of custom Message types were used for this which require them to be "registered" with the protobuf manager service before they can be used to transfer data to and from Gazebo. It's still possible to compile these message types using c++ code created by an application called "protoc" but at least in the environment currently provided by the current python installation when libraries are built using them a number of problems were observed:

1) After the first such plugin is encountered in a Gazebo model's xml-based "sdf" file any othe plugin (but only of a different type) that is included fails to load and an error is generated.
2) Even without this error (e.g. only a single plugin type is used) a client application that tries to communicate to the plugin will get another fatal error that the contained message type couldn't be registered with the protobuf service (according to comments on the web this is a known issue that may or may not have been fixed in more recent libraries)

One solution to get around these problems is to only use the built in message types that are supported by the main application. Since these are already compiled in (and not dynamically loaded) they don't seem to require registration with the protobuf service and don't generate these errors when used in custom built plugins

<h5>(II) Java JNI</h3>
An attempt was made to use Java's JNI protocol to communicate directly between a Java program and an application built with c++ that made calls using the Gazebo library but many of those functions generated a "library not found" error (probably for a dependent library) at runtime. These kinds of failures are very difficult to track down since no clue is provided on what library caused the error (note: I suspect it may be zlib.dll since that library is present in both the bin directory used by the java jni and the Library/bin directory installed by anaconda - but totally different file sizes).

The work-around adopted was to use Wpilib's "NetworkTables" API to pass data between the robot application and a c++ based Gazebo interface application, which contains direct calls the the Gazebo library routines. (Looking back this approach is probably better anyway for a variety of reasons) 

<h4>(III) Gazebo Cameras</h3>Gazebo camera sensors and Mjpeg servers aren't well supported in Windows.
The version of Gazebo supplied by conda-forge was apparently build with support for only the ".png" image format. This causes the "save snapshot" and "record movie" features to fail in the main application and also when using the camera sensor class in the robot's sdf file (with the save to file option enabled).

It's possible to get around this problem and to generate the expected ".jpg" files using the "freeimage.dll" library (present in the anaconda environment) but the general purpose "mjpeg-streamer" application that's available in Linux isn't supported in a useful form in Windows.(note: An "MJPEG" server is used to publish a video stream or a set of jpeg images to an httml client such as a web page or an "mjpeg video window" in First's smartdashboard application)

The solution to this problem was to use Wpilib's Camera Server API (cscore) instead. This has a nice advantage in that it allows images to be sent directly from memory to a cs::MjpegServer class without having to be written out to an intermediate jpeg file at all.
   
<h3> Project Implementation Details [TODO] </h3>

<h2> Building This Project [TODO]
