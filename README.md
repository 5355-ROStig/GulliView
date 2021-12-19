GulliView
=============

Locate AprilTags using the legendary GulliView!

Authors
=============

Original author: Edwin Olson <ebolson@umich.edu>  
C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>

***

Code has been modified for Vision Based Localization of Autonomous
Vehicles in the Gulliver Project at Chalmers University, Sweden

Modification Authors:  
Andrew Soderberg-Rivkin <sandrew@student.chalmers.se>  
Sanjana Hangal <sanjana@student.chalmers.se>


***

Helper files and other functions have also been merged with the original
project to help with certain functionalities:

Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

More information can be found at: https://code.google.com/p/cv2cg/

***

Code has been further modified in "DAT295/DIT669 Autonomous and cooperative
vehicular systems" at Chalmers University of Technology, Sweden. Now depends
on the original AprilTag library instead of being a fork of the C++ AprilTag
port, achieving performance gains.

Modification Authors:  
Oskar Fredriksson <oskfre@student.chalmers.se>  
Martin Hilgendorf <marhilg@student.chalmers.se>  
William Johnsson <wiljohn@student.chalmers.se>

Requirements
============

Tested on Debian 12 and Ubuntu. Make sure that dependencies are installed:

    sudo apt update && sudo apt install cmake libopencv-dev libboost-all-dev

We also need to install AprilTag:

    git clone https://github.com/AprilRobotics/apriltag.git
    cd apriltag
    cmake .
    sudo make install

Building
========

To compile the code, 

    git clone https://github.com/5355-ROStig/GulliView.git
    cd GulliView
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

Running GulliView
=================

The binary is located in the build directory. Command line
options can be seen by running `./GulliView -h`. For running GulliView in
the eg5355 lab on all cameras, there is a script `startCamerasBroadcast.sh`
in the build directory.

It's not working :(
=================

* Has the right tag family been provided using the `-t` flag?
* Make sure the user running the program is in the video group:  
  `sudo useradd <user> video`
* If GulliView is not running with the `-n` (nogui) flag, X forwarding 
needs to be enabled if connected via SSH.
* The camera IDs presented by the OS may have changed. Try changing the 
command line option for the camera device (`-d <device>`).
* You may have trouble if multiple cameras are connected via the same USB hub.
* Using a powered USB hub instead of the on-board ports may work sometimes.
