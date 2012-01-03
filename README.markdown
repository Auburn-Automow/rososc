ROS Stack for interacting with Open Sound Control Hardware and Software                                                                                  

Includes:                                                                            
*  pytouchosc - A library for interacting with TouchOSC layout files                 
*  osc_bridge - A bridge between OSC and ROS                                         
*  touchosc_bridge - A bridge between TouchOSC iOS app and ROS                       
*  teleop_handler - A tabpage handler for robot teleoperation                        
*  diagnostics_handler - A tabpage handler for diagnostics information

For more information on TouchOSC: [hexler.net](http://hexler.net/software/touchosc)

For more information on ROSOSC: [ROS.org wiki: ROSOSC](http://ros.org/wiki/rososc)

For ROSOSC Tutorials: [ROS.org wiki: ROSOSC_tutorials](http://ros.org/wiki/rososc_tutorials)

For more information on ROS: [ROS.org](http://ros.org)

Installation Notes:

On Ubuntu

    sudo apt-get install python-pip
    rosmake --rosdep-install rososc
    
On Mac, if you followed the homebrew guide, pip is already installed:

    rosmake --rosdep-install rososc