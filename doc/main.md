gazebo-yarp-plugins {#main_page}
=====================

[TOC]

gazebo-yarp-plugins
=====================

The gazebo-yarp-plugins are a set of Gazebo plugins that expose YARP interfaces 
for robot simulated in Gazebo. The main design objective for the gazebo-yarp-plugins 
is to provide the exact same interfaces that can be found in real world YARP-powered 
robot in the Gazebo simulation, to ensure that YARP-based software can run with no modifications
both in simulation and on the real robots. 

Reference Documentation
-----------------------
\ref installation
\ref embed_plugins
\ref troubleshooting
\ref contributing

Full Plugins List
----------------
- [`Camera`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/camera)
- [`Clock`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/clock) allows to synchronize the simulation with an external controller.
- [`Controlboard`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/controlboard)
- [`External Wrench`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/externalwrench)
- [`Force Torque`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/forcetorque)
- [`IMU`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/imu)
- [`Joint Sensors`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/jointsensors)
- [`Show Model CoM`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/showmodelcom)
- [`World Interface`](https://github.com/robotology/gazebo-yarp-plugins/tree/master/plugins/worldinterface)