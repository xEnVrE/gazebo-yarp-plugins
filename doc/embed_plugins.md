# Use the gazebo-yarp-plugins in Gazebo Models {#embed_plugins}

# Use the gazebo-yarp-plugins in Gazebo Models

To understand the structure of gazebo-yarp-plugins, it is useful to understand what Gazebo plugins and Yarp device drivers are.

In a nutshell, Gazebo plugins are C++ classes that extend the functionalities of the Gazebo simulator, while Yarp device drivers are classes used in Yarp for abstracting the functionality of devices used in robots.

For additional information it is possible to access the official documentation for both [Gazebo plugins](http://gazebosim.org/tutorials?cat=write_plugin) and [YARP Device Drivers](http://wiki.icub.org/yarpdoc/note_devices.html).

The `gazebo-yarp-plugins` consists of:
* gazebo plugins that instantiate yarp device drivers and
* yarp device drivers that wrap gazebo functionalities inside the yarp device interfaces . 

The plugins/devices currently implemented are:

|  Functionality     | Plugin Name  |  Gazebo Plugin class  | YARP Device class  |
| :----------------: |:-------------:| :-----:|:---------------------------------:|
| Control Board (encoder readings, motor control, ...)  | GazeboYarpControBoard | GazeboYarpControBoardDriver |
| 6-Axis Force Torque sensor| GazeboYarpForceTorque |  GazeboYarpForceTorqueDriver |
| Inertial Measurement Unit | GazeboYarpIMU | GazeboYarpIMUDriver |
| Network of Distributed  Inertial Measurement Units |   | 


## Using the plugins in Gazebo Models
In Gazebo, the simulated models are described using the [SDF (simulation description format)](http://gazebosim.org/sdf.html), an XML-based file format that can be easily produced from URDF files (the description format used in the ROS project). The plugins are included in the simulated model using the "plugin" tag, by specifying the name of the plugin shared object. Configuration of the plugin is provided trough the child element of the "plugin" tag, as in this example for the controlboard of CoMan's torso:

     <plugin name="coman_torso_controlboard" filename="libgazebo_yarp_controlboard.so">
        <yarpConfigurationFile>model://coman_urdf/conf/coman_gazebo_torso.ini</yarpConfigurationFile>
     </plugin>

The coman_gazebo_torso.ini is a Yarp configuration file that mimics the structure of the configuration file for the real control boards: https://github.com/EnricoMingo/iit-coman-ros-pkg/blob/master/coman_gazebo/sdf/conf/coman_gazebo_torso.ini