/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_PRESSURESENSORSARRAYDRIVER_H
#define GAZEBOYARP_PRESSURESENSORSARRAYDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

#include <ignition/math.hh>

namespace yarp {
    namespace dev {
        class GazeboYarpPressureSensorsArrayDriver;
    }
}

/**
 * YARP Device Driver for an array of pressure sensors.
 *
 * It is designed for pressure sensors that are represented in the SDF model as fixed joints.
 * The jointNames is the list of the fixed joints that represent the pressure sensors.
 * The sensorDirection array, expressed in the child link frame, is the direction of the measured force.
 * More in detail, the force that the child applies on its parent expressed in the child orientation frame
 * is obtained from the physics engine, and then is projected on the provided axis.
 *
 *
 * Parameters accepted in the config argument of the open method:
 * | Parameter name  | Type   | Units | Default Value | Required | Description | Notes |
 * |:---------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
 * | jointNames      | vector of strings | - |   -    | Yes      | Ordered list of the joints representing a pressure sensor. |  -  |
 * | sensorDirections | vector of vectors of 3 doubles | - | - | Yes | Direction of the measured pressure. | - |
 */
class yarp::dev::GazeboYarpPressureSensorsArrayDriver:
        public yarp::dev::IAnalogSensor,
        public yarp::dev::IPreciselyTimed,
        public yarp::dev::DeviceDriver
{
public:

    GazeboYarpPressureSensorsArrayDriver();

    virtual ~GazeboYarpPressureSensorsArrayDriver();

    /**
     * Gazebo stuff
     */
    bool gazebo_init();
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    /**
     * Yarp interfaces implementation
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ANALOG SENSOR
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int ch);

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();

private:
    bool configure(yarp::os::Property & pluginParameters);

    gazebo::physics::Model* m_robot;
    std::vector<gazebo::physics::Joint *> m_joint_ptrs;
    std::vector<ignition::math::Vector3d> m_sensorDirections;

    /**
     * Buffer of joint sensors data.
     */
    yarp::sig::Vector m_jointsensors_data;

    /**
     * YARP timestamp of the data.
     */
    yarp::os::Stamp m_stamp;

    /**
     * Mutex protecting the buffer of joint sensors data and the timestamp.
     */
    yarp::os::Mutex m_dataMutex;

    /**
     * Connection to the Gazebo physics engine update.
     */
    gazebo::event::ConnectionPtr m_updateConnection;


};

#endif 