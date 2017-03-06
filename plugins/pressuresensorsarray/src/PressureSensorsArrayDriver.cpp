/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "PressureSensorsArrayDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;


GazeboYarpPressureSensorsArrayDriver::GazeboYarpPressureSensorsArrayDriver()
{}


GazeboYarpPressureSensorsArrayDriver::~GazeboYarpPressureSensorsArrayDriver()
{}

void GazeboYarpPressureSensorsArrayDriver::onUpdate(const gazebo::common::UpdateInfo & _info)
{
    yarp::os::LockGuard guard(m_dataMutex);

    m_stamp.update(_info.simTime.Double());
    for ( unsigned int jnt_cnt=0; jnt_cnt < m_joint_ptrs.size(); jnt_cnt++ )
    {
        // Read force/torque from the joint
        gazebo::physics::JointWrench jntWrench = m_joint_ptrs[jnt_cnt]->GetForceTorque(0);

        // Project force on the axis
        m_jointsensors_data[jnt_cnt] = m_sensorDirections[jnt_cnt].Dot(jntWrench.body1Force);
    }

    return;
}

//DEVICE DRIVER
bool GazeboYarpPressureSensorsArrayDriver::open(yarp::os::Searchable& config)
{
    yarp::os::Property pluginParameters;
    pluginParameters.fromString(config.toString().c_str());

    std::string robotName (pluginParameters.find("robotScopedName").asString().c_str());

    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(m_robot == NULL)
    {
        yError() << "GazeboYarpPressureSensorsArrayDriver error: robot was not found";
        return false;
    }

    if( !configure(pluginParameters) )
    {
        return false;
    }

    yarp::os::LockGuard guard(m_dataMutex);

    m_jointsensors_data.resize(m_joint_ptrs.size());
    m_jointsensors_data.zero();


    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin (
            boost::bind ( &GazeboYarpPressureSensorsArrayDriver::onUpdate, this, _1 ) );

    return true;
}

bool GazeboYarpPressureSensorsArrayDriver::configure(yarp::os::Property & pluginParameters)
{
    //////////////////////
    // Parse jointNames
    //////////////////////

    yarp::os::Bottle *propJointNames=pluginParameters.find("jointNames").asList();
    if(propJointNames==0)
    {
        yError() <<"GazeboYarpPressureSensorsArrayDriver: Error parsing parameters: \"jointNames\" should be followed by a list\n";
        return false;
    }

    std::vector< std::string > jointNames;
    jointNames.resize(propJointNames->size());
    for(int ax=0; ax < propJointNames->size(); ax++)
    {
        jointNames[ax] = propJointNames->get(ax).asString().c_str();
    }

    m_joint_ptrs.resize(jointNames.size());

    const gazebo::physics::Joint_V & gazebo_models_joints = m_robot->GetJoints();

    for(unsigned int i=0; i < jointNames.size(); i++ ) {
        bool joint_found = false;
        std::string controlboard_joint_name = jointNames[i];
        std::string  controlboard_joint_name_scoped_ending = "::" + controlboard_joint_name;

        for(unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++ ) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetScopedName();
            if( GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_joint_name_scoped_ending) ) {
                joint_found = true;
                m_joint_ptrs[i] = boost::get_pointer(gazebo_models_joints[gazebo_joint]);
            }
        }

        if( !joint_found ) {
            yError() << "GazeboYarpPressureSensorsArrayDriver::setJointPointers(): Error, cannot find joint " << controlboard_joint_name;
            return false;
        }
    }

    //////////////////////
    // Parse sensorDirections
    //////////////////////

    yarp::os::Bottle *propSensorDirections= pluginParameters.find("sensorDirections").asList();
    if (propSensorDirections==0)
    {
        yError() <<"GazeboYarpPressureSensorsArrayDriver: Error parsing parameters: \"sensorDirections\" should be followed by a list of 3d vectors\n";
        return false;
    }

    if (propSensorDirections->size() != jointNames.size())
    {
        yError() <<"GazeboYarpPressureSensorsArrayDriver: Size mismatch between \"sensorDirections\" and \"jointNames\".\n";
        return false;
    }

    m_sensorDirections.resize(propSensorDirections->size());
    for (int ax=0; ax < propSensorDirections->size(); ax++)
    {
        yarp::os::Bottle *prop3dVec= propSensorDirections->get(ax).asList();

        if (prop3dVec==0 || prop3dVec->size() != 3)
        {
            yError() <<"GazeboYarpPressureSensorsArrayDriver: Error parsing parameters: all elements of the \"sensorDirections\" list should be 3d vectors.\n";
            return false;
        }

        m_sensorDirections[ax].X() = prop3dVec->get(0).asDouble();
        m_sensorDirections[ax].Y() = prop3dVec->get(1).asDouble();
        m_sensorDirections[ax].Z() = prop3dVec->get(2).asDouble();
    }

    return true;
}

bool GazeboYarpPressureSensorsArrayDriver::close()
{
    yarp::os::LockGuard guard(m_dataMutex);

    this->m_updateConnection.reset();

    return true;
}

//ANALOG SENSOR
int GazeboYarpPressureSensorsArrayDriver::read(yarp::sig::Vector &out)
{
    yarp::os::LockGuard guard(m_dataMutex);

    if ( (int)out.size() != m_jointsensors_data.size() ) {
        yWarning() << " GazeboYarpPressureSensorsArrayDriver:read() warning : resizing input vector, this can probably be avoided" ;
        out.resize(m_jointsensors_data.size());
    }

    out = m_jointsensors_data;

    return AS_OK;
}

int GazeboYarpPressureSensorsArrayDriver::getChannels()
{
    return (int)m_joint_ptrs.size();
}

int GazeboYarpPressureSensorsArrayDriver::getState(int /*ch*/)
{
    return AS_OK;
}

int GazeboYarpPressureSensorsArrayDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpPressureSensorsArrayDriver::calibrateSensor(const yarp::sig::Vector& /*value*/)
{
    return AS_OK;
}

int GazeboYarpPressureSensorsArrayDriver::calibrateChannel(int /*ch*/)
{
    return AS_OK;
}

int GazeboYarpPressureSensorsArrayDriver::calibrateChannel(int /*ch*/, double /*v*/)
{
    return AS_OK;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpPressureSensorsArrayDriver::getLastInputStamp()
{
    yarp::os::LockGuard guard(m_dataMutex);

    return m_stamp;
}
