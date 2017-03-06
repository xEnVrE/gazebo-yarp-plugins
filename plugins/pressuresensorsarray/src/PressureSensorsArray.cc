/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "PressureSensorsArrayDriver.h"
#include "PressureSensorsArray.hh"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpPressureSensorsArray)

namespace gazebo {

    GazeboYarpPressureSensorsArray::GazeboYarpPressureSensorsArray() : ModelPlugin(), m_imultwrapper(0)
    {
    }

    GazeboYarpPressureSensorsArray::~GazeboYarpPressureSensorsArray()
    {
        if (m_imultwrapper) {
            m_imultwrapper->detachAll();
            m_imultwrapper = 0;
        }
        if (m_dev_wrapper.isValid())
            m_dev_wrapper.close();
        if (m_dev_driver.isValid())
            m_dev_driver.close();
        GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_gazeboScopedModelName);
        yarp::os::Network::fini();
    }

    void GazeboYarpPressureSensorsArray::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
            yError() << "GazeboYarpPressureSensorsArray::Load error: yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        if (!_parent) {
            gzerr << "GazeboYarpPressureSensorsArray plugin requires a parent.\n";
            return;
        }

        // Add my gazebo device driver to the factory.
        ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpPressureSensorsArrayDriver>
                                                    ("gazebo_pressuresensorsarray", "analogServer", "GazeboYarpPressureSensorsArray"));

        //Getting .ini configuration file from sdf
        ::yarp::os::Property wrapper_properties;
        ::yarp::os::Property driver_properties;

        bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,driver_properties);

        if (!configuration_loaded) {
            return;
        };

        ///< \todo TODO handle in a better way the parameters that are for the wrapper and the one that are for driver
        wrapper_properties = driver_properties;


        m_gazeboScopedModelName = _parent->GetScopedName();

        //Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));
        driver_properties.put("robotScopedName", m_gazeboScopedModelName.c_str());

        //Open the wrapper
        //Force the wrapper to be of type "analogServer" (it make sense? probably no)
        wrapper_properties.put("device","analogServer");
        if (!m_dev_wrapper.open(wrapper_properties)) {
            yError() << "GazeboYarpPressureSensorsArray Plugin failed: error in opening yarp driver wrapper";
            return;
        }

        //Open the driver
        driver_properties.put("device","gazebo_pressuresensorsarray");
        if (!m_dev_driver.open(driver_properties)) {
            yError() << "GazeboYarpPressureSensorsArray Plugin failed: error in opening yarp driver";
            return;
        }

        //Attach the driver to the wrapper
        ::yarp::dev::PolyDriverList driver_list;

        if (!m_dev_wrapper.view(m_imultwrapper)) {
            yError() << "GazeboYarpPressureSensorsArray : error in loading wrapper";
            return;
        }

        driver_list.push(&m_dev_driver, "dummy");

        if (m_imultwrapper->attachAll(driver_list)) {
        } else {
            yError() << "GazeboYarpPressureSensorsArray : error in connecting wrapper and device ";
        }

    }

}
