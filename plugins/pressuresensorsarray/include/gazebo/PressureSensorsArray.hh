/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_PRESSURESENSORSARRAY_HH
#define GAZEBOYARP_PRESSURESENSORSARRAY_HH

#include <gazebo/common/Plugin.hh>

#include <string>

#include <yarp/dev/PolyDriverList.h>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
    class GazeboYarpPressureSensorsArray : public ModelPlugin
    {
    public:
        GazeboYarpPressureSensorsArray();
        virtual ~GazeboYarpPressureSensorsArray();

        /**
         * Saves the gazebo pointer, creates the device driver
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
        yarp::dev::PolyDriver m_dev_wrapper;
        yarp::dev::IMultipleWrapper* m_imultwrapper;
        yarp::dev::PolyDriver m_dev_driver;

        /**
         * Scoped name of the Gazebo model containing this plugin.
         */
        std::string m_gazeboScopedModelName;
    };

}

#endif
