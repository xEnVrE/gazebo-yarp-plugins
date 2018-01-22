/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_FAKEPOINTCLOUD_HH
#define GAZEBOYARP_FAKEPOINTCLOUD_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>

//
#include "FakePointCloudSampler.h"

namespace gazebo
{
    /// \class GazeboFakePointCloud
    /// Gazebo Plugin simulating point cloud acqusition using
    /// the VCG library.
    ///
    /// ADD FURTHER EXPLANATIONS

    class GazeboYarpFakePointCloud : public ModelPlugin
    {
    public:
	void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
	void OnWorldUpdate();
	
    private:
	yarp::os::Network m_yarp;
	
	gazebo::physics::ModelPtr m_model;
	gazebo::event::ConnectionPtr m_worldUpdateConnection;
	gazebo::common::Time m_lastUpdateTime;

	FakePointCloudSampler m_sampler;

	double m_period;

	void DeliverPointCloud();
    };
}

#endif
