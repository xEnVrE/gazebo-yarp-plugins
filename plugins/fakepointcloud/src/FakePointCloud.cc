/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

// boost
#include <boost/bind.hpp>

// std
#include <string>

//
#include "FakePointCloud.hh"

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpFakePointCloud)

namespace gazebo {

void GazeboYarpFakePointCloud::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{    
    // Store pointer to the model
    m_model = _parent;

    // Store pointer to the world
    m_world = m_model->GetWorld();
    
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpFakePointCloud::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Clear last update time
    m_lastUpdateTime = gazebo::common::Time(0.0);
    
    // Listen to the update event
    auto worldUpdateBind = boost::bind(&GazeboYarpFakePointCloud::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

void GazeboYarpFakePointCloud::OnWorldUpdate()
{    
    // Get current time
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time currentTime = m_world->SimTime();
#else
    gazebo::common::Time currentTime = m_world->GetSimTime();
#endif
    
    // Sample a new fake point cloud if a period is elapsed
    // TODO: get period from the configuration file
    if(currentTime - m_lastUpdateTime >= 1.0) {
	
	// Get the current pose of the object
	gazebo::math::Pose cur_pose = m_model->GetWorldPose();;

	// Fill yarp-like quantities
	yarp::sig::Vector position;
	yarp::math::Quaternion attitude;
	
	position.push_back(cur_pose.pos.x);
	position.push_back(cur_pose.pos.y);
	position.push_back(cur_pose.pos.z);

	attitude = yarp::math::Quaternion(cur_pose.rot.x,
					  cur_pose.rot.y,
					  cur_pose.rot.z,
					  cur_pose.rot.w);

	// Store current time for next update
	m_lastUpdateTime = currentTime;
    }
}
    
}
