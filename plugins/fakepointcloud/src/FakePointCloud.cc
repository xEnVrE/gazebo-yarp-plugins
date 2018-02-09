/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

// ignition
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

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

void GazeboYarpFakePointCloud::SamplePointCloud(PointCloud &pc)
{
    // Get the current pose of the canonical link of the model
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d curPose = m_model->GetLink()->WorldPose();
#else
    gazebo::math::Pose curPoseGazebo = m_model->GetLink()->GetWorldPose();
    // Convert to Ignition so that the same interface
    // can be used in the rest of the function
    ignition::math::Pose3d curPose = curPoseGazebo.Ign();
#endif

    // Get the positional and rotational parts
    ignition::math::Vector3d pos = curPose.Pos();
    ignition::math::Quaterniond rot = curPose.Rot();
    
    // Fill yarp-like quantities
    yarp::sig::Vector position;
    yarp::math::Quaternion attitude;
	
    position.push_back(pos.X());
    position.push_back(pos.Y());
    position.push_back(pos.Z());

    attitude = yarp::math::Quaternion(rot.X(),
				      rot.Y(),
				      rot.Z(),
				      rot.W());
    // Set current pose
    m_sampler.SetPose(position, attitude);
    
    // Sample the point cloud
    m_sampler.SamplePointCloud(m_nPoints, pc);
}

GazeboYarpFakePointCloud::~GazeboYarpFakePointCloud()
{
    // close the output port
    m_portOut.close();
}

void GazeboYarpFakePointCloud::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpFakePointCloud::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the model
    m_model = _parent;

    // Get name of the model
    std::string model_name = m_model->GetName();

    // Open port
    std::string port_name = "/" + model_name + "/fakepointcloud:o";
    bool isPortOpen = m_portOut.open(port_name);
    if (!isPortOpen) {
	yError() << "GazeboYarpFakePointCloud::Load error:"
	         << "cannot open output port for model"
		 << model_name;
	return;
    }
    
    // Load update period
    if (_sdf->HasElement("period")) {
	// set update period
        m_period = _sdf->Get<double>("period");

	yInfo() << "GazeboYarpFakePointCloud::Load period is"
		<< m_period;
    } else {
	yError() << "GazeboYarpFakePointCloud::Load error:"
	         << "failure in loading parameter 'period' for model"
		 << model_name;
	return;
    }

    // Load observer origin
    if (_sdf->HasElement("observerOrigin")) {
	ignition::math::Vector3d origin_ign = _sdf->Get<ignition::math::Vector3d>("observerOrigin");
	
	yarp::sig::Vector origin(3, 0.0);
	for (size_t i=0; i<3; i++)
	    origin[i] = origin_ign[i];
	
	// Set observer origin within the sampler
	m_sampler.SetObserverOrigin(origin);

	yInfo() << "GazeboYarpFakePointCloud::Load observer origin is"
		<< origin.toString();
    } else {
	yError() << "GazeboYarpFakePointCloud::Load error:"
	         << "failure in loading parameter 'observerOrigin' for model"
		 << model_name;
	return;
    }

    // Load number of points of the point cloud
    if (_sdf->HasElement("numPoints")) {
	// Set number of points
	m_nPoints = _sdf->Get<int>("numPoints");

	yInfo() << "GazeboYarpFakePointCloud::Load number of points is"
		<< m_nPoints;
    } else {
	yError() << "GazeboYarpFakePointCloud::Load error:"
	         << "failure in loading parameter 'numPoints' for model"
		 << model_name;
	return;
    }

    // Load mesh path
    if (_sdf->HasElement("meshPath")) {
	std::string mesh_name = _sdf->Get<std::string>("meshPath");

	// Check for empty strings
	if (mesh_name.empty()) {
	    yError() << "GazeboYarpFakePointCloud::Load error:"
		     << "parameter 'meshPath' evaluated to an empty string for model name"
		     << model_name;
	    return;
	}

	// Extract path
        std::string mesh_path = gazebo::common::SystemPaths::Instance()->FindFileURI(mesh_name);
	
	// Load object model within the sampler
	if (m_sampler.LoadObjectModel(mesh_path)) {
	    yInfo() << "GazeboYarpFakePointCloud::Load model"
		    << mesh_path
		    << "loaded for model"
		    << model_name;
	} else {
	    yError() << "GazeboYarpFakePointCloud::Load error:"
		     << "cannot load mesh for model"
		     << model_name;
	}
	
    } else {
	yError() << "GazeboYarpFakePointCloud::Load error:"
	         << "failure in loading parameter 'meshPath' for model"
		 << model_name;
	return;
    }

    // Load showPointCloud flag
    if (_sdf->HasElement("showPointCloud"))
	m_showPointCloud = _sdf->Get<bool>("showPointCloud");
    else
	// default to false
	m_showPointCloud = false;

    // Clear last update time
    m_lastUpdateTime = gazebo::common::Time(0.0);

    // Set the default colour of the visualization markers
    // TODO: take from SDF configuration
    if (m_showPointCloud)
#if GAZEBO_MAJOR_VERSION >= 8
	m_viewer.setDefaultColour("Gazebo/RedTransparent");
#else
    yWarning() << "GazeboYarpFakePointCloud::Load warning:"
	       << "showPointCloud option requires minimum"
	       << "version 8 of Gazebo (model)"
	       << model_name;
#endif
	


    // Listen to the update event
    auto worldUpdateBind = boost::bind(&GazeboYarpFakePointCloud::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

void GazeboYarpFakePointCloud::OnWorldUpdate()
{    
    // Get current time
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time currentTime = m_model->GetWorld()->SimTime();
#else
    gazebo::common::Time currentTime = m_model->GetWorld()->GetSimTime();
#endif
    
    // Sample a new fake point cloud if a period is elapsed
    if(currentTime - m_lastUpdateTime >= m_period) {
	
	// Store current time for next update
	m_lastUpdateTime = currentTime;

	// Sample point cloud
	PointCloud &cloud = m_portOut.prepare();
	SamplePointCloud(cloud);

	// Deliver the point cloud
	m_portOut.write();

	// Update visualization markers
	std::vector<ignition::math::Vector3d> cloud_ign;
	for (size_t i=0; i<cloud.size(); i++)
	{
	    ignition::math::Vector3d point(cloud[i].x,
					   cloud[i].y,
					   cloud[i].z);					   
	    cloud_ign.push_back(point);
	}
	m_viewer.showPointCloud(cloud_ign);
	
    }
}
    
}
