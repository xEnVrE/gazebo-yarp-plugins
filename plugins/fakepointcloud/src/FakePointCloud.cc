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

GZ_REGISTER_MODEL_PLUGIN(gazebo::FakePointCloud)

namespace gazebo {

FakePointCloud::~FakePointCloud()
{
    // close the output port
    m_portOut.close();
}

bool FakePointCloud::LoadObserverOrigin(yarp::sig::Vector &origin)
{
    // Try to load the origin of the observer
    ignition::math::Vector3d originIgn;    
    if (!LoadParam<ignition::math::Vector3d>("observerOrigin", originIgn))
	return false;

    // Convert to yarp vector
    origin.resize(3);
    for (size_t i=0; i<3; i++)
	origin[i] = originIgn[i];
	
    yInfo() << "FakePointCloud::Load observer origin is"
	    << origin.toString();

    return true;
}

bool FakePointCloud::LoadMeshPath(std::string &path)
{
    // Try to load the mesh path
    std::string sdfPath;
    if (!LoadParam<std::string>("meshPath", sdfPath))
	return false;
    
    // The retrieved path is relative to the path of a gazebo model
    // hence the complete path has to be evaluated
    path = gazebo::common::SystemPaths::Instance()->FindFileURI(sdfPath);

    // Check for empty path
    // which could happen in case of FindFileURI failing
    if (path.empty()) {
	yError() << "FakePointCloud::Load error:"
		 << "the mesh path evaluated to an empty string for model name"
		 << m_modelName;
	return false;
    }
    
    return true;
}

bool FakePointCloud::ConfigureMesh()
{
    std::string meshPath;
    if (!LoadMeshPath(meshPath))
	return false;
	
    // Configure object mesh within the sampler
    if (m_sampler.LoadObjectModel(meshPath)) {
	yInfo() << "FakePointCloud::Load mesh"
		<< meshPath
		<< "loaded for model"
		<< m_modelName;
    } else {
	yError() << "FakePointCloud::Load error:"
		 << "cannot load mesh"
		 << meshPath
		 << "for model"
		 << m_modelName;
	return false;
    }

    return true;
}

void FakePointCloud::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "FakePointCloud::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the model
    m_model = _parent;

    // Store pointer to the sdf
    m_sdf = _sdf;

    // Get name of the model
    std::string m_modelName = m_model->GetName();

    // Load update rate
    if (!LoadStrictPositiveScalarParam<double>("period", m_period))
	return;

    // Load number of points of the point cloud
    if (!LoadStrictPositiveScalarParam<int>("numPoints", m_nPoints))
	return;

    // Load showPointCloud flag
    if (!LoadParam<bool>("showPointCloud", m_showPointCloud))
	return;

    // Load enableNoise
    bool noiseEnabled;
    if (!LoadParam<bool>("enableNoise", noiseEnabled))
	return;

    if (noiseEnabled)
    {
	// Load mean and standard deviation for gaussian noise
	double mean;
	double std;
	if (!LoadParam<double>("noiseMean", mean))
	    return;
	if (!LoadParam<double>("noiseStd", std))
	    return;

	// Configure noise generator within sampler
	m_sampler.setGaussianNoiseParameters(mean, std);

	// Enable noise
	m_sampler.setGaussianNoiseEnabled(true);
    }

    // Load outputPortSuffix suffix
    std::string outputPortSuffix;
    if (!LoadParam<std::string>("outputPortSuffix", outputPortSuffix))
	return;
    
    // Load observer origin
    yarp::sig::Vector observerOrigin;
    if (!LoadObserverOrigin(observerOrigin))
	return;
    
    // Configure observer origin
    m_sampler.SetObserverOrigin(observerOrigin);

    // Configure mesh
    if (!ConfigureMesh())
	return;

    // Configure the point cloud viewer part
    if (m_showPointCloud)
#if GAZEBO_MAJOR_VERSION >= 8
    {
	// Get the desired colour for the point cloud
	std::string colour;

	if (!LoadParam<std::string>("pointCloudColour", colour))
	    return;
	
	// Set default colour
	// m_viewer.setDefaultColour("Gazebo/RedTransparent");
	m_viewer.setDefaultColour(colour);	

	// Set the namespace
	m_viewer.setNamespace(m_modelName);
    }
#else
    yWarning() << "FakePointCloud::Load"
	       << "showPointCloud feature requires minimum"
	       << "version 8 of Gazebo (warning for model"
	       << m_modelName << ")";
#endif

    // Clear last update time
    m_lastUpdateTime = gazebo::common::Time(0.0);
	
    // Open port
    // Using the model name provided by gazebo enforces uniqueness of port name
    // in case a model is inserted in the environment more than once
    std::string port_name = "/" + m_modelName + "/" + outputPortSuffix;
    bool isPortOpen = m_portOut.open(port_name);
    if (!isPortOpen) {
	yError() << "FakePointCloud::Load error:"
	         << "cannot open output port for model"
		 << m_modelName;
	return;
    }

    // Listen to the reset event
    auto worldResetBind = boost::bind(&FakePointCloud::OnWorldReset, this);
    m_worldResetConnection = gazebo::event::Events::ConnectWorldReset(worldResetBind);

    // Listen to the update event
    auto worldUpdateBind = boost::bind(&FakePointCloud::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

void FakePointCloud::OnWorldReset()
{
    // Reset the time
    m_lastUpdateTime = gazebo::common::Time(0.0);
}
    
void FakePointCloud::OnWorldUpdate()
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
	PointCloudXYZ &cloud = m_portOut.prepare();
	SamplePointCloud(cloud);

	// Deliver the point cloud
	m_portOut.write();


#if GAZEBO_MAJOR_VERSION >= 8
	// Update visualization markers if required	
	if (m_showPointCloud)
	{
	    std::vector<ignition::math::Vector3d> cloud_ign;
	    for (size_t i=0; i<cloud.size(); i++)
	    {
		ignition::math::Vector3d point(cloud(i).x,
					       cloud(i).y,
					       cloud(i).z);
		cloud_ign.push_back(point);
	    }
	    if(!m_viewer.showPointCloud(cloud_ign)) {

		yError() << "FakePointCloud::OnWorldUpdate error:"
			 << "failure in point cloud visualization for model"
			 << m_modelName;
	    }
		
	}
#endif
    }
}

void FakePointCloud::SamplePointCloud(PointCloudXYZ &pc)
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
    
}
