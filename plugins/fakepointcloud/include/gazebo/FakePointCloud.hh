/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef FAKEPOINTCLOUD_HH
#define FAKEPOINTCLOUD_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

//
#include "FakePointCloudSampler.h"

// PointCloudViewer only supported in
// Gazebo >= 8
#if GAZEBO_MAJOR_VERSION >= 8
#include "PointCloudViewer.h"
#endif

namespace gazebo
{
    /// \class GazeboFakePointCloud
    /// Gazebo Plugin simulating point cloud acqusition using
    /// the VCG library.
    ///
    /// ADD FURTHER EXPLANATIONS

    class FakePointCloud : public ModelPlugin
    {
    public:
	~FakePointCloud();
	
	/**
	 * Store pointer to the model, load parameters from the SDF,
	 * reset the time of the last update and
	 * connect the method OnWorldUpdate to the World update event of Gazebo
	 */	
	void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);

	/**
	 * Check if a period is elapsed since last update 
	 * and in case sample a new point cloud, send it over 
	 * the output port and update the visualization markers.
	 */	
	void OnWorldUpdate();
	
    private:
	
	/**
	 * Instance to yarp::os::Network
	 */
	yarp::os::Network m_yarp;

	/**
	 * Port where to send point clouds
	 */
	yarp::os::BufferedPort<PointCloud> m_portOut;

	/**
	 * Pointer to the model where the plugin is inserted
	 */
	gazebo::physics::ModelPtr m_model;

	/**
	 * Pointer to the sdf associated to the model
	 */
	sdf::ElementPtr m_sdf;

	/**
	 * Connection to the World update event of Gazebo
	 */
	gazebo::event::ConnectionPtr m_worldUpdateConnection;

	/**
	 * Time of the last update of the plugin
	 */
	gazebo::common::Time m_lastUpdateTime;

	/**
	 * Instance to the sampler of the fake point cloud
	 */
	FakePointCloudSampler m_sampler;

	/**
	 * Instance to the viewer of the fake point cloud
	 */
#if GAZEBO_MAJOR_VERSION >= 8	
	PointCloudViewer m_viewer;
#endif	
	/**
	 * Update period of the plugin
	 */
	double m_period;

	/**
	 * Number of points of the point cloud
	 */	
	int m_nPoints;

	/**
	 * Whether to show the point cloud in gazebo 
	 * using visual markers or not
	 */	
	bool m_showPointCloud;

	/**
	 * Model name
	 */	
	bool m_modelName;

	/**
	 * Load a parameter with a certain type from the SDF
	 */	
	template<typename T>
	bool LoadParam(const std::string &name,
		       T &param);

	/**
	 * Load a positive scalar parameter from the SDF
	 */	
	template<typename T>
	bool LoadStrictPositiveScalarParam(const std::string &name,
					   T &param);
	/**
	 * Load parameter observerOrigin from the SDF
	 */	
	bool LoadObserverOrigin(yarp::sig::Vector &origin);

	/**
	 * Load parameter meshPath from the SDF
	 */	
	bool LoadMeshPath(std::string &path);

	/**
	 * Configure the mesh of the object to be sampled
	 */	
	bool ConfigureMesh();
	
	/**
	 * Sample a point cloud and send it to a port
	 */	
	void SamplePointCloud(PointCloud&);
    };
}

namespace gazebo 
{
    template<typename T>
    bool FakePointCloud::LoadParam(const std::string &name,
					     T &param)
    {
	// Check if the element exists
	if (!(m_sdf->HasElement(name))) {
	    yError() << "FakePointCloud::Load error:"
		     << "cannot find parameter"
		     << name
		     << "for model"
		     << m_model->GetName();
	    return false;
	}

	// Get the associated parameter
	sdf::ParamPtr paramPtr = m_sdf->GetElement(name)->GetValue();
	    
	// Check if the value can be interpreted
	// as the required type
	if (!paramPtr->Get<T>(param)) {
	    yError() << "FakePointCloud::Load error:"
		     << "parameter"
		     << name
		     << "for model"
		     << m_modelName << "should be a"
		     << typeid(param).name();
	    return false;
	}
    
	return true;
    }
	
    template<typename T>
    bool FakePointCloud::LoadStrictPositiveScalarParam(const std::string &name,
								 T &param)
    {
	if (!LoadParam<T>(name, param))
	    return false;

	if (param <= 0) {
	    yError() << "FakePointCloud::Load error:"
		     << "parameter"
		     << name
		     << "for model"
		     << m_modelName << "should be a strictly positive"
		     << typeid(param).name();
	    return false;
	}

	yInfo() << "FakePointCloud::Load"
		<< name
		<< "is"
		<< param;

	return true;
    }
}
#endif
