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

    class GazeboYarpFakePointCloud : public ModelPlugin
    {
    public:
	~GazeboYarpFakePointCloud();
	
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
	 * Sample a point cloud and send it to a port
	 */	
	void SamplePointCloud(PointCloud&);
    };
}

#endif
