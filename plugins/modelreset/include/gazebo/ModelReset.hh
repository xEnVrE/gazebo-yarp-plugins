/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_MODELPOSEPUBLISHER_HH
#define GAZEBOYARP_MODELPOSEPUBLISHER_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ConnectionReader.h>

namespace gazebo
{
    /// \class GazeboYarpModelReset
    /// Gazebo Plugin that allows to reset the pose of a model
    /// using RPC calls.
    ///
    class GazeboYarpModelReset : public ModelPlugin, public yarp::os::PortReader
    {
    public:
	~GazeboYarpModelReset();
	
	/**
	 * Store pointer to the model, store the initial pose of the model 
	 * and configure the Rpc server.
	 */
	void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
	
    private:
	/**
	 * Instance of yarp::os::Network
	 */
	yarp::os::Network m_yarp;

	/**
	 * Pointer to the model where the plugin is inserted
	 */
	gazebo::physics::ModelPtr m_model;

	/**
	 * Instance of yarp::os::RpcServer
	 */
	yarp::os::RpcServer m_rpcPort;

	/**
	 * Initial pose of the model
	 */
#if GAZEBO_MAJOR_VERSION >= 8
	ignition::math::Pose3d m_initialPose;
#else
	gazebo::math::Pose m_initialPose;
#endif

	/**
	 * Read incoming rpc requests
	 */
	bool read(yarp::os::ConnectionReader& connection);

	/**
	 * Process incoming rpc requests
	 */
	void processRequest(const yarp::os::Bottle &request,
			    yarp::os::Bottle &response);

	/**
	 * Reset the initial pose of the model
	 */
	void resetModelPose();
    };
}
#endif
