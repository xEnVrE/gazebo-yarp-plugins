/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef ESTIMATEVIEWER_HH
#define ESTIMATEVIEWER_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

// std
#include <string>

namespace gazebo
{
    /// \class EstimateViewer
    ///
    class EstimateViewer : public VisualPlugin
    {
    public:
	~EstimateViewer();
	
	/**
	 * Store pointer to the visual.
	 */	
	void Load(gazebo::rendering::VisualPtr, sdf::ElementPtr);
	
    private:
	/**
	 * Instance of yarp::os::Network
	 */
	yarp::os::Network m_yarp;

	/**
	 * PolyDriver required to access a yarp::dev::IFrameTransform
	 */
	yarp::dev::PolyDriver m_drvTransformClient;

	/**
	 * Pointer to yarp::dev::IFrameTransform view of the PolyDriver
	 */
	yarp::dev::IFrameTransform* m_tfClient;

	/**
	 * Pointer to the visual
	 */
	gazebo::rendering::VisualPtr m_visual;

	/**
	 * Pointer to the sdf associated to the visual
	 */
	sdf::ElementPtr m_sdf;	

	/**
	 * Connection to the Render event of Gazebo
	 */
	gazebo::event::ConnectionPtr m_renderConnection;

	/**
	 * Name of the model where the visual element is contained
	 */
	std::string m_modelName;

	/**
	 * Name of the source frame required to retrieve the current estimate
	 */
	std::string m_sourceFrameName;

	/**
	 * Target frame suffix required to retrieve the current estimate
	 */
	std::string m_targetFrameNameSuffix;

	/**
	 * Load a string parameter from the SDF
	 */
	bool LoadStringParam(const std::string &name,
			     std::string &value);
	/**
	 * Extract the name of the model from the name of the visual
	 */
	std::string GetModelName();
	
	/**
	 * Check if a period is elapsed since last update 
	 * and in case calls the method `OnRenderUpdate`.
	 */	
	void OnRenderUpdate();
    };
}
#endif
