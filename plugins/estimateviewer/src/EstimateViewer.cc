/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/Events.hh>

// ignition
#include <ignition/math/Pose3.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Quaternion.h>
#include <yarp/math/FrameTransform.h>

// boost
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#include "EstimateViewer.hh"

GZ_REGISTER_VISUAL_PLUGIN(gazebo::EstimateViewer)

namespace gazebo {

EstimateViewer::~EstimateViewer()
{
}

void EstimateViewer::Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "EstimateViewer::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the visual
    m_visual = _parent;

    // Store model name
    m_modelName = GetModelName();

    // Prepare properties for the PolyDriver
    // It is used to access a IFrameTransformServer where
    // the estimated pose of the object is published
    yarp::os::Property propTfClient;
    propTfClient.put("device", "transformClient");
    // The local port depends on the model name that is unique
    // also in the case of multiple insertions of the same model
    // in Gazebo
    propTfClient.put("local", "/" + m_modelName + "/estimate_viewer/transformClient:i");
    propTfClient.put("remote", "/transformServer");

    // Open the driver and obtain a IFrameTransform view
    m_tfClient = nullptr;
    bool drv_ok = m_drvTransformClient.open(propTfClient);
    drv_ok = drv_ok && m_drvTransformClient.view(m_tfClient) && m_tfClient != nullptr;

    // Return if the driver open failed
    // or the view retrieval failed
    // or the IFrameTransform pointer is not valid
    if (!drv_ok) {
	yError() << "EstimateViewer::Load error:"
		 << "failure in opening iFrameTransform interface for model"
		 << m_modelName;
	return;
    }

    // Listen to the update event
    auto renderBind = boost::bind(&EstimateViewer::OnRenderUpdate, this);
    m_renderConnection = gazebo::event::Events::ConnectRender(renderBind);
}

std::string EstimateViewer::GetModelName()
{
    // Get the name of the visual element
    std::string visual_name = m_visual->GetName();

    // The name provided by Gazebo is of the form
    // model_name::link_name::visual_name
    // hence the first item is required
    std::vector<std::string> components;
    boost::split(components, visual_name,
		 boost::is_any_of("::"), boost::token_compress_on );

    return components[0];
}

void EstimateViewer::OnRenderUpdate()
{
    // Get the estimated pose
    yarp::sig::Matrix matrixTransform(4, 4);
    std::string source = "/inertial";
    std::string target = "/" + m_modelName + "/estimate/frame";
    bool ok_transform = m_tfClient->getTransform(target, source, matrixTransform);
    // Return if the transform is not ready
    if (!ok_transform)
    	return;

    // Convert the estimate pose of the object
    // to a ignition::math::Pose3d
    yarp::math::FrameTransform frameTransform;
    frameTransform.fromMatrix(matrixTransform);
    auto pos = frameTransform.translation;
    yarp::math::Quaternion rot = frameTransform.rotation;
    ignition::math::Pose3d estPoseIgn(pos.tX, pos.tY, pos.tZ,
				   rot.w(), rot.x(), rot.y(), rot.z());
    
    // Set the new visual representation of the estimated pose
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d &estPose = estPoseIgn;
#else
    gazebo::math::Pose estPose(estPoseIgn);
#endif
    m_visual->SetWorldPose(estPose); 
}

}
