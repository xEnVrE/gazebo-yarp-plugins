/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>

// ignition
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

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

#include "FakePointCloudViewer.hh"

GZ_REGISTER_VISUAL_PLUGIN(gazebo::FakePointCloudViewer)

namespace gazebo {

FakePointCloudViewer::~FakePointCloudViewer()
{
    // close the port
    m_pointCloudPort.close();

    // close the driver
    m_drvTransformClient.close();
}

void FakePointCloudViewer::Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "FakePointCloudViewer::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the visual
    m_visual = _parent;

    // Store model name
    m_modelName = GetModelName();

    // Prepare properties for the PolyDriver
    // It is used to access a IFrameTransformServer where
    // the absolute pose of the object is published
    yarp::os::Property propTfClient;
    propTfClient.put("device", "transformClient");
    // The local port depends on the model name that is unique
    // also in the case of multiple insertions of the same model
    // in Gazebo
    propTfClient.put("local", "/" + m_modelName + "/fakepointcloud_viewer/transformClient:i");
    propTfClient.put("remote", "/transformServer");

    // Open the driver and obtain a IFrameTransform view
    m_tfClient = nullptr;
    bool drv_ok = m_drvTransformClient.open(propTfClient);
    drv_ok = drv_ok && m_drvTransformClient.view(m_tfClient) && m_tfClient != nullptr;

    // Return if the driver open failed
    // or the view retrieval failed
    // or the IFrameTransform pointer is not valid
    if (!drv_ok) {
	yError() << "FakePointCloudViewer::Load error:"
		 << "failure in opening iFrameTransform interface for model"
		 << m_modelName;
	return;
    }

    // Open a port for point clouds
    // libgazebo_yarp_fakePointCloud.so opens a port named
    // "/model_name/fakepointcloud:o".
    // This port will be called "/model_name/fakepointcloud_viewer:i"
    std::string port_name = "/" + m_modelName + "/fakepointcloud_viewer:i";
    bool port_ok = m_pointCloudPort.open(port_name);
    if (!port_ok) {
	yError() << "FakePointCloudViewer::Load error:"
		 << "failure in opening port"
		 << port_name
		 << "for model"
		 << m_modelName;
	return;
    }

    // Allocate a fixed number of points within the line
    // TODO: get the parameter n from the config
    int n = 100;
    m_line = m_visual->CreateDynamicLine(gazebo::rendering::RENDERING_LINE_LIST);
    m_line->setMaterial("Gazebo/Black");
    m_line->setVisibilityFlags(GZ_VISIBILITY_GUI);
    // 2*n points are required since half of them
    // are used as origin of each vector
    for (size_t i=0; i<2*n; i++)
    	m_line->AddPoint(ignition::math::Vector3d::Zero);
    m_line->Update();
    
    // Listen to the update event
    auto renderBind = boost::bind(&FakePointCloudViewer::OnRenderUpdate, this);
    m_renderConnection = gazebo::event::Events::ConnectRender(renderBind);
}

std::string FakePointCloudViewer::GetModelName()
{
    // Get the name of the visual element
#if GAZEBO_MAJOR_VERSION >= 8    
    std::string visual_name = m_visual->Name();
#else
    std::string visual_name = m_visual->GetName();
#endif

    // The name provided by Gazebo is of the form
    // model_name::link_name::visual_name
    // hence the first item is required
    std::vector<std::string> components;
    boost::split(components, visual_name,
		 boost::is_any_of("::"), boost::token_compress_on );

    return components[0];
}

void FakePointCloudViewer::OnRenderUpdate()
{
    // Since point clouds are provided in Gazebo world frame
    // and relative to the origin of Gazebo world frame
    // it is required to get the current pose of the object
    // to obtain the position of the points expressed in the local
    // visual frame and relative to the origin of the local visual frame
    //
    yarp::sig::Matrix matrixTransform(4, 4);
    std::string source = "/inertial";
    std::string target = "/" + m_modelName + "/frame";
    bool ok_transform = m_tfClient->getTransform(target, source, matrixTransform);
    // Return if the transform is not ready
    if (!ok_transform)
	return;

    // Read point cloud from the port
    bool shouldWait = false;
    PointCloud *pc = m_pointCloudPort.read(shouldWait);

    // Return if no point cloud available
    if (pc == YARP_NULLPTR)
    	return;

    // Convert the absolute pose of the object
    // to a ignition::math::Pose3d
    yarp::math::FrameTransform frameTransform;
    frameTransform.fromMatrix(matrixTransform);
    auto pos = frameTransform.translation;
    yarp::math::Quaternion rot = frameTransform.rotation;
    ignition::math::Pose3d absPose(pos.tX, pos.tY, pos.tZ,
				   rot.w(), rot.x(), rot.y(), rot.z());

    // Update the dynamic lines
    ignition::math::Vector3d diff = ignition::math::Vector3d::Zero;    
    for (size_t i=0; i<m_line->GetPointCount(); i++) {

	if (i<pc->size()) {
	    // Extract point
	    PointCloudItem pcItem = (*pc)[i];

	    // Convert to a ignition::math::Pose3d with no rotation
	    ignition::math::Pose3d point(pcItem.x, pcItem.y, pcItem.z,
					 0, 0, 0);

	    // Evaluate the pose from the origin of the visual to the point
	    // expressed in the local visual frame
	    ignition::math::Pose3d diff_pose = point - absPose;

	    // Make the vector a bit longer so that it is visible outside
	    // the mesh
	    // NOTE: for a better looking representation consider using
	    // plugin FakePointCloud that now is handling visualization
	    // in Gazebo >= 8.0  using visualization markers
	    //
	    diff = diff_pose.Pos();
	    diff += diff / diff.Length() * 0.002;

	}
	// Assign the (2*i + 1)-th point of the line
	m_line->SetPoint(2*i + 1, diff);
    }
}

}
