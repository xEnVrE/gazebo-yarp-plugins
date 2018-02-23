/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// ignition
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "PointCloudViewer.h"

PointCloudViewer::~PointCloudViewer()
{
    // remove all the markers
    for (size_t i=0; i<m_markers.size(); i++)
	removeMarker(i);

    // send request for all the markers to gazebo
    for (size_t i=0; i<m_markers.size(); i++)
	m_node.Request("/marker", m_markers[i]);
}

void PointCloudViewer::setDefaultColour(const std::string &pointColor)
{
    m_defaultColor = pointColor;
}

void PointCloudViewer::setNamespace(const std::string &ns)
{
    m_namespace = ns;
}

bool PointCloudViewer::showPointCloud(const std::vector<ignition::math::Vector3d>& pc)
{
    if (m_namespace.empty())
	return false;
    
    // save the original size of the vector of markers
    int size = m_markers.size();
    
    // process the point cloud
    for (size_t i=0; i<pc.size(); i++)
	if (i < size)
	    // a marker is available
	    // recycle it by changing its pose
	    setMarker(i, pc[i]);
	else
	    // add a new marker
	    addMarker(pc[i]);
    
    if (pc.size() < size)
	// some markers are not needed anymore
	for (size_t i=pc.size(); i<size; i++)
	    removeMarker(i);

    // send request for all the markers to gazebo
    for (size_t i=0; i<m_markers.size(); i++)
	m_node.Request("/marker", m_markers[i]);

    return true;
}

void PointCloudViewer::addMarker(const ignition::math::Vector3d& pos)
{
    // create the marker message
    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns("/pointcloud_" + m_namespace);
    markerMsg.set_id(m_markers.size());
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::SPHERE);
    
    // set the pose
    ignition::msgs::Set(markerMsg.mutable_pose(),
			ignition::math::Pose3d(pos.X(),
					       pos.Y(),
					       pos.Z(),
					       0, 0, 0));
    // shrink the sphere
    ignition::msgs::Set(markerMsg.mutable_scale(),
			ignition::math::Vector3d(0.005, 0.005, 0.005));
    
    // set the color
    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    matMsg->mutable_script()->set_name(m_defaultColor);

    // add to the vector of markers
    m_markers.push_back(markerMsg);
}

void PointCloudViewer::setMarker(int index, const ignition::math::Vector3d& pos)
{
    // get the marker
    ignition::msgs::Marker &markerMsg = m_markers[index];

    // set the ADD_MODIFY action since this marker could
    // have been removed previously
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);

    // update the pose
    ignition::msgs::Set(markerMsg.mutable_pose(),
			ignition::math::Pose3d(pos.X(),
					       pos.Y(),
					       pos.Z(),
					       0, 0, 0));
}

void PointCloudViewer::removeMarker(int index)
{
    // get the marker
    ignition::msgs::Marker &markerMsg = m_markers[index];

    // set marker as deleted
    markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
}
