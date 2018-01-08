/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef FAKEPOINTCLOUD_VIEWER_HH
#define FAKEPOINTCLOUD_VIEWER_HH

// ignition
#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ignition/math/Vector3.hh>

// std
#include <vector>
#include <string>

/// \class PointCloudViewer
///
/// Show point cloud as a series of spheres
/// represented using Gazebo visualization markers.
///

class PointCloudViewer
{    
public:
    ~PointCloudViewer();
    
    /*
     * Set the default color for the markers.
     */
    void setDefaultColour(const std::string &markerColor);
    
    /*
     * Set the name space to be used within gazebo messages
     */
    void setNamespace(const std::string &ns);

    /*
     * Show the point cloud using Gazebo visualization markers.
     */
    bool showPointCloud(const std::vector<ignition::math::Vector3d>&);
    
private:
    // transport layer required to send Marker messages to Gazebo
    ignition::transport::Node m_node;

    // vector of Marker messages
    std::vector<ignition::msgs::Marker> m_markers;

    // default color used for spheres
    std::string m_defaultColor;

    // name space to be used within gazebo messages
    std::string m_namespace;

    /*
     * Add a new marker to the viewer.
     */
    void addMarker(const ignition::math::Vector3d&);

    /*
     * Set an existing marker in the viewer.
     */
    void setMarker(int index, const ignition::math::Vector3d&);

    /*
     * Remove an existing marker in the viewer.
     */
    void removeMarker(int index);
    
};

#endif
