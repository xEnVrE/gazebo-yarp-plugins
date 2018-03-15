/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_SKIN_HH
#define GAZEBOYARP_SKIN_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

// icub-main
#include <iCub/skinDynLib/common.h>
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <string>
#include <unordered_map>

typedef std::unordered_map<std::string, gazebo::physics::LinkPtr> linksMap;

enum class linkNumberEnum;

struct ContactSensor
{
    // Some of these may be useful for future implementations
    // of the Skin plugin
    std::string sensorName;
    std::string collisionName;
    gazebo::physics::LinkPtr parentLink;
    gazebo::sensors::ContactSensorPtr sensor;

    // Required to reconstrct
    // the output of the skinManager
    iCub::skinDynLib::BodyPart bodyPart;
    linkNumberEnum linkNumber;
    iCub::skinDynLib::SkinPart skinPart;

    // In this simplificative simulation
    // each contact is associated with
    // one taxel only. Taxels IDs are used
    // to distinguish which finger tip was
    // involved in the contact.
    unsigned int taxelId;
};

enum class linkNumberEnum {
    // Only finger tips are considered in this implementation    
    HAND = 6
};

namespace gazebo
{
    /// \class GazeboYarpSkin
    ///
    class GazeboYarpSkin : public ModelPlugin
    {
    public:
	~GazeboYarpSkin();
	
	/**
	 * Store pointer to the model,
	 * and connect to the World update event of Gazebo.
	 */	
	void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
	
    private:
	/**
	 * Instance of yarp::os::Network
	 */
	yarp::os::Network m_yarp;

	/**
	 * Buffered port where skinContactList is sent
	 */
	yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> m_portSkin;

	/**
	 * Parameters of the plugin
	 */
	yarp::os::Property m_parameters;

	/**
	 * Driver required to access the IFrameTransform interface
	 */
	yarp::dev::PolyDriver m_drvTransformClient;

	/**
	 * Pointer to the IFrameTransform view
	 */
	yarp::dev::IFrameTransform *m_tfClient;

	/**
	 * Indicates whether the transform from /inertial
	 * to the robot root frame has been received
	 */
	bool m_robotRootFrameReceived;

	/**
	 * Pointer to the model where the plugin is inserted
	 */
	gazebo::physics::ModelPtr m_model;

	/**
	 * Pointer to the SDF associated to the model
	 */
	sdf::ElementPtr m_sdf;

	/**
	 * Connection to the World update event of Gazebo
	 */
	gazebo::event::ConnectionPtr m_worldUpdateConnection;

	/**	
	 * Transformation from inertial to the root link of the robot
	 */
	ignition::math::Pose3d m_inertialToRobot;

	/**
	 * List of ContactSensor(s)
	 */
	std::vector<ContactSensor> m_contactSensors;
	
	/**
	 * Map between links local names and gazebo::physics::LinkPtr(s)
	 */
	std::unordered_map<std::string, gazebo::physics::LinkPtr> m_linksMap;

	/**
	 * String indicating which hand is considered, left or right
	 */
	std::string m_whichHand;

	/**
	 * Name of the robot
	 */
	std::string m_robotName;

	/**
	 * Name of source frame requried to retrieve the pose of the robot
	 */
	std::string m_robotSourceFrameName;

	/**
	 * Name of target frame required to retrieve the pose of the robot
	 */
	std::string m_robotTargetFrameName;

	/**
	 * Name of the local port used to instantiate the driver
	 * for the torso encoders
	 */
	std::string m_torsoControlBoardLocalPort;

	/**
	 * Name of the local port used to instantiate the driver
	 * for the arm encoders
	 */
	std::string m_armControlBoardLocalPort;

	/**
	 * Name of the local port used to instantiate the driver
	 * for FrameTransformClient
	 */
	std::string m_transformClientLocalPort;

	/**
	 * Name of the output port
	 */
	std::string m_outputPortName;

	/**
	 * Load a string parameter from the SDF
	 *
	 */
	bool LoadStringParam(const std::string &name,
					     std::string &value);
	/*
	 * Retrieve the pose of the robot root frame that is published
	 * in the FrameTransformServer.
	 */
	bool RetrieveRobotRootFrame(ignition::math::Pose3d &pose);

	/**
	 * Retrieve a links given their local, i.e. not scoped, names.
	 * A local name is supposed to be unique within the model.
	 */
	bool RetrieveLinksFromLocalNames(const std::vector<std::string> &linksLocalNames,
					 linksMap &map);
	/**
	 * Configure the gazebo contact sensor corresponding to the link
	 * with local name linkLocalName.
	 */	
	bool ConfigureGazeboContactSensor(const std::string &linkLocalName,
					  ContactSensor &sensor);
	/**
	 * Configure all the contacts sensors attached to the links
	 * listed in the .ini configuration file.
	 */	
	bool ConfigureAllContactSensors();
	
	/**
	 *
	 */	
	void OnWorldUpdate();
    };
}
#endif
