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
	 * pointer to the model where the plugin is inserted
	 */
	gazebo::physics::ModelPtr m_model;

	/**
	 * Connection to the World update event of Gazebo
	 */
	gazebo::event::ConnectionPtr m_worldUpdateConnection;

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
