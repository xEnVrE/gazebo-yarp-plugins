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

// icub-main
#include <iCub/skinDynLib/common.h>
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <string>

enum class bodyPartEnum;
enum class linkNumberEnum;
enum class skinPartEnum;

struct ContactSensor
{
    std::string sensorName;
    std::string collisionName;
    gazebo::physics::LinkPtr parentLink;
    gazebo::sensors::ContactSensorPtr sensor;

    // Useful information to reconstrct
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
	 * Load all the contact sensors attached to the finger tips of
	 * the right hand.
	 */	
	bool loadRightFingerTips(const std::string &robotName);

	/**
	 * Load all the contact sensors attached to the finger tips of
	 * the left hand.
	 */	
	bool loadLeftFingerTips(const std::string &robotName);

	/**
	 * Load the gazebo contact sensor corresponding to the link
	 * with name linkName. One contact sensor per link is assumed.
	 */	
	bool loadGazeboContactSensor(const std::string &linkName,
				     const iCub::skinDynLib::BodyPart &bodyPart,
				     const linkNumberEnum &linkNumber,
				     const iCub::skinDynLib::SkinPart &skinPart);
	/**
	 *
	 */	
	void OnWorldUpdate();
    };
}
#endif
