/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/sensors.hh>

// ignition
// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Vector3.hh>
// #include <ignition/math/Quaternion.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>

// icub-main
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/dynContact.h>

// boost
#include <boost/bind.hpp>

// std
#include <numeric>

#include "Skin.hh"

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpSkin)

namespace gazebo {

GazeboYarpSkin::~GazeboYarpSkin()
{
    // Close the port
    m_portSkin.close();
    
    // Close the driver
}

void GazeboYarpSkin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store pointer to the model
    m_model = _parent;
    
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Get .ini configuration file from plugin sdf
    bool ok = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, m_parameters);
    if (!ok) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "error loading .ini configuration from plugin SDF";
        return;
    }

    // Get hand type
    yarp::os::Value &whichHandValue = m_parameters.find("whichHand");
    if (whichHandValue.isNull()) {
        yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
		 << "configuration parameter 'whichHand' not found.";
        return;
    }
    m_whichHand = whichHandValue.asString();
    
    // Configure all the contact sensors
    ok = ConfigureAllContactSensors();
    if (!ok) {
	return;
    }

    // Open skin port
    ok = m_portSkin.open("/" + m_whichHand + "_hand/skinManager/skin_events:o");
    if (!ok)  {
        yError() << "GazeboYarpSkin::Load error:"
		 << "cannot open port /skinManager/skin_events:o";
        return;
    }
	
    // listen to the update event
    auto worldUpdateBind = boost::bind(&GazeboYarpSkin::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

bool GazeboYarpSkin::RetrieveLinksFromLocalNames(const std::vector<std::string> &linksLocalNames,
						 linksMap &map)
{
    // Get all the links within the robot
    const gazebo::physics::Link_V &links = m_model->GetLinks();
    
    // Search for the given links
    // Lots of redundancy here, room for improvements...
    //
    for (size_t i=0; i<links.size(); i++)
    {
	// Get the scoped name of the current link
	std::string currentLinkScopedName = links[i]->GetScopedName();
	
	for (size_t j=0; j<linksLocalNames.size(); j++)
	{
	    // Check if the ending of the name of the current link corresponds to
	    // that of one of the given links
	    std::string linkNameScopedEnding = "::" + linksLocalNames[j];
	    if (GazeboYarpPlugins::hasEnding(currentLinkScopedName, linkNameScopedEnding))
	    {
		// Store the link into the map
		map[linksLocalNames[j]] = links[i];

		break;
	    }
	}
    }

    // Return false if not all the links were found
    if (map.size() != linksLocalNames.size())
	return false;

    return true;
}

bool GazeboYarpSkin::ConfigureGazeboContactSensor(const std::string &linkLocalName,
						  ContactSensor &sensor)
{
    // Retrieve the link
    sensor.parentLink = m_linksMap[linkLocalName];
    if (sensor.parentLink == NULL)
	return false;
    
    // Check if this link contains any sensor
    size_t nSensors = sensor.parentLink->GetSensorCount();
    if (nSensors <= 0)
	return false;
    
    // One contact sensor per link is expected
    // however many sensors can be attached to the same link
    // hence it is required to search for the contact sensor
    sdf::ElementPtr modelSdf = sensor.parentLink->GetSDF();
    sdf::ElementPtr child;
    bool foundContactSensor = false;
    for (child = modelSdf->GetElement("sensor");
	 child != sdf::ElementPtr(nullptr);
	 child = child->GetNextElement("sensor"))
    {
	if ((child->GetAttribute("type")->GetAsString()) == "contact")
	{
	    // check if a child "contact" tag exists since it is not required
	    // by default
	    if (child->HasElement("contact"))
	    {
		foundContactSensor = true;
		break;
	    }
	}
    }
    if (!foundContactSensor)
	return false;

    // Extract scoped sensor name
    std::string localSensorName = child->GetAttribute("name")->GetAsString();
    std::vector<std::string> scopedNameList = m_model->SensorScopedName(localSensorName);
    sensor.sensorName = std::accumulate(std::begin(scopedNameList),
					std::end(scopedNameList),
					sensor.sensorName);
    // Extract collision name
    sdf::ElementPtr contact = child->GetElement("contact");
    sensor.collisionName = contact->GetElement("collision")->GetValue()->GetAsString();

    // Get the sensor from the sensor manager
    gazebo::sensors::SensorManager *sensorMgr = gazebo::sensors::SensorManager::Instance();
    if (!sensorMgr->SensorsInitialized())
	return false;
    gazebo::sensors::SensorPtr genericPtr = sensorMgr->GetSensor(sensor.sensorName);
    sensor.sensor = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(genericPtr);
    if (sensor.sensor == nullptr)
	return false;

    // Activate sensor
    sensor.sensor->SetActive(true);

    return true;
}

bool GazeboYarpSkin::ConfigureAllContactSensors()
{
    // Configure skinManager parameters
    iCub::skinDynLib::BodyPart bodyPart;
    iCub::skinDynLib::SkinPart skinPart;
    linkNumberEnum linkNumber = linkNumberEnum::HAND;
    if (m_whichHand == "right")
    {
	bodyPart = iCub::skinDynLib::BodyPart::RIGHT_ARM;
	skinPart = iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND;
    }
    else
    {
	bodyPart = iCub::skinDynLib::BodyPart::LEFT_ARM;
	skinPart = iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND;
    }
    
    // Get local link names of the finger tips
    yarp::os::Bottle linksLocalNamesBottle = m_parameters.findGroup("linkNames");
    std::vector<std::string> linksLocalNames;
    if (linksLocalNamesBottle.isNull()) {
        yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
		 << "configuration parameter 'linkNames' not found.";
        return false;
    }
    int numberOfLinks = linksLocalNamesBottle.size()-1;
    for (size_t i=0; i<numberOfLinks; i++)
	linksLocalNames.push_back(linksLocalNamesBottle.get(i+1).asString().c_str());

    // Retrieve the links from the model
    RetrieveLinksFromLocalNames(linksLocalNames, m_linksMap);

    // Configure contact sensors
    std::string linkLocalName;
    bool ok;
    for (size_t i=0; i<numberOfLinks; i++)
    {
	ContactSensor sensor;

	// Copy skinManager parameters
	sensor.bodyPart = bodyPart;
	sensor.linkNumber = linkNumber;
	sensor.skinPart = skinPart;

	// Configure Gazebo contact sensor
	linkLocalName = linksLocalNames[i];
	ok = ConfigureGazeboContactSensor(linkLocalName, sensor);
	if (!ok) {
	    yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
		     << "cannot configure link"
		     << linkLocalName;
	    return false;
	}

	// Add sensor to the list
	m_contactSensors.push_back(sensor);
    }

    return true;
}

void GazeboYarpSkin::OnWorldUpdate()
{
    // Process contacts for each contact sensor
    for (size_t i=0; i<m_contactSensors.size(); i++)
    {
	ContactSensor &sensor = m_contactSensors[i];
	msgs::Contacts contacts = sensor.sensor->Contacts();
	
	// Only geometric center of the contact is considered
	// in this implementation
	// yarp::sig::Vector null(3, 0.0);
	// iCub::skinDynLib::dynContact(sensor.bodyPart,
	// 			     static_cast<int>(sensor.linkNumber),
	// 			     null);
	// iCub::skinDynLib::skinContact(
	if (contacts.contact_size() == 0)
	    continue;
	
	yInfo() << "*****************************************";
	yInfo() << "*" << sensor.sensorName;
	yInfo() << "# contacts" << contacts.contact_size();
	yInfo() << "*****************************************";

	for (size_t j=0; j<contacts.contact_size(); j++)
	{
	    for (size_t k=0; k<contacts.contact(j).position_size(); k++)
	    {
		yInfo() << "(" << k << ")" << "Position:";
		auto position = contacts.contact(j).position(k);
		yInfo() << position.x() << position.y() << position.z();
	    }

	}
	yInfo() << "*****************************************";
	yInfo() << "";
    }
}

}
