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

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

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
}

void GazeboYarpSkin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the model
    m_model = _parent;

    // TODO: extract robot name from configuation
    std::string robotName = "iCub";

    // Load all the contact sensors attached to finger tips
    if (!loadRightFingerTips(robotName) || !loadLeftFingerTips(robotName))
	return;

    // Open skin port
    m_portSkin.open("/skinManager/skin_events:o");

    // listen to the update event
    auto worldUpdateBind = boost::bind(&GazeboYarpSkin::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

bool GazeboYarpSkin::loadRightFingerTips(const std::string &robotName)    
{
    // TODO: get these from configuration file
    std::vector<std::string> rightFingerTipsLinkNames;
    rightFingerTipsLinkNames.push_back("r_tl4");
    rightFingerTipsLinkNames.push_back("r_ail3");
    rightFingerTipsLinkNames.push_back("r_ml3");
    rightFingerTipsLinkNames.push_back("r_ril3");
    rightFingerTipsLinkNames.push_back("r_lil3");
    
    // Load all the contact sensors
    bool ok;
    for (size_t i=0; i<rightFingerTipsLinkNames.size(); i++)
    {
	std::string linkName = rightFingerTipsLinkNames[i];
	ok = loadGazeboContactSensor(robotName + "::r_hand::" + linkName,
				     iCub::skinDynLib::BodyPart::RIGHT_ARM,
				     linkNumberEnum::HAND,
				     iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND);
	if (!ok) {
	    yError() << "GazeboYarpSkin::Load error:"
		     << "cannot load contact sensor attached to link"
		     << linkName;
	    return false;	
	}
    }

    return true;
}

bool GazeboYarpSkin::loadLeftFingerTips(const std::string &robotName)
{
    // TODO: get these from configuration file
    std::vector<std::string> leftFingerTipsLinkNames;
    leftFingerTipsLinkNames.push_back("l_tl4");
    leftFingerTipsLinkNames.push_back("l_ail3");
    leftFingerTipsLinkNames.push_back("l_ml3");
    leftFingerTipsLinkNames.push_back("l_ril3");
    leftFingerTipsLinkNames.push_back("l_lil3");
    
    // Load all the contact sensors
    bool ok;
    for (size_t i=0; i<leftFingerTipsLinkNames.size(); i++)
    {
	std::string linkName = leftFingerTipsLinkNames[i];
	ok = loadGazeboContactSensor(robotName + "::l_hand::" + linkName,
				     iCub::skinDynLib::BodyPart::LEFT_ARM,
				     linkNumberEnum::HAND,
				     iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND);
	if (!ok) {
	    yError() << "GazeboYarpSkin::Load error:"
		     << "cannot load contact sensor attached to link"
		     << linkName;
	    return false;	
	}
    }

    return true;
}    

bool GazeboYarpSkin::loadGazeboContactSensor(const std::string &linkName,
				     const iCub::skinDynLib::BodyPart &bodyPart,
				     const linkNumberEnum &linkNumber,
				     const iCub::skinDynLib::SkinPart &skinPart)
{
    ContactSensor sensor;

    // Copy skinManager info
    sensor.bodyPart = bodyPart;
    sensor.linkNumber = linkNumber;
    sensor.skinPart = skinPart;

    // Retrieve the link
    sensor.parentLink = m_model->GetLink(linkName);
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

    // Add sensor to the list
    m_contactSensors.push_back(sensor);

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
