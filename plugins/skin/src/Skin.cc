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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/FrameTransform.h>

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

    // Prepare properties for the Encoders
    yarp::os::Property propEnc;
    propEnc.put("device", "remote_controlboard");
    propEnc.put("remote", "/icubSim/" + m_whichHand + "_arm");
    propEnc.put("local", "/gazebo_yarp_skin/encoder/" + m_whichHand + "_arm");
    ok = m_drvArmEnc.open(propEnc);
    if (!ok) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "unable to open the Remote Control Board driver.";
	return;
    }

    // Try to retrieve the view
    m_drvArmEnc.view(m_iEnc);
    if (!ok || m_iEnc == 0) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "unable to retrieve the Encoders view.";
	return;
    }

    // Prepare properties for the FrameTransformClient
    yarp::os::Property propTfClient;
    propTfClient.put("device", "transformClient");
    propTfClient.put("local", "/gazebo_yarp_skin/" + m_whichHand + "_hand/transformClient");
    propTfClient.put("remote", "/transformServer");
	
    // try to open the driver
    ok = m_drvTransformClient.open(propTfClient);
    if (!ok) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "unable to open the FrameTransformClient driver.";
	return;
    }
    
    // Try to retrieve the view
    ok = m_drvTransformClient.view(m_tfClient);
    if (!ok || m_tfClient == 0) {
        yError() << "GazeboYarpSkin::Load error:"
		 << "unable to retrieve the FrameTransformClient view.";
	return;
    }

    // Retrieve the pose of the root frame of the robot
    ok = RetrieveRobotRootFrame(m_inertialToRobot);
    if (!ok) {
	yError() << "GazeboYarpSkin:Load error:"
		 << "unable to get the pose of the root link of the robot.";
	return;
    }
    
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

bool GazeboYarpSkin::RetrieveRobotRootFrame(ignition::math::Pose3d &pose)
{
    // Get the pose of the root frame of the robot
    // TODO: get source and target from configuration file
    yarp::sig::Matrix inertialToRobot(4,4);
    std::string source = "/inertial";
    std::string target = "/iCub/frame";

    bool ok = false;
    double t0 = yarp::os::SystemClock::nowSystem();
    while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
    {
	// this might fail if the gazebo pluging
	// publishing the pose is not started yet
	if (m_tfClient->getTransform(target, source, inertialToRobot))
	{
	    ok = true;
	    break;
	}
	yarp::os::SystemClock::delaySystem(1.0);
    }

    // Convert to ignition::math::Pose3d
    yarp::math::FrameTransform frame;
    frame.fromMatrix(inertialToRobot);

    yarp::math::FrameTransform::Translation_t  &pos = frame.translation;
    yarp::math::Quaternion &quat = frame.rotation;
    
    pose = ignition::math::Pose3d(pos.tX, pos.tY, pos.tZ,
				  quat.w(), quat.x(), quat.y(), quat.z());
    
    return true;
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

bool GazeboYarpSkin::RetrieveHandPose(ignition::math::Pose3d &pose)
{
    // Get current value of encoders
    int nEncs;
    bool ok = m_iEnc->getAxes(&nEncs);
    if(!ok)
	return false;

    yarp::sig::Vector encs(nEncs);
    ok = m_iEnc->getEncoders(encs.data());
    if(!ok)
	return false;

    // Instantiate an arm chain
    iCub::iKin::iCubArm arm(m_whichHand);

    // Set the current values of the joints
    arm.setAng(encs);

    // Get pose of the hand
    // as x,y,z + axis/angle
    yarp::sig::Vector poseVector;
    bool axisAngleRepr = true;
    poseVector = arm.EndEffPose(axisAngleRepr);

    // Convert attitude to a quaternion
    yarp::math::Quaternion quat;
    quat.fromAxisAngle(poseVector.subVector(3,6));

    // Convert to a ignition::math::Pose3d
    pose = ignition::math::Pose3d(poseVector[0], poseVector[1], poseVector[2],
				  quat.w(), quat.x(), quat.y(), quat.z());

    return true;
}
	
void GazeboYarpSkin::OnWorldUpdate()
{
    // Retrieve hand pose
    ignition::math::Pose3d handPose;
    if (!RetrieveHandPose(handPose)) {
        yError() << "GazeboYarpSkin::OnWorldUpdate error:"
		 << "unable to get the pose of the"
		 << m_whichHand
		 << "hand";
        return;
    }

    // Compose transformation from inertial to robot root frame
    // and transformation from robot root frame to hand frame
    ignition::math::Pose3d inertialToHand = m_inertialToRobot + handPose;
    
    // Process contacts for each contact sensor
    iCub::skinDynLib::skinContactList &skinContactList = m_portSkin.prepare();
    skinContactList.clear();
    
    for (size_t i=0; i<m_contactSensors.size(); i++)
    {
	ContactSensor &sensor = m_contactSensors[i];
	msgs::Contacts contacts = sensor.sensor->Contacts();
	
	// Skip to next sensor if no contacts
	if (contacts.contact_size() == 0)
	    continue;
	
	for (size_t j=0; j<contacts.contact_size(); j++)
	{
	    for (size_t k=0; k<contacts.contact(j).position_size(); k++)
	    {
		// Extract position from message
		auto position = contacts.contact(j).position(k);

		// Convert to a pose with no rotation
		ignition::math::Pose3d point(position.x(), position.y(), position.z(),
					     0, 0, 0);

		// Find the vector from the center of the palm of the hand
		// to the contact point expressed in the frame attached to the
		// palm of the hand (as done in the skinManager)
		ignition::math::Pose3d diff = point - inertialToHand;

		
		// One skinContact for each position is considered
		// TODO: averaging may be an idea
		
		// Only geometric center of the contact is considered
		// in this implementation
		ignition::math::Vector3d & diffPos = diff.Pos();
		yarp::sig::Vector diffVector(3, 0.0);
		diffVector[0] = diffPos.X();
		diffVector[1] = diffPos.Y();
		diffVector[2] = diffPos.Z();		
		iCub::skinDynLib::dynContact dynContact(sensor.bodyPart,
						     static_cast<int>(sensor.linkNumber),
						     diffVector);
		iCub::skinDynLib::skinContact skinContact(dynContact);
		skinContact.setSkinPart(sensor.skinPart);

		// Add contact to the list
		skinContactList.push_back(skinContact);
	    }
	}
    }
    // Send data over port
    m_portSkin.write();
}
    
}
