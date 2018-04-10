/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// yarp
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

// gazebo
#include <gazebo/physics/Model.hh>

// ignition
#include <ignition/math/Pose3.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>

// std
#include <string>

#include "ModelReset.hh"

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpModelReset)

namespace gazebo {

GazeboYarpModelReset::~GazeboYarpModelReset()
{
    // Close the rpc server
    m_rpcPort.close();
}

void GazeboYarpModelReset::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpModelReset::Load"
		 << "Error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to model
    m_model = _parent;

    // Store initial position of the model
#if GAZEBO_MAJOR_VERSION >= 8
    m_initialPose = m_model->WorldPose();
#else
    m_initialPose = m_model->GetWorldPose();
#endif

    // Open rpc server port
    std::string portName = "/" + m_model->GetName() + "/model_reset/rpc";
    bool ok = m_rpcPort.open(portName);
    if (!ok)
    {
        yError() << "GazeboYarpModelReset::Load"
		 << "Error: unable to open port"
		 << portName;
	return;
    }

    // Configure callback for rpc
    m_rpcPort.setReader(*this);
}

bool GazeboYarpModelReset::read(yarp::os::ConnectionReader& connection)
{
    // Get request from the connection
    yarp::os::Bottle request;
    bool ok = request.read(connection);
    if (!ok)
    {
	yError() << "GazeboYarpModelReset::read"
		 << "Error: unable to read the rpc request"
		 << "from the incoming connection";
	return false;
    }

    // Process request
    yarp::os::Bottle response;
    processRequest(request, response);

    // Sends the response back
    yarp::os::ConnectionWriter* toSender = connection.getWriter();
    if (toSender == NULL)
    {
	yError() << "GazeboYarpModelReset::read"
		 << "Error: unable to get a ConnectionWriter from the"
		 << "incoming connection";

	return false;
    }
    response.write(*toSender);

    return true;
}

void GazeboYarpModelReset::processRequest(const yarp::os::Bottle &request,
					  yarp::os::Bottle &response)
{
    // Check for empty or invalid requests
    yarp::os::Value requestValue = request.get(0);
    std::string error_text;
    if (requestValue.isNull())
    {
	error_text = "An empty rpc request was received and ignored!";

	yWarning() << "GazeboYarpModelReset::processRequest"
		   << "Warning:"
		   << error_text;

	response.addString(error_text);

	return;
    }
    else if (!requestValue.isString())
    {
	error_text = "An invalid rpc request was received and ignored!";

	yWarning() << "GazeboYarpModelReset::processRequest"
		   << "Warning:"
		   << error_text;

	response.addString(error_text);

	return;
    }

    // Extract the string
    std::string requestString = requestValue.asString();

    // Execute depending on the string content
    if (requestString == "help")
    {
	// write response
	response.addVocab(yarp::os::Vocab::encode("many"));
	response.addString("Available commands:");
	response.addString("- reset-pose");
	response.addString("- help");	
    }
    else if (requestString == "reset-pose")
    {
	// reset pose of the model
	resetModelPose();

	// write response
	response.addString("Pose of model " + m_model->GetName() +
			   " was reset.");
    }
    else
    {
	// write response
	response.addString("Command not recognized.");
    }
}

void GazeboYarpModelReset::resetModelPose()
{
    m_model->SetWorldPose(m_initialPose);
}

}
