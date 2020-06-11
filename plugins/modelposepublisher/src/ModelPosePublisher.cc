/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>

// ignition
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

// Eigen
#include <Eigen/Dense>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/FrameTransform.h>

// boost
#include <boost/bind.hpp>

#include "ModelPosePublisher.hh"

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpModelPosePublisher)

namespace gazebo {

GazeboYarpModelPosePublisher::~GazeboYarpModelPosePublisher()
{
    // Close the driver
    m_drvTransformClient.close();

    // Close the ports
    m_velocityOutputPort.close();
    m_rawVelocityOutputPort.close();
}

void GazeboYarpModelPosePublisher::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpModelPosePublisher::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the model
    m_model = _parent;

    // Load update period
    if (_sdf->HasElement("period")) {
	    // Get the parameter
	    sdf::ParamPtr periodPtr = _sdf->GetElement("period")->GetValue();

	    // Check if it is a strictly positive double
	    if (!periodPtr->Get<double>(m_period) || m_period <= 0) {
            yError() << "GazeboYarpModelPosePublisher::Load error:"
                     << "parameter 'period' for model"
                     << m_model->GetName() << "should be a strictly positive number";
            return;
	    }
    } else {
	    // Default to 10 ms
	    m_period = 0.01;
    }

    // Load link name
    if (_sdf->HasElement("linkName")) {
        // get the parameter
        std::string linkName;
        sdf::ParamPtr linkNamePtr = _sdf->GetElement("linkName")->GetValue();

        if ((linkNamePtr != nullptr) && (linkNamePtr->Get<std::string>(linkName)) && !(linkName.empty()))
        {
            if (m_model->GetLink(linkName) == nullptr)
            {
                yError() << "GazeboYarpModelPosePublisher::Load error:"
                         << "a valid optional parameter <linkName> = '" + linkName + "' found for model"
                         << m_model->GetName()
                         << ", however the specified link does not exist. The plugin will not be loaded.";

                return;
            }

            m_linkName = linkName;

            yInfo() << "GazeboYarpModelPosePublisher::Load info:"
                    << "a valid optional parameter <linkName> = '" + m_linkName + "' found for model"
                    << m_model->GetName();
        }
    }

    // Prepare properties for the PolyDriver
    yarp::os::Property propTfClient;    
    propTfClient.put("device", "transformClient");
    // The local port depends on the model name that is unique
    // also in the case of multiple insertions of the same model
    // in Gazebo
    propTfClient.put("local", "/" + m_model->GetName() + "/" + m_linkName + "/transformClient");
    propTfClient.put("remote", "/transformServer");
    // This is the update period of the transformClient in ms
    propTfClient.put("period", m_period * 1000);

    // Open the driver and obtain a a IFrameTransform view
    m_tfClient = nullptr;
    bool ok = m_drvTransformClient.open(propTfClient);
    ok = ok && m_drvTransformClient.view(m_tfClient) && m_tfClient != nullptr;

    // Return if the driver open failed
    // or the view retrieval failed
    // or the IFrameTransform pointer is not valid
    if (!ok) {
	    yError() << "GazeboYarpModelPosePublisher::Load error:"
	             << "failure in opening iFrameTransform interface for model"
                 << m_model->GetName();
	    return;
    }

    // Open port for output velocity
    if (!m_velocityOutputPort.open("/" + m_model->GetName() + "/" + m_linkName + "/velocity:o")) {
	    yError() << "GazeboYarpModelPosePublisher::Load error:"
	             << "failure in opening output velocity port for model"
                 << m_model->GetName();
	    return;
    }

    if (!m_rawVelocityOutputPort.open("/" + m_model->GetName() + "/" + m_linkName + "/raw_velocity:o")) {
	    yError() << "GazeboYarpModelPosePublisher::Load error:"
	             << "failure in opening output raw velocity port for model"
                 << m_model->GetName();
	    return;
    }

    // Clear last update time
    m_lastUpdateTime = gazebo::common::Time(0.0);

    // Listen to the update event
    auto worldUpdateBind = boost::bind(&GazeboYarpModelPosePublisher::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

void GazeboYarpModelPosePublisher::PublishTransform()
{
    // Get the current pose of the canonical link of the model    
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d curPose = m_model->GetLink(m_linkName)->WorldPose();
#else
    gazebo::math::Pose curPoseGazebo = m_model->GetLink(m_linkName)->GetWorldPose();
    // Convert to Ignition so that the same interface
    // can be used in the rest of the function
    ignition::math::Pose3d curPose = curPoseGazebo.Ign();
#endif

    // Get the positional and rotational parts
    ignition::math::Vector3d pos = curPose.Pos();
    ignition::math::Quaterniond rot = curPose.Rot();

    // Convert the pose to a homogeneous transformation matrix
    // (the first two arguments can be blank)
    yarp::math::FrameTransform  inertialToModel("", "",
                                                pos.X(),
                                                pos.Y(),
                                                pos.Z(),
                                                rot.X(),
                                                rot.Y(),
                                                rot.Z(),
                                                rot.W());

    // Set a new transform using
    // /inertial for source frame name and
    // /<model_name>/frame for the target frame name
    m_tfClient->setTransform("/" + m_model->GetName() + "/" + m_linkName + "/frame", "/inertial", inertialToModel.toMatrix());

    // Get the current twist of the link of the model
    ignition::math::Vector3d linear_velocity = m_model->GetLink(m_linkName)->RelativeLinearVel();
    ignition::math::Vector3d angular_velocity = m_model->GetLink(m_linkName)->RelativeAngularVel();

    // Send the raw velocity
    {
        yarp::sig::Vector& twist = m_rawVelocityOutputPort.prepare();
        twist.resize(6);

        twist(0) = linear_velocity[0];
        twist(1) = linear_velocity[1];
        twist(2) = linear_velocity[2];
        twist(3) = angular_velocity[0];
        twist(4) = angular_velocity[1];
        twist(5) = angular_velocity[2];

        m_rawVelocityOutputPort.write();
    }

    // Send the velocity
    if (m_lastPoseInitialized)
    {
        ignition::math::Vector3d last_pos = m_lastPose.Pos();

        yarp::sig::Vector& twist = m_velocityOutputPort.prepare();
        twist.resize(6);

        twist(0) = (pos.X() - last_pos.X()) / m_period;
        twist(1) = (pos.Y() - last_pos.Y()) / m_period;
        twist(2) = (pos.Z() - last_pos.Z()) / m_period;

        ignition::math::Vector3d angular_velocity = estimate_angular_velocity(m_lastPose.Rot(), curPose.Rot(), m_period);
        twist(3) = angular_velocity[0];
        twist(4) = angular_velocity[1];
        twist(5) = angular_velocity[2];

        m_velocityOutputPort.write();
    }

    m_lastPoseInitialized = true;
    m_lastPose = curPose;
}

void GazeboYarpModelPosePublisher::OnWorldUpdate()
{
    // Get current time
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time currentTime = m_model->GetWorld()->SimTime();
#else
    gazebo::common::Time currentTime = m_model->GetWorld()->GetSimTime();
#endif

    // Check if an update period is elapsed
    if(currentTime - m_lastUpdateTime >= m_period) {

	    // Store current time for next update
	    m_lastUpdateTime = currentTime;

	    // Publish the transform with the current pose
	    PublishTransform();
    }
}


ignition::math::Vector3d GazeboYarpModelPosePublisher::estimate_angular_velocity(const ignition::math::Quaterniond& initial_rotation, const ignition::math::Quaterniond& final_rotation, const double& elapsed_time)
{
    Eigen::Quaterniond q0(initial_rotation.W(), initial_rotation.X(), initial_rotation.Y(), initial_rotation.Z());

    Eigen::Quaterniond q1(final_rotation.W(), final_rotation.X(), final_rotation.Y(), final_rotation.Z());

    Eigen::Matrix3d relative_rotation = q1.toRotationMatrix() * q0.toRotationMatrix().transpose();

    double acos_argument = (relative_rotation.trace() - 1) / 2.0;

    if (acos_argument < -1.0)
    {
        acos_argument = -1.0;
    }
    else if (acos_argument > 1.0)
    {
        acos_argument = 1.0;
    }
    double theta = std::acos(acos_argument) + std::numeric_limits<double>::min();

    Eigen::Matrix3d w_tensor = 1 / (2 * elapsed_time) * theta / std::sin(theta) * (relative_rotation - relative_rotation.transpose());

    ignition::math::Vector3d angular_velocity;
    angular_velocity[0] = - w_tensor(1, 2);
    angular_velocity[1] = w_tensor(0, 2);
    angular_velocity[2] = - w_tensor(0, 1);

    return angular_velocity;
}

}
