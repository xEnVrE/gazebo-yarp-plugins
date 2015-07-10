/*
* Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
* Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <cstdio>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


const double RobotPositionTolerance = 0.9;

GazeboYarpControlBoardDriver::GazeboYarpControlBoardDriver() : deviceName("") {}
GazeboYarpControlBoardDriver::~GazeboYarpControlBoardDriver() {}

bool GazeboYarpControlBoardDriver::gazebo_init()
{
    //m_robot = gazebo_pointer_wrapper::getModel();
    // std::cout<<"if this message is the last one you read, m_robot has not been set"<<std::endl;
    //assert is a NOP in release mode. We should change the error handling either with an exception or something else
    assert(m_robot);
    if (!m_robot) return false;

    std::cout<<"Robot Name: "<<m_robot->GetName() <<std::endl;
    std::cout<<"# Joints: "<<m_robot->GetJoints().size() <<std::endl;
    std::cout<<"# Links: "<<m_robot->GetLinks().size() <<std::endl;

    this->m_robotRefreshPeriod = (unsigned)(this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0);
    if (!setJointNames()) return false;

    m_numberOfJoints = m_jointNames.size();
    m_positions.resize(m_numberOfJoints);
    m_zeroPosition.resize(m_numberOfJoints);
    m_referenceVelocities.resize(m_numberOfJoints);
    m_velocities.resize(m_numberOfJoints);
    amp.resize(m_numberOfJoints);
    m_torques.resize(m_numberOfJoints); m_torques.zero();
    m_trajectoryGenerationReferenceSpeed.resize(m_numberOfJoints);
    m_referencePositions.resize(m_numberOfJoints);
    m_trajectoryGenerationReferencePosition.resize(m_numberOfJoints);
    m_trajectoryGenerationReferenceAcceleraton.resize(m_numberOfJoints);
    m_referenceTorques.resize(m_numberOfJoints);
    m_jointLimits.resize(m_numberOfJoints);
    m_positionPIDs.reserve(m_numberOfJoints);
    m_velocityPIDs.reserve(m_numberOfJoints);
    m_impedancePosPDs.reserve(m_numberOfJoints);
    m_torqueOffsett.resize(m_numberOfJoints);
    m_minStiffness.resize(m_numberOfJoints, 0.0);
    m_maxStiffness.resize(m_numberOfJoints, 1000.0);
    m_minDamping.resize(m_numberOfJoints, 0.0);
    m_maxDamping.resize(m_numberOfJoints, 100.0);
    
    m_positionGazeboPIDs.reserve(m_numberOfJoints);
    m_velocityGazeboPIDs.reserve(m_numberOfJoints);
    
    //motor
    m_numberOfMotorJoints = m_motorJointNames.size();
    m_motor_zeroPosition.resize(m_numberOfMotorJoints);
    m_motor_positions.resize(m_numberOfMotorJoints);
    m_motor_velocities.resize(m_numberOfMotorJoints);
    m_motor_torques.resize(m_numberOfMotorJoints);

    setMinMaxPos();
    setMinMaxImpedance();
    setPIDs();
    setGazeboPIDs();
    m_positions.zero();
    m_zeroPosition.zero();
    m_referenceVelocities.zero();
    m_velocities.zero();
    m_trajectoryGenerationReferenceSpeed.zero();
    m_referencePositions.zero();
    m_trajectoryGenerationReferencePosition.zero();
    m_trajectoryGenerationReferenceAcceleraton.zero();
    m_referenceTorques.zero();
    amp = 1; // initially on - ok for simulator
    started = false;
    m_controlMode = new int[m_numberOfJoints];
    m_interactionMode = new int[m_numberOfJoints];
    m_isMotionDone = new bool[m_numberOfJoints];
    m_clock = 0;
    m_torqueOffsett = 0;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
        m_controlMode[j] = VOCAB_CM_POSITION;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
        m_interactionMode[j] = VOCAB_IM_STIFF;
    
    //Motor
    m_motor_positions.zero();
    m_motor_velocities.zero();
    m_motor_torques.zero();
    m_motor_zeroPosition.zero();

    std::cout << "gazebo_init set pid done!" << std::endl;

    this->m_updateConnection
        = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpControlBoardDriver::onUpdate,
                                                                    this, _1));
    m_gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node);
    m_gazeboNode->Init(this->m_robot->GetWorld()->GetName());
    m_jointCommandPublisher = m_gazeboNode->Advertise<gazebo::msgs::JointCmd>(std::string("~/") + this->m_robot->GetName() + "/joint_cmd");

    _T_controller = 1;

    std::stringstream ss(m_pluginParameters.find("initialConfiguration").toString());
    if (!(m_pluginParameters.find("initialConfiguration") == "")) {
        double tmp = 0.0;
        yarp::sig::Vector initial_config(m_numberOfJoints);
        unsigned int counter = 1;
        while (ss >> tmp) {
            if(counter > m_numberOfJoints) {
                std::cout<<"To many element in initial configuration, stopping at element "<<counter<<std::endl;
                break;
            }
            initial_config[counter-1] = tmp;
            m_trajectoryGenerationReferencePosition[counter - 1] = GazeboYarpPlugins::convertRadiansToDegrees(tmp);
            m_referencePositions[counter - 1] = GazeboYarpPlugins::convertRadiansToDegrees(tmp);
            m_positions[counter - 1] = GazeboYarpPlugins::convertRadiansToDegrees(tmp);
            counter++;
        }
        std::cout<<"INITIAL CONFIGURATION IS: "<<initial_config.toString()<<std::endl;

        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
#if GAZEBO_MAJOR_VERSION >= 4
            if(isJointMotor(i))
              m_motorJointPointers[joint_motor_map[i]]->SetPosition(0,initial_config[i]);
            else
              m_jointPointers[i]->SetPosition(0,initial_config[i]);
#else
            gazebo::math::Angle a;
            a.SetFromRadian(initial_config[i]);
            if(isJointMotor(i))
              m_motorJointPointers[joint_motor_map[i]]->SetAngle(0,a);
            else
              m_jointPointers[i]->SetAngle(0,a);
#endif
        }
    }
    return true;
}

void GazeboYarpControlBoardDriver::computeTrajectory(const int j)
{
    if ((m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]) < -RobotPositionTolerance) {
        if (m_trajectoryGenerationReferenceSpeed[j] !=0)
            m_referencePositions[j] += (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
        m_isMotionDone[j] = false;
    } else if ((m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]) > RobotPositionTolerance) {
        if (m_trajectoryGenerationReferenceSpeed[j] != 0)
            m_referencePositions[j]-= (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
        m_isMotionDone[j] = false;
    } else {
        m_referencePositions[j] = m_trajectoryGenerationReferencePosition[j];
        m_isMotionDone[j] = true;
    }
}

void GazeboYarpControlBoardDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    m_clock++;
    
    gazebo::common::Time stepTime = _info.simTime - m_lastTimestamp.getTime();

    if (!started) {//This is a simple way to start with the robot in standing position
        started = true;
        for (unsigned int j = 0; j < m_numberOfJoints; ++j)
            if(isJointMotor(j))
              sendPositionToGazebo (j, m_motor_positions[joint_motor_map[j]],stepTime);
            else
              sendPositionToGazebo (j, m_positions[j],stepTime);
    }

    // Sensing motor position & torque, only if there are motor side joints
    // These values will be added to the joint side
    if(m_numberOfMotorJoints > 0)
    {
      for (unsigned int motor_jnt_cnt = 0; motor_jnt_cnt < m_motorJointPointers.size(); motor_jnt_cnt++) {
  //TODO: consider multi-dof joint ?
          m_motor_positions[motor_jnt_cnt] = m_motorJointPointers[motor_jnt_cnt]->GetAngle (0).Degree();
          m_motor_velocities[motor_jnt_cnt] = GazeboYarpPlugins::convertRadiansToDegrees(m_motorJointPointers[motor_jnt_cnt]->GetVelocity(0));
          m_motor_torques[motor_jnt_cnt] = m_motorJointPointers[motor_jnt_cnt]->GetForce(0u);
      }
    }
    
    // Sensing position & torque
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++) {
//TODO: consider multi-dof joint ?
        if(isJointMotor(jnt_cnt))
        {
          m_positions[jnt_cnt] = m_motor_positions[joint_motor_map[jnt_cnt]] + m_jointPointers[jnt_cnt]->GetAngle (0).Degree();
          m_velocities[jnt_cnt] = m_motor_velocities[joint_motor_map[jnt_cnt]] + GazeboYarpPlugins::convertRadiansToDegrees(m_jointPointers[jnt_cnt]->GetVelocity(0));
          m_torques[jnt_cnt] = m_motor_torques[joint_motor_map[jnt_cnt]] + m_jointPointers[jnt_cnt]->GetForce(0u);
        }
        else
        {
          m_positions[jnt_cnt] = m_jointPointers[jnt_cnt]->GetAngle (0).Degree();
          m_velocities[jnt_cnt] = GazeboYarpPlugins::convertRadiansToDegrees(m_jointPointers[jnt_cnt]->GetVelocity(0));
          m_torques[jnt_cnt] = m_jointPointers[jnt_cnt]->GetForce(0u);
        }
    }

    // Updating timestamp
    m_lastTimestamp.update(_info.simTime.Double());

    //logger.log(m_velocities[2]);

    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        //set pos joint value, set m_referenceVelocities joint value
        if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT)
            && (m_interactionMode[j] == VOCAB_IM_STIFF)) {
            if (m_clock % _T_controller == 0) {
                if (m_controlMode[j] == VOCAB_CM_POSITION) {
                    computeTrajectory(j);
                }
                sendPositionToGazebo(j, m_referencePositions[j],stepTime);
            }
        } else if ((m_controlMode[j] == VOCAB_CM_VELOCITY) && (m_interactionMode[j] == VOCAB_IM_STIFF)) {//set vmo joint value
            if (m_clock % _T_controller == 0) {
                sendVelocityToGazebo(j, m_referenceVelocities[j],stepTime);
            }
        } else if (m_controlMode[j] == VOCAB_CM_TORQUE) {
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_referenceTorques[j],stepTime);
            }
        } else if (m_controlMode[j] == VOCAB_CM_OPENLOOP) {
            //OpenLoop control sends torques to gazebo at this moment.
            //Check if gazebo implements a "motor" entity and change the code accordingly.
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_referenceTorques[j],stepTime);
            }
        } else if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT)
            && (m_interactionMode[j] == VOCAB_IM_COMPLIANT)) {
            if (m_clock % _T_controller == 0) {
                if (m_controlMode[j] == VOCAB_CM_POSITION) {
                    computeTrajectory(j);
                }
                sendImpPositionToGazebo(j, m_referencePositions[j],stepTime);
            }
        }
    }
}

void GazeboYarpControlBoardDriver::setMinMaxPos()
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i) {
        if(isJointMotor(i))
        {
          m_jointLimits[i].max = m_motorJointPointers[joint_motor_map[i]]->GetUpperLimit(0).Degree();
          m_jointLimits[i].min = m_motorJointPointers[joint_motor_map[i]]->GetLowerLimit(0).Degree();
        }
        else
        {
          m_jointLimits[i].max = m_jointPointers[i]->GetUpperLimit(0).Degree();
          m_jointLimits[i].min = m_jointPointers[i]->GetLowerLimit(0).Degree();
        }
    }
}

bool GazeboYarpControlBoardDriver::setJointNames()  //WORKS
{
    bool advanced = false;
    yarp::os::Bottle joint_names_bottle;
    if(m_pluginParameters.findGroup("jointNames").isNull())
    {
      joint_names_bottle = m_pluginParameters.findGroup("jointNamesElastic");
      advanced = true;
      std::cout << "§§§§§§§§§§§ You are using Elastic Joint Names §§§§§§§§§§ \n";
    } else
    {
      joint_names_bottle = m_pluginParameters.findGroup("jointNames");
      std::cout << "§§§§§§§§§§§ You are using Standard Joint Names §§§§§§§§§§ \n";
    }

    if (joint_names_bottle.isNull()) {
        std::cout << "GazeboYarpControlBoardDriver::setJointNames(): Error cannot find jointNames." << std::endl;
        return false;
    }

    int nr_of_joints = joint_names_bottle.size()-1;
    int nr_of_elastic_joints = 0;

    m_jointNames.resize(nr_of_joints);
    m_jointPointers.resize(nr_of_joints);
    joint_motor_map.resize(nr_of_joints, -1);
    
    if(advanced)// count number of elastic joints
    { 
      for (unsigned int i = 0; i < m_jointNames.size(); i++) {
        if(joint_names_bottle.get(i+1).asList()->size() > 1)
          nr_of_elastic_joints++;
      }
    }
    
    m_motorJointNames.resize(nr_of_elastic_joints);
    m_motorJointPointers.resize(nr_of_elastic_joints);

    const gazebo::physics::Joint_V & gazebo_models_joints = m_robot->GetJoints();

    int elastic_joint_idx = 0;
    for (unsigned int i = 0; i < m_jointNames.size(); i++) {
        bool joint_found = false;
        bool elastic_joint = false; //used to check if there is an elastic joint in the current joint or not
        bool elastic_joint_found = false;
        yarp::os::Bottle* joint_bottle;
        std::string controlboard_joint_name;
        std::string controlboard_elastic_joint_name;
        if(advanced)
        {
          joint_bottle = joint_names_bottle.get(i+1).asList();
          controlboard_joint_name = joint_bottle->get(0).asString().c_str();
          if (joint_bottle->size() > 1)
          {
            elastic_joint = true;
            controlboard_elastic_joint_name = joint_bottle->get(1).asString().c_str();
          }
        } else {
          controlboard_joint_name = joint_names_bottle.get(i+1).asString().c_str();
        }
        
        for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            if (GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_joint_name)) {
                joint_found = true;
                m_jointNames[i] = gazebo_joint_name;
                m_jointPointers[i] = this->m_robot->GetJoint(gazebo_joint_name);
            }
        }
        
        if(elastic_joint)
        {
            for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !elastic_joint_found; gazebo_joint++) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            if (GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_elastic_joint_name)) {
                elastic_joint_found = true;
                m_motorJointNames[elastic_joint_idx] = gazebo_joint_name;
                m_motorJointPointers[elastic_joint_idx] = this->m_robot->GetJoint(gazebo_joint_name);
                joint_motor_map[i] = elastic_joint_idx;
                elastic_joint_idx++;
            }
          }
        }

        if (!joint_found || (elastic_joint && !elastic_joint_found)) {
            if(!joint_found)
            {
              yError() << "GazeboYarpControlBoardDriver::setJointNames(): cannot find joint " << m_jointNames[i]
                        << " ( " << i << " of " << nr_of_joints << " ) " << "\n";
              yError() << "jointNames is " << joint_names_bottle.toString() << "\n";
              m_jointNames.resize(0);
              m_jointPointers.resize(0);
            }
            if(elastic_joint && !elastic_joint_found)
            {
              yError() << "GazeboYarpControlBoardDriver::setJointNames(): cannot find joint " << m_motorJointNames[i]
                        << " ( " << i << " of " << nr_of_joints << " ) " << "\n";
              yError() << "elasticJointNames is " << joint_bottle->toString() << "\n";
              m_motorJointNames.resize(0);
              m_motorJointPointers.resize(0);
            }
            return false;
        }
    }
    
    return true;
}

void GazeboYarpControlBoardDriver::setPIDsForGroup(std::string pidGroupName,
                                                  std::vector<GazeboYarpControlBoardDriver::PID>& pids,
                                                  enum PIDFeedbackTerm pidTerms)
{  
    yarp::os::Property prop;
    if (m_pluginParameters.check(pidGroupName.c_str())) {
        std::cout<<"Found PID information in plugin parameters group " << pidGroupName << std::endl;

        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;

            yarp::os::Bottle& pid = m_pluginParameters.findGroup(pidGroupName.c_str()).findGroup(property_name.str().c_str());

            GazeboYarpControlBoardDriver::PID pidValue = {0, 0, 0, -1, -1};
            if (pidTerms & PIDFeedbackTermProportionalTerm)
                pidValue.p = pid.get(1).asDouble();
            if (pidTerms & PIDFeedbackTermDerivativeTerm)
                pidValue.d = pid.get(2).asDouble();
            if (pidTerms & PIDFeedbackTermIntegrativeTerm)
                pidValue.i = pid.get(3).asDouble();

            pidValue.maxInt = pid.get(4).asDouble();
            pidValue.maxOut = pid.get(5).asDouble();


            pids.push_back(pidValue);
            std::cout<<"  P: "<<pidValue.p<<" I: "<<pidValue.i<<" D: "<<pidValue.d<<" maxInt: "<<pidValue.maxInt<<" maxOut: "<<pidValue.maxOut<<std::endl;
        }
        std::cout<<"OK!"<<std::endl;
    } else {
        double default_p = pidTerms & PIDFeedbackTermProportionalTerm ? 500.0 : 0;
        double default_i = pidTerms & PIDFeedbackTermIntegrativeTerm ? 0.1 : 0;
        double default_d = pidTerms & PIDFeedbackTermDerivativeTerm ? 1.0 : 0;
        std::cout<<"PID gain information not found in plugin parameters, using default gains ( "
        <<"P " << default_p << " I " << default_i << " D " << default_d << " )" <<std::endl;
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            GazeboYarpControlBoardDriver::PID pid = {500, 0.1, 1.0, -1, -1};
            pids.push_back(pid);
        }
    }
}

void GazeboYarpControlBoardDriver::setMinMaxImpedance()
{

    yarp::os::Bottle& name_bot = m_pluginParameters.findGroup("WRAPPER").findGroup("networks");
    std::string name = name_bot.get(1).toString();

    yarp::os::Bottle& kin_chain_bot = m_pluginParameters.findGroup(name);
    if (kin_chain_bot.check("min_stiffness")) {
        std::cout<<"min_stiffness param found!"<<std::endl;
        yarp::os::Bottle& min_stiff_bot = kin_chain_bot.findGroup("min_stiffness");
        if(min_stiff_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minStiffness[i] = min_stiff_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No minimum stiffness value found in ini file, default one will be used!"<<std::endl;

    if (kin_chain_bot.check("max_stiffness")) {
        std::cout<<"max_stiffness param found!"<<std::endl;
        yarp::os::Bottle& max_stiff_bot = kin_chain_bot.findGroup("max_stiffness");
        if (max_stiff_bot.size()-1 == m_numberOfJoints) {
            for (unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxStiffness[i] = max_stiff_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No maximum stiffness value found in ini file, default one will be used!"<<std::endl;

    if (kin_chain_bot.check("min_damping")) {
        std::cout<<"min_damping param found!"<<std::endl;
        yarp::os::Bottle& min_damping_bot = kin_chain_bot.findGroup("min_damping");
        if(min_damping_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minDamping[i] = min_damping_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No minimum dampings value found in ini file, default one will be used!"<<std::endl;

    if(kin_chain_bot.check("max_damping")) {
        std::cout<<"max_damping param found!"<<std::endl;
        yarp::os::Bottle& max_damping_bot = kin_chain_bot.findGroup("max_damping");
        if (max_damping_bot.size() - 1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxDamping[i] = max_damping_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No maximum damping value found in ini file, default one will be used!"<<std::endl;

    std::cout<<"min_stiffness: [ "<<m_minStiffness.toString()<<" ]"<<std::endl;
    std::cout<<"max_stiffness: [ "<<m_maxStiffness.toString()<<" ]"<<std::endl;
    std::cout<<"min_damping: [ "<<m_minDamping.toString()<<" ]"<<std::endl;
    std::cout<<"max_damping: [ "<<m_maxDamping.toString()<<" ]"<<std::endl;
}

void GazeboYarpControlBoardDriver::setPIDs()
{
    setPIDsForGroup("GAZEBO_PIDS", m_positionPIDs, PIDFeedbackTermAllTerms);
    setPIDsForGroup("GAZEBO_VELOCITY_PIDS", m_velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
    setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
}

void GazeboYarpControlBoardDriver::setGazeboPIDs()
{
    for(unsigned int i = 0; i < m_numberOfJoints; i++)
    {
      m_positionGazeboPIDs[i].SetPGain(m_positionPIDs[i].p);
      m_positionGazeboPIDs[i].SetIGain(m_positionPIDs[i].i);
      m_positionGazeboPIDs[i].SetDGain(m_positionPIDs[i].d);
      m_positionGazeboPIDs[i].SetIMax(m_positionPIDs[i].maxInt);
      m_positionGazeboPIDs[i].SetIMin(-m_positionPIDs[i].maxInt);
      m_positionGazeboPIDs[i].SetCmdMax(m_positionPIDs[i].maxOut);
      m_positionGazeboPIDs[i].SetCmdMin(-m_positionPIDs[i].maxOut);
      
      m_velocityGazeboPIDs[i].SetPGain(m_velocityPIDs[i].p);
      m_velocityGazeboPIDs[i].SetIGain(m_velocityPIDs[i].i);
      m_velocityGazeboPIDs[i].SetDGain(m_velocityPIDs[i].d);
      m_velocityGazeboPIDs[i].SetIMax(m_velocityPIDs[i].maxInt);
      m_velocityGazeboPIDs[i].SetIMin(-m_velocityPIDs[i].maxInt);
      m_velocityGazeboPIDs[i].SetCmdMax(m_velocityPIDs[i].maxOut);
      m_velocityGazeboPIDs[i].SetCmdMin(-m_velocityPIDs[i].maxOut);
    }
}

bool GazeboYarpControlBoardDriver::sendPositionsToGazebo(Vector &refs, gazebo::common::Time stepTime)
{
    if(stepTime.Double() == 0)
      return false;
    
    for (unsigned int j=0; j<m_numberOfJoints; j++) {
        sendPositionToGazebo(j, refs[j], stepTime);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendPositionToGazebo(int j, double ref, gazebo::common::Time stepTime)
{
    if(stepTime.Double() == 0)
      return false;
    
    double cmd;
    double err;
    if(isJointMotor(j))
      err = m_motorJointPointers[joint_motor_map[j]]->GetAngle(0).Radian() + m_jointPointers[j]->GetAngle(0).Radian() - GazeboYarpPlugins::convertDegreesToRadians(ref);
    else
      err = m_jointPointers[j]->GetAngle(0).Radian() - GazeboYarpPlugins::convertDegreesToRadians(ref);    
    
    cmd = m_positionGazeboPIDs[j].Update(err,stepTime);
    
    if(isJointMotor(j))
      m_motorJointPointers[joint_motor_map[j]]->SetForce(0, cmd);
    else
      m_jointPointers[j]->SetForce(0, cmd);
    
//     // Debug outputs
//     std::cout << stepTime.Double() << std:: endl;
//     std::cout << "PID: " << m_positionPIDs[j].p << " " << m_positionPIDs[j].i << " " << m_positionPIDs[j].d << std:: endl;
//     double pe,ie,de;
//     if(isJointMotor(j))
//     {
//       m_positionGazeboPIDs[j].GetErrors(pe,ie,de);
//       std::cout << "Joint " << m_motorJointNames[joint_motor_map[j]] << cmd << ", err: " << err << ", pidErr: " << pe << "," << ie << "," << de  << std::endl;
//     }
//     else
//     {
//       m_positionGazeboPIDs[j].GetErrors(pe,ie,de);
//       std::cout << "Joint " << m_jointNames[j] << cmd << ", err: " << err << ", pidErr: " << pe << "," << ie << "," << de  << std::endl;
//     }
    
    return true;
}

bool GazeboYarpControlBoardDriver::sendVelocitiesToGazebo(yarp::sig::Vector& refs, gazebo::common::Time stepTime) //NOT TESTED
{
    if(stepTime.Double() == 0)
      return false;
    
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendVelocityToGazebo(j,refs[j],stepTime);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendVelocityToGazebo(int j, double ref, gazebo::common::Time stepTime) //NOT TESTED
{    
    if(stepTime.Double() == 0)
      return false;
    
    double cmd;
    double err;
    if(isJointMotor(j))
      err = m_motorJointPointers[joint_motor_map[j]]->GetVelocity(0) + m_jointPointers[j]->GetVelocity(0) - GazeboYarpPlugins::convertDegreesToRadians(ref);
    else
      err = m_jointPointers[j]->GetVelocity(0) - GazeboYarpPlugins::convertDegreesToRadians(ref);    
    
    cmd = m_velocityGazeboPIDs[j].Update(err,stepTime);
    
    if(isJointMotor(j))
      m_motorJointPointers[joint_motor_map[j]]->SetForce(0, cmd);
    else
      m_jointPointers[j]->SetForce(0, cmd);
    
    return true;
}

bool GazeboYarpControlBoardDriver::sendTorquesToGazebo(yarp::sig::Vector& refs, gazebo::common::Time stepTime) //NOT TESTED
{
    if(stepTime.Double() == 0)
      return false;
      
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendTorqueToGazebo(j,refs[j],stepTime);
    }
    
    return true;
}

bool GazeboYarpControlBoardDriver::sendTorqueToGazebo(const int j,const double ref, gazebo::common::Time stepTime) //NOT TESTED
{
    if(stepTime.Double() == 0)
      return false;
    
    if(isJointMotor(j)) // not implemented yet
      return false;
    else
      m_jointPointers[j]->SetForce(0, ref);
    
    return true;
}

void GazeboYarpControlBoardDriver::sendImpPositionToGazebo ( const int j, const double des, gazebo::common::Time stepTime)
{
    if(j >= 0 && j < m_numberOfJoints) {
        /*
        Here joint positions and speeds are in [deg] and [deg/sec].
        Therefore also stiffness and damping has to be [Nm/deg] and [Nm*sec/deg].
        */
        //std::cout<<"m_velocities"<<j<<" : "<<m_velocities[j]<<std::endl;
        double q = m_positions[j] - m_zeroPosition[j];
        double t_ref = -m_impedancePosPDs[j].p * (q - des) - m_impedancePosPDs[j].d * m_velocities[j] + m_torqueOffsett[j];
        sendTorqueToGazebo(j, t_ref, stepTime);
    }
}

void GazeboYarpControlBoardDriver::sendImpPositionsToGazebo (Vector &dess, gazebo::common::Time stepTime)
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
        sendImpPositionToGazebo(i, dess[i], stepTime);
}

bool GazeboYarpControlBoardDriver::isJointMotor(int j)
{
    if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1)
      return true;
    
    return false;
}

