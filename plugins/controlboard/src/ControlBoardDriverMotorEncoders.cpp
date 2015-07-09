/*
* Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
* Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ControlBoardDriver.h"

using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::getMotorEncoder(int j, double *v)  //NOT TESTED
{
    if (v && j >= 0 && j < (int)m_numberOfJoints) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1){
          *v = m_motor_positions[joint_motor_map[j]]-m_motor_zeroPosition[joint_motor_map[j]];
          return true;
        }
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getMotorEncoders(double *encs)  //NOT TESTED
{
    if (!encs) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[i] != -1)
          encs[i] = m_motor_positions[joint_motor_map[i]]-m_motor_zeroPosition[joint_motor_map[i]];
        else
          encs[i]  = 0;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getMotorEncodersTimed(double *encs, double *time)  //NOT TESTED
{
    double my_time = m_lastTimestamp.getTime();
    for (unsigned int i = 0; i <m_numberOfJoints; ++i) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[i] != -1)
          encs[i] = m_motor_positions[joint_motor_map[i]]-m_motor_zeroPosition[joint_motor_map[i]];
        else
          encs[i]  = 0;
        time[i] = my_time;
    }
    
    return true;
}

bool GazeboYarpControlBoardDriver::getMotorEncoderTimed(int j, double *encs, double *time)  //NOT TESTED
{
    if (time && encs && j >= 0 && j < (int)m_numberOfJoints) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1){
          *encs = m_motor_positions[joint_motor_map[j]]-m_motor_zeroPosition[joint_motor_map[j]];
          *time = m_lastTimestamp.getTime();
          return true;
        }
    }
    return false;
}


/**
* Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
*/
bool GazeboYarpControlBoardDriver::resetMotorEncoder(int j)  //NOT TESTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1)
        {
          m_motor_zeroPosition[joint_motor_map[j]] = m_motor_positions[joint_motor_map[j]];
          return true;
        }
    }
    return false;
}

bool GazeboYarpControlBoardDriver::resetMotorEncoders()  //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1)
          m_motor_zeroPosition[joint_motor_map[j]] = m_motor_positions[joint_motor_map[j]];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setMotorEncoder(int j, double val)  //NOT TESTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1)
        {
          m_motor_zeroPosition[joint_motor_map[j]] = m_motor_positions[joint_motor_map[j]] - val;
          return true;
        }
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setMotorEncoders(const double *vals)  //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1)
          m_motor_zeroPosition[joint_motor_map[j]] = m_motor_positions[joint_motor_map[j]] - vals[j];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::getMotorEncoderSpeed(int j, double *sp)  //NOT TESTED
{
    if (sp && j >= 0 && j < (int)m_numberOfJoints) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[j] != -1){
          *sp = m_motor_velocities[joint_motor_map[j]];
          return true;
        }
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getMotorEncoderSpeeds(double *spds) //NOT TESTED
{
    if (!spds) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        if(m_numberOfMotorJoints > 0 && joint_motor_map[i] != -1)
          getMotorEncoderSpeed(i, &spds[i]);
        else
          spds[i] = 0;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getMotorEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    if (spds && j >= 0 && j < (int)m_numberOfJoints) {
        *spds = 0.0;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getMotorEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (unsigned int i=0; i<m_numberOfJoints; ++i) {
        accs[i] = 0.0;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getNumberOfMotorEncoders (int *num)
{
  if(!num) return false;
  
  *num = m_numberOfMotorJoints;
  
  return true;
}

bool GazeboYarpControlBoardDriver::getMotorEncoderCountsPerRevolution (int m, double *cpr) //NOT IMPLEMENTED
{
  return false;
}

bool GazeboYarpControlBoardDriver::setMotorEncoderCountsPerRevolution (int m, const double cpr) //NOT IMPLEMENTED
{
  return false;
}

