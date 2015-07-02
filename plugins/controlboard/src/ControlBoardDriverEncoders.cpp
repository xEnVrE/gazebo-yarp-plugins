/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ControlBoardDriver.h"

using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::getEncoder(int j, double *v) //WORKS
{
		if (v && j >= 0 && j < (int)m_numberOfJoints) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					*v = m_positions[j]-m_zeroPosition[j] + (m_motor_positions[m_joint_idx[j]]-m_motor_zeroPosition[m_joint_idx[j]]);
				else
					*v = m_positions[j]-m_zeroPosition[j];
        return true;
    }
    
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoders(double *encs) //WORKS
{
    if (!encs) return false;
    for (unsigned int i = 0; i < m_numberOfElasticJoints; ++i) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[i] != -1)
					encs[i] = m_positions[i]-m_zeroPosition[i] + (m_motor_positions[m_joint_idx[i]]-m_motor_zeroPosition[m_joint_idx[i]]);
				else
					encs[i]  = m_positions[i]-m_zeroPosition[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncodersTimed(double *encs, double *time)
{
    double my_time = m_lastTimestamp.getTime();
    for (unsigned int i = 0; i < m_numberOfElasticJoints; ++i) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[i] != -1)
					encs[i] = m_positions[i]-m_zeroPosition[i] + (m_motor_positions[m_joint_idx[i]]-m_motor_zeroPosition[m_joint_idx[i]]);
				else
					encs[i]  = m_positions[i]-m_zeroPosition[i];
        time[i] = my_time;
    }

    return true;
}

/**
 * Read the instantaneous acceleration of the specified axis
 * @param j axis index
 * @param encs pointer to double
 * @param time corresponding timestamp (pointer to)
 * @return true if all goes well, false if anything bad happens.
 */
bool GazeboYarpControlBoardDriver::getEncoderTimed(int j, double *encs, double *time)
{
		if (time && encs && j >= 0 && j < (int)m_numberOfJoints) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					*encs = m_positions[j]-m_zeroPosition[j] + (m_motor_positions[m_joint_idx[j]]-m_motor_zeroPosition[m_joint_idx[j]]);
				else
					*encs = m_positions[j]-m_zeroPosition[j];
				
        *time = m_lastTimestamp.getTime();
        return true;
    }
    return false;
}


/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
 */
bool GazeboYarpControlBoardDriver::resetEncoder(int j) //WORKS
{
		if (j >= 0 && j < (int)m_numberOfJoints) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					m_zeroPosition[j] = m_positions[j] + m_motor_positions[m_joint_idx[j]];
				else
					m_zeroPosition[j] = m_positions[j];
        return true;
    }
    
    return false;
}

bool GazeboYarpControlBoardDriver::resetEncoders() //WORKS
{
		for (unsigned int j = 0; j < m_numberOfJoints; j++) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					m_zeroPosition[j] = m_positions[j] + m_motor_positions[m_joint_idx[j]];
				else
					m_zeroPosition[j] = m_positions[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setEncoder(int j, double val) //WORKS
{
		if (j >= 0 && j < (int)m_numberOfJoints) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					m_zeroPosition[j] = m_positions[j] + m_motor_positions[m_joint_idx[j]] - val;
				else
					m_zeroPosition[j] = m_positions[j] - val;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setEncoders(const double *vals) //WORKS
{
		for (unsigned int j = 0; j < m_numberOfJoints; j++) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					m_zeroPosition[j] = m_positions[j] + m_motor_positions[m_joint_idx[j]] - vals[j];
				else
					m_zeroPosition[j] = m_positions[j] - vals[j];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::getEncoderSpeed(int j, double *sp) //NOT TESTED
{
		if (sp && j >= 0 && j < (int)m_numberOfJoints) {
				if(m_numberOfElasticJoints > 0 && m_joint_idx[j] != -1)
					*sp = m_velocities[j] + m_motor_velocities[m_joint_idx[j]];
				else
					*sp = m_velocities[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoderSpeeds(double *spds) //NOT TESTED
{
    if (!spds) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        getEncoderSpeed(i, &spds[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    if (spds && j >= 0 && j < (int)m_numberOfJoints) {
        *spds = 0.0;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (unsigned int i=0; i<m_numberOfJoints; ++i) {
        accs[i] = 0.0;
    }
    return true;
}
