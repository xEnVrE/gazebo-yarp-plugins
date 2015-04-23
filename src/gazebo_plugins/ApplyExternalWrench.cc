/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Jorhabib Eljaik
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ApplyExternalWrench.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN ( ApplyExternalWrench )

ApplyExternalWrench::ApplyExternalWrench()
{
    this->m_wrenchToApply.force.resize ( 3,0 );
    this->m_wrenchToApply.torque.resize ( 3,0 );
    this->m_duration = 0;
    timeIni = 0;

    // Writing applied force values
     m_forcePort.open("/applyWrench/force");
}
ApplyExternalWrench::~ApplyExternalWrench()
{
    printf ( "*** GazeboYarpApplyExternalWrench closing ***\n" );
    printf ( "Trying to stop rpcThread in ApplyExternalWrench plugin\n" );
    m_rpcThread.stop();
    gazebo::event::Events::DisconnectWorldUpdateBegin ( this->m_updateConnection );
    printf ( "Goodbye from ApplyExternalWrench plugin\n" );
}

void ApplyExternalWrench::UpdateChild()
{
    // Reading apply wrench command
    yarp::os::Bottle tmpBottle;
    yarp::os::Bottle    m_forceBottle;


    // Copying command
    this->m_lock.lock();
    tmpBottle = this->m_rpcThread.getCmd();
    this->m_lock.unlock();

    // initialCmdBottle will contain the initial bottle in RPCServerThread
    static yarp::os::Bottle prevCmdBottle = tmpBottle;

    // Parsing command
    this->m_linkName = tmpBottle.get ( 0 ).asString();
    for ( int i=1; i<4; i++ ) {
        m_wrenchToApply.force[i-1] = tmpBottle.get ( i ).asDouble();
    }
    for ( int i=4; i<7; i++ ) {
        m_wrenchToApply.torque[i-4] = tmpBottle.get ( i ).asDouble();
    }

    m_frequency = tmpBottle.get( 8 ).asDouble();

    std::string fullScopeLinkName = "";
    if(this->m_subscope!="") {
      fullScopeLinkName = std::string ( this->m_modelScope + "::" + this->m_subscope + "::" + this->m_linkName );
      this->m_onLink  = m_myModel->GetLink ( fullScopeLinkName );
    } else {
      this->m_onLink  = m_myModel->GetLink ( this->m_linkName );      
    }
    
//  This piece of code shows the full name (scoped name + link name) of every link in the current loaded model.   
//     gazebo::physics::Link_V tmpLinksVector;
//     tmpLinksVector = m_myModel->GetLinks();   
//     for (int i=0; i<tmpLinksVector.size(); i++)
//       std::cout << tmpLinksVector[i]->GetName() << " ";
//     std::cout << std::endl;
    
    if ( !this->m_onLink ) {
        std::cout << "[ERROR] ApplyWrench plugin: link named " << this->m_linkName<< "not found"<<std::endl;
        return;
    } 

    // Get wrench duration
    this->m_duration = tmpBottle.get ( 7 ).asDouble();

    // Copying command to force and torque Vector3 variables
   // math::Vector3 force ( this->m_wrenchToApply.force[0], this->m_wrenchToApply.force[1], this->m_wrenchToApply.force[2] );
    //math::Vector3 torque ( this->m_wrenchToApply.torque[0], this->m_wrenchToApply.torque[1], this->m_wrenchToApply.torque[2] );

    // Taking duration of the applied force into account
    static bool applying_force_flag = 0;
    // If the rpc command changed update initial time to check duration
    if ( prevCmdBottle != tmpBottle ) {
        timeIni = yarp::os::Time::now();
        prevCmdBottle = tmpBottle;
        applying_force_flag = 1;
    }

    double time_current = yarp::os::Time::now();
    // This has to be done during the specified duration
    if ( applying_force_flag && ( time_current - timeIni ) < this->m_duration ) {
//         printf ( "Applying external force on the robot for %f seconds...\n", this->m_duration );



        double t = time_current - timeIni;
        double t_min = 0;
        double t_max = this->m_duration;
        double t_edge = t_max - (m_duration/5);

        //mapping the range of forces (min-to-max/ max-to-min) to the duration of simulation

        double out_min = 0;
        double out_max = 1.57;

        double rel_y_min = 0;
        double rel_y_max = 0.035;

        double rel_z_min = 0;
        double rel_z_max = 0.015;

        double rel_y;
        double rel_z;

        if(t < t_edge){
         rel_y = rel_y_min + ((rel_y_max - rel_y_min) / (t_edge - t_min)) * (t - t_min);
         rel_z = rel_z_min + ((rel_z_max - rel_z_min) / (t_edge - t_min)) * (t - t_min);
        }
        if(t >= t_edge){rel_y = rel_y_max; rel_z=rel_z_max;}

        double out = out_min + ((out_max - out_min) / (t_max - t_min)) * (t - t_min);

         //generate random noise value between 
        double min = 0;
        double max = 3;
        double noise = min + (rand() % (int)(max - min + 1));
      // printf("%2f\n",rel_z);
           //Apply a time varying sinusoidal force
  
        //decreasing force
  //math::Vector3 force ( this->m_wrenchToApply.force[0]*cos((m_frequency*out))-(noise/10), this->m_wrenchToApply.force[1]*cos((m_frequency*out))-(noise/10), this->m_wrenchToApply.force[2]*cos((m_frequency*out))-noise );
  
       //increasing force
  math::Vector3 force ( this->m_wrenchToApply.force[0]*sin((m_frequency*out))-(noise/10), this->m_wrenchToApply.force[1]*sin((m_frequency*out))-(noise/10), this->m_wrenchToApply.force[2]*sin((m_frequency*out))-noise );
     
        //increasing ramp
   // math::Vector3 force ( this->m_wrenchToApply.force[0]*t/(t_max - t_min)-(noise/10), this->m_wrenchToApply.force[1]*t/(t_max - t_min)-(noise/10), this->m_wrenchToApply.force[2]*t/(t_max - t_min)-noise );
  
  //math::Vector3 force ( this->m_wrenchToApply.force[0], this->m_wrenchToApply.force[1], this->m_wrenchToApply.force[2] );
  math::Vector3 torque ( this->m_wrenchToApply.torque[0], this->m_wrenchToApply.torque[1], this->m_wrenchToApply.torque[2] );

    m_forceBottle.addDouble( force[0] );
    m_forceBottle.addDouble( force[1] );
    m_forceBottle.addDouble( force[2] );

    m_forcePort.write(m_forceBottle);


    //math::Vector3 rel = math::Vector3( 0.075, 0.035, 0.015 ); //hard-coded for collision box of l_foot,r_foot
       math::Vector3 rel = math::Vector3( 0.0, rel_y, rel_z );
   // math::Vector3 rel = math::Vector3( 0.0, 0.035, 0.015);

        this->m_onLink->AddForceAtRelativePosition (force, rel);
        //this->m_onLink->AddForce ( force );
        this->m_onLink->AddTorque ( torque );
        math::Vector3 linkCoGPos = this->m_onLink->GetWorldCoGPose().pos; // Get link's COG position where wrench will be applied
        

       
        //Transformation to visualize the applied wrench at a relative position from the CoG
        math::Quaternion linkCoGRot = this->m_onLink->GetWorldCoGPose().rot;
        math::Matrix4 linkTransform = linkCoGRot.GetAsMatrix4();
        linkTransform.SetTranslate(linkCoGPos);
        math::Vector3 ForceContactPos = linkTransform*rel;

      


        math::Vector3 newZ = force.Normalize(); // normalized force. I want the z axis of the cylinder's reference frame to coincide with my force vector
        math::Vector3 newX = newZ.Cross ( math::Vector3::UnitZ );
        math::Vector3 newY = newZ.Cross ( newX );
        math::Matrix4 rotation = math::Matrix4 ( newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1 );
        math::Quaternion forceOrientation = rotation.GetRotation();
        //math::Pose linkCoGPose ( linkCoGPos - rotation*math::Vector3 ( 0,0,.15 ), forceOrientation );
        math::Pose linkCoGPose ( ForceContactPos - rotation*math::Vector3 ( 0,0,.15 ), forceOrientation );
        msgs::Set ( m_visualMsg.mutable_pose(), linkCoGPose );
        msgs::Set ( m_visualMsg.mutable_material()->mutable_ambient(),common::Color ( 1,0,0,0.3 ) );
        m_visualMsg.set_visible ( 1 );
        m_visPub->Publish ( m_visualMsg );
    }

    if ( applying_force_flag && ( time_current - timeIni ) > this->m_duration ) {
        applying_force_flag = 0;
        m_visualMsg.set_visible ( 0 );
        m_visPub->Publish ( m_visualMsg );
    }

}

void ApplyExternalWrench::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    // Check if yarp network is active;
    if ( !this->m_yarpNet.checkNetwork() ) {
        printf ( "ERROR Yarp Network was not found active in ApplyExternalWrench plugin\n" );
        return;
    }

    // What is the parent name??
    this->m_modelScope = _model->GetScopedName();
    printf ( "Scoped name: %s\n",this->m_modelScope.c_str() );

    // Copy the pointer to the model to access later from UpdateChild
    this->m_myModel = _model;

    bool configuration_loaded = false;

    // Read robot name
    if ( _sdf->HasElement ( "robotNamefromConfigFile" ) ) {
        std::string iniRobotName      = _sdf->Get<std::string> ( "robotNamefromConfigFile" );
        std::string iniRobotNamePath =  gazebo::common::SystemPaths::Instance()->FindFileURI ( iniRobotName );

        if ( iniRobotNamePath != "" && this->m_iniParams.fromConfigFile ( iniRobotNamePath.c_str() ) ) {
            std::cout << "ApplyExternalWrench: Found robotNamefromConfigFile in "<< iniRobotNamePath << std::endl;
            yarp::os::Value robotNameParam = m_iniParams.find ( "gazeboYarpPluginsRobotName" );
            this->robotName = robotNameParam.asString();
            printf ( "ApplyExternalWrench: robotName is %s \n",robotName.c_str() );
            m_rpcThread.setRobotName  ( robotName );
            m_rpcThread.setScopedName ( this->m_modelScope );
            gazebo::physics::Link_V links = _model->GetLinks();
            m_rpcThread.setDefaultLink("root_link");
	    this->m_subscope = retrieveSubscope(links, m_modelScope);
            configuration_loaded = true;
        } else {
            printf ( "ERROR trying to get robot configuration file\n" );
            return;
        }
    } else {
            std::cout << "ApplyExternalWrench: robot name from sdf description will be used!"<<std::endl;
            this->robotName = _model->GetName();
            printf ( "ApplyExternalWrench: robotName is %s \n",robotName.c_str() );
            m_rpcThread.setRobotName ( robotName );
            m_rpcThread.setScopedName ( this->m_modelScope );
            gazebo::physics::Link_V links = _model->GetLinks();
            m_rpcThread.setDefaultLink(links.at(0)->GetName());
            configuration_loaded = true;
    }

    // Starting RPC thread to read desired wrench to be applied
    if ( !m_rpcThread.start() ) {
        printf ( "ERROR: rpcThread did not start correctly\n" );
    }


    /// ############ TRYING TO MODIFY VISUALS #######################################################


    this->m_node = transport::NodePtr ( new gazebo::transport::Node() );

    this->m_node->Init ( _model->GetWorld()->GetName() );
    m_visPub = this->m_node->Advertise<msgs::Visual> ( "~/visual", 10 );

    // Set the visual's name. This should be unique.
    m_visualMsg.set_name ( "__CYLINDER_VISUAL__" );

    // Set the visual's parent. This visual will be attached to the parent
    m_visualMsg.set_parent_name ( _model->GetScopedName() );

    // Create a cylinder
    msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
    geomMsg->set_type ( msgs::Geometry::CYLINDER );
    geomMsg->mutable_cylinder()->set_radius ( 0.01 );
    geomMsg->mutable_cylinder()->set_length ( .30 );

    // Don't cast shadows
    m_visualMsg.set_cast_shadows ( false );

    /// ############ END: TRYING TO MODIFY VISUALS #####################################################


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ApplyExternalWrench::UpdateChild, this ) );
}

std::string ApplyExternalWrench::retrieveSubscope ( gazebo::physics::Link_V& v , std::string scope)
{
    std::string tmpName = v[0]->GetName();
    std::size_t found = tmpName.find_first_of(":");
    if(found!=std::string::npos)
      tmpName = tmpName.substr (0, found);
    else
      tmpName = "";
    return tmpName;
}


}


// ############ RPCServerThread class ###############

void RPCServerThread::setRobotName ( std::string robotName )
{
    this->m_robotName = robotName;
}

void RPCServerThread::setScopedName ( std::string scopedName )
{
    this->m_scopedName = scopedName;
}

void RPCServerThread::setDurationBuffer ( double d )
{
    this->m_durationBuffer = d;
}

double RPCServerThread::getDurationBuffer()
{
    return this->m_durationBuffer;
}

void RPCServerThread::setDefaultLink(const std::string &defaultLink)
{
    this->m_defaultLink = defaultLink;
}

bool RPCServerThread::threadInit()
{

    printf ( "Starting RPCServerThread\n" );
    printf ( "Opening rpc port\n" );
    if ( !m_rpcPort.open ( std::string ( "/"+m_robotName + "/applyExternalWrench/rpc:i" ).c_str() ) ) {
        printf ( "ERROR opening RPC port /applyExternalWrench\n" );
        return false;
    }
    // Default link on which wrenches are applied
    //m_cmd.addString ( this->m_scopedName + "::l_arm" );
    m_cmd.addString ( this->m_defaultLink );
    m_cmd.addDouble ( 0 ); // Force  coord. x
    m_cmd.addDouble ( 0 ); // Force  coord. y
    m_cmd.addDouble ( 0 ); // Force  coord. z
    m_cmd.addDouble ( 0 ); // Torque coord. x
    m_cmd.addDouble ( 0 ); // Torque coord. y
    m_cmd.addDouble ( 0 ); // Torque coord. z
    m_cmd.addDouble ( 0 ); // Wrench duration

    m_cmd.addDouble ( 0 ); // Angular frequency for a time varying force


    this->m_durationBuffer = m_cmd.get ( 8 ).asDouble();

    return true;
}
void RPCServerThread::run()
{
    while ( !isStopping() ) {
        yarp::os::Bottle command;
        m_rpcPort.read ( command,true );
        if ( command.get ( 0 ).asString() == "help" ) {
            this->m_reply.addVocab ( yarp::os::Vocab::encode ( "many" ) );
            this->m_reply.addString ( "Insert a command with the following format:" );
            this->m_reply.addString ( "[link] [force] [torque] [duration] [frequency]" );
            this->m_reply.addString ( "e.g. chest 10 0 0 0 0 0 1 20");
            this->m_reply.addString ( "[link]:     (string) Link ID of the robot as specified in robot's SDF" );
            this->m_reply.addString ( "[force]:    (double x, y, z) Force components in N w.r.t. world reference frame" );
            this->m_reply.addString ( "[torque]:   (double x, y, z) Torque components in N.m w.r.t world reference frame" );
            this->m_reply.addString ( "[duration]: (double) Duration of the applied force in seconds" );
            
            this->m_reply.addString ( "[frequency]: (double) Angular frequency for a time varying sinusoidal force fsin(wt)" );

            this->m_reply.addString ( "Note: The reference frame is the base/root robot frame with x pointing backwards and z upwards.");
            this->m_rpcPort.reply ( this->m_reply );
        } else {
            if ( command.get ( 0 ).isString() \
                    && ( command.get ( 1 ).isDouble() || command.get ( 1 ).isInt() )  && ( command.get ( 2 ).isDouble() || command.get ( 2 ).isInt() ) && ( command.get ( 3 ).isDouble() || command.get ( 3 ).isInt() ) \
                    && ( command.get ( 4 ).isDouble() || command.get ( 4 ).isInt() )  && ( command.get ( 5 ).isDouble() || command.get ( 5 ).isInt() ) && ( command.get ( 6 ).isDouble() || command.get ( 6 ).isInt() )  \
                    && ( command.get ( 7 ).isDouble() || command.get ( 7 ).isInt() )  && ( command.get ( 8 ).isDouble() || command.get ( 8 ).isInt() ) ) {
                this->m_reply.addString ( "[ACK] Correct command format" );
                this->m_rpcPort.reply ( m_reply );
                m_lock.lock();
                m_cmd = command;
                m_lock.unlock();
            } else {
                this->m_reply.clear();
                this->m_reply.addString ( "ERROR: Incorrect command format" );
                this->m_rpcPort.reply ( this->m_reply );
            }
        }
        m_reply.clear();
        command.clear();
    }
}
void RPCServerThread::threadRelease()
{
    yarp::os::Thread::threadRelease();
    m_rpcPort.close();
    printf ( "Goodbye from RPC thread\n" );
}
yarp::os::Bottle RPCServerThread::getCmd()
{
    return m_cmd;
}

void RPCServerThread::onStop()
{
    m_rpcPort.interrupt();
}

