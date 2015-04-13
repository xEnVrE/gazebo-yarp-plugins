/*
* Copyright (C) 2007-2014 Istituto Italiano di Tecnologia RBCS, ADVR and iCub Facility
* Authors: Mirko Ferrati, Cheng Fang, Silvio Traversaro
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef GAZEBOYARP_OBJECTS_HH
#define GAZEBOYARP_OBJECTS_HH

#include <gazebo/common/Plugin.hh>
#include <yarp/os/Network.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/contacts.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>

namespace yarp {
    namespace os {
        class Port;
        class Bottle;
    }
}

namespace gazebo
{
    class ObjectsServer;
    
    struct grasp_info
    {
        std::string link_name;
        std::string object_name;
        std::string handle_name;
        std::string filter_name;
        physics::JointPtr joint_attached_to_object;
        physics::JointPtr joint_attached_to_model;
        physics::LinkPtr handle_link;
        std::vector<std::string> collisions_str;
        transport::SubscriberPtr contactSub;
        bool grasped;
    };

    class GazeboYarpObjects : public ModelPlugin
    {
    public:
        GazeboYarpObjects();

        virtual ~GazeboYarpObjects();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void gazeboYarpObjectsLoad(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        virtual bool attach(const std::string& link_name, const std::string& object_name);

        virtual bool detach(const std::string& link_name, const std::string& object_name);

    private:
        void cleanup();
        bool createHandle(std::string link_name);
        
        /// \brief Callback for contact messages from the physics engine.
        void OnContacts(ConstContactsPtr &_msg);
        
        yarp::os::Network m_network;
        std::string m_portName;

        physics::ModelPtr m_model;
        physics::WorldPtr m_world;
        bool attach_impl(std::string link_name, std::string object_name, gazebo::math::Pose touch_point, gazebo::math::Vector3 normal);
        
        transport::NodePtr node;
        //RPC variables
        yarp::os::Port *m_rpcPort;
        ObjectsServer *m_clockServer;
        std::vector<grasp_info> grasps;
        sdf::ElementPtr m_sdf;
    };
}




#endif
