/*
* Copyright (C) 2007-2014 Istituto Italiano di Tecnologia RBCS, ADVR and iCub Facility
* Authors: Silvio Traversaro and Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef GAZEBOYARP_OBJECTS_HH
#define GAZEBOYARP_OBJECTS_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include </gazebo/physics/PhysicsTypes.hh>
#include <yarp/os/Network.h>

namespace yarp {
    namespace os {
        class Port;
        class Bottle;
    }
}

namespace gazebo
{
    class ObjectsServer;

    class GazeboYarpObjects : public ModelPlugin
    {
    public:
        GazeboYarpObjects();

        virtual ~GazeboYarpObjects();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void gazeboYarpObjectsLoad(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        virtual bool createSphere(const std::string& name, const double radius, const double mass);

        virtual bool attach(const std::string& link_name, const std::string& object_name);

        virtual bool deleteObject(const std::string& name);

        virtual bool detach(const std::string& object_name);

        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
        void cleanup();

        yarp::os::Network m_network;
        std::string m_portName;
        std::string object_name_attach,link_name_attach;

        physics::ModelPtr m_model;
        physics::WorldPtr m_world;

        std::map< std::string , std::string > objects_map;
        bool to_attach_0, to_attach_1;

        //RPC variables
        yarp::os::Port *m_rpcPort;
        ObjectsServer *m_clockServer;
        std::map<std::string, physics::JointPtr> joints_attached;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };
}




#endif
