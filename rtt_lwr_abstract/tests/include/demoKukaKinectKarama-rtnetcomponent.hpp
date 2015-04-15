// Filename:  demoKukaKinectKarama-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) from Karama Sriti
// Description: Orocos component using RTNET to control the kuka speed given 
//              the distance between the robot and a human.

#ifndef demoKukaKinectKarama_RTNET_COMPONENT_HPP
#define demoKukaKinectKarama_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include <Eigen/Dense>

#include <movingMean.hpp>

class DemoKukaKinectKaramaRTNET : public FriRTNetExampleAbstract{
    public:
        DemoKukaKinectKaramaRTNET(std::string const& name);

        RTT::InputPort<double> port_distance;

        RTT::OutputPort<double> port_dummy_double;

        void updateHook();

 	bool configureHook();
        bool doStart();

        void setNumObs(unsigned int numObs);

        std::vector<double> m_joint_vel_command;

        MovingMean mean;
        double distance;
        int direction;
};



#endif
