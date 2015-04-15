// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleKinematic.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleKinematic::FriExampleKinematic(std::string const& name) : FriExampleAbstract(name){
    this->addOperation("setDesiredQ", &FriExampleKinematic::setDesiredQ, this, RTT::OwnThread);
    this->addOperation("setJointImpedance", &FriExampleKinematic::setJointImpedance, this, RTT::OwnThread);

    this->addOperation("setLambda", &FriExampleKinematic::setLambda, this, RTT::OwnThread);

    qdes.resize(LWRDOF);
    lambda = 0.05;
}

FriExampleKinematic::~FriExampleKinematic(){
}

bool FriExampleKinematic::doStart(){
    sensor_msgs::JointState joint_state_data;
    RTT::FlowStatus joint_state_fs = iport_joint_state.read(joint_state_data);
    if(joint_state_fs == RTT::NewData){
        std::vector<double>::iterator it = qdes.begin();
        BOOST_FOREACH(double d, joint_state_data.position){
            *it = d;
            ++it;
        }
        friStart();
        return true;
    }
    else{
        std::cout << "Cannot read robot position, fail to start" << std::endl;
        return false;
    }
}

void FriExampleKinematic::updateHook(){
    motion_control_msgs::JointVelocities command;
    command.velocities.assign(LWRDOF, 0.0);

    std::vector<double> qerror(LWRDOF, 0.0);
    sensor_msgs::JointState joint_state_data;
    RTT::FlowStatus joint_state_fs = iport_joint_state.read(joint_state_data);

    if(joint_state_fs == RTT::NewData){
        for(unsigned int i = 0; i < LWRDOF; i++){
            qerror[i] = qdes[i] - joint_state_data.position[i];
        }
        double normQerror = 0;
        BOOST_FOREACH(double d, qerror){
            normQerror += d*d;
            normQerror = sqrt(normQerror);
        }
        if(normQerror < 0.1){
            oport_joint_velocities.write(command); 
            return;
        }
        else{
            for(unsigned int i = 0; i < LWRDOF; i++){
                command.velocities[i] = lambda * qerror[i];
            }
            oport_joint_velocities.write(command);
        }
    }
}

void FriExampleKinematic::setDesiredQ(std::vector<double> &qdes_){
    std::vector<double>::iterator it = qdes.begin();
    BOOST_FOREACH(double d, qdes_){
        *it = d;
        ++it;
    }
}

void FriExampleKinematic::setLambda(double l){
    lambda = l;
}

void FriExampleKinematic::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
    if(stiffness.size() != LWRDOF || damping.size() != LWRDOF){
        std::cout << "Wrong vector size, should be " <<  LWRDOF << ", " << LWRDOF << std::endl;
        return;
    }
    else{
        lwr_fri::FriJointImpedance joint_impedance_command;
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_impedance_command.stiffness[i] = stiffness[i];
            joint_impedance_command.damping[i] = damping[i];
        }

        oport_joint_impedance.write(joint_impedance_command);
    }
}

ORO_CREATE_COMPONENT(FriExampleKinematic)

