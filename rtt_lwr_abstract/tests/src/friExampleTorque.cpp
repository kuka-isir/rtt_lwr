// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleTorque.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleTorque::FriExampleTorque(std::string const& name) : FriExampleAbstract(name){
    this->addOperation("setJointImpedance", &FriExampleTorque::setJointImpedance, this, RTT::OwnThread);
    this->addOperation("getFRIJointState", &FriExampleTorque::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("setT", &FriExampleTorque::setT, this, RTT::OwnThread);
    torque = 0.0;

}

FriExampleTorque::~FriExampleTorque(){
}

bool FriExampleTorque::doStart(){
    //setting stiffness
    std::cout << "Setting the stiffness and damping" << std::endl;
    std::vector<double> stiff(LWRDOF, 100.0);
    std::vector<double> damp(LWRDOF, 0.7);
    setJointImpedance(stiff, damp);
    friStart();
    return true;
}

void FriExampleTorque::updateHook(){
    RTT::FlowStatus fs = iport_fri_joint_state.read(fri_joint_state_data);
    if(fs == RTT::NewData){

        //The controller makes an interpolation between msrJntPos and cmdJntPos
        //to generate a trajectory. However, the distance between them has to be small
        //so that the generated trajectory does not violate velocities limits.
        //So we get current joint position fri_joint_state_data.msrJntPos
        //and send it back to the controller in order to keep a msrJntPos and cmdJntPos close.
        motion_control_msgs::JointPositions joint_position_command;
        joint_position_command.positions.assign(7, 0.0);
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_position_command.positions[i] = fri_joint_state_data.msrJntPos[i];
        }

        motion_control_msgs::JointEfforts joint_eff_command;

        joint_eff_command.efforts.assign(LWRDOF, 0.0); 
        joint_eff_command.efforts[2] = torque;

        if(requiresControlMode(30)){
            oport_joint_efforts.write(joint_eff_command);
        }
        oport_joint_position.write(joint_position_command);
    }
    else{
        std::cout << "No new fri_joint_state data" << std::endl; 
    }
}

void FriExampleTorque::setT(double t){
    torque = t;
}

void FriExampleTorque::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
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

void FriExampleTorque::getFRIJointState(){
    //lwr_fri::FriJointState fri_joint_state_data;
    RTT::FlowStatus fri_jointStateFS = iport_fri_joint_state.read(fri_joint_state_data);

    if(fri_jointStateFS == RTT::NewData){
        std::cout << "Measured Joint configuration" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration before FRI" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration FRI offset" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPosFriOffset){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Measured Joint torque actuator" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Estimated External Joint torque sensor" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.estExtJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Gravity compensation" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.gravity){
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
}
ORO_CREATE_COMPONENT(FriExampleTorque)

