// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleTorqueRTNet.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleTorqueRTNet::FriExampleTorqueRTNet(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addOperation("getFRIJointState", &FriExampleTorqueRTNet::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("setT", &FriExampleTorqueRTNet::setT, this, RTT::OwnThread);
    torque = 0.0;

}

FriExampleTorqueRTNet::~FriExampleTorqueRTNet(){
}

bool FriExampleTorqueRTNet::doStart(){
    //setting stiffness
    std::cout << "Setting the stiffness and damping" << std::endl;
    std::vector<double> stiff(LWRDOF, 100.0);
    std::vector<double> damp(LWRDOF, 0.7);
    fri_joint_state_data.assign(LWRDOF,0.0);
    friStart();
    return true;
}

bool FriExampleTorqueRTNet::configureHook(){
    setPeer("lwr");
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
    return true;
}

void FriExampleTorqueRTNet::updateHook(){

    std::string fri_mode("e_fri_unkown_mode");
    bool fri_cmd_mode = false;
    RTT::FlowStatus fs_event = iport_events.read(fri_mode);
    if (fri_mode == "e_fri_cmd_mode")
        fri_cmd_mode = true;
    else if (fri_mode == "e_fri_mon_mode")
        fri_cmd_mode = false;



    RTT::FlowStatus fs = iport_msr_joint_pos.read(fri_joint_state_data);
    if(fs == RTT::NewData){

        //The controller makes an interpolation between msrJntPos and cmdJntPos
        //to generate a trajectory. However, the distance between them has to be small
        //so that the generated trajectory does not violate velocities limits.
        //So we get current joint position fri_joint_state_data.msrJntPos
        //and send it back to the controller in order to keep a msrJntPos and cmdJntPos close.
        std::vector<double> joint_position_command;
        joint_position_command.assign(LWRDOF, 0.0);
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_position_command[i] = fri_joint_state_data[i];
        }

        std::vector<double> joint_eff_command;

        joint_eff_command.assign(LWRDOF, 0.0); 
        joint_eff_command[2] = torque;

	if (fri_cmd_mode){
            if(requiresControlMode(30)){
                oport_add_joint_trq.write(joint_eff_command);
            }
            oport_joint_position.write(joint_position_command);
        }
	else{
	    //std::cout << "KRC in monitor mode, not writing command" << std::endl;	
	}
    }
    else{
        std::cout << "No new fri_joint_state data" << std::endl; 
    }
}

void FriExampleTorqueRTNet::setT(double t){
    torque = t;
}


void FriExampleTorqueRTNet::getFRIJointState(){
    //lwr_fri::FriJointState fri_joint_state_data;
    RTT::FlowStatus fri_jointStateFS = iport_msr_joint_pos.read(fri_joint_state_data);

    if(fri_jointStateFS == RTT::NewData){
        std::cout << "Measured Joint configuration" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        
    }
}
ORO_CREATE_COMPONENT(FriExampleTorqueRTNet)

