// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleJacobian.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleJacobian::FriExampleJacobian(std::string const& name) : FriExampleAbstract(name){
    this->addOperation("setJointImpedance", &FriExampleJacobian::setJointImpedance, this, RTT::OwnThread);
    this->addOperation("getFRIJointState", &FriExampleJacobian::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("setT", &FriExampleJacobian::setT, this, RTT::OwnThread);
    torque = 0.0;
    SetToZero(J);

    //Xdes.x=0.5;
    //Xdes.y=0.5;
    //Xdes.z=0.5;
    Kp=6.0;
    Kd=0.1;
    JS.velocity.assign(7,0.0);
}

FriExampleJacobian::~FriExampleJacobian(){
}

bool FriExampleJacobian::doStart(){
    //setting stiffness
    std::cout << "Setting the stiffness and damping" << std::endl;
    std::vector<double> stiff(LWRDOF, 1.0);
    std::vector<double> damp(LWRDOF, 0.0);
    setJointImpedance(stiff, damp);
    friStart();
    return true;
}

void FriExampleJacobian::updateHook(){
    
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
	RTT::FlowStatus jacobian_fs = iport_jacobian.read(J);
	if(jacobian_fs==RTT::NewData){
		motion_control_msgs::JointEfforts joint_eff_command;
        	joint_eff_command.efforts.assign(LWRDOF, 0.0); 
		RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
			if(cartPos_fs==RTT::NewData){
				RTT::FlowStatus joint_state_fs = iport_joint_state.read(JS);
				if(joint_state_fs==RTT::NewData){
					Eigen::MatrixXd Jac(6,7);
					Jac.noalias() = J.data;
					Jac.transposeInPlace();
					Eigen::VectorXd Xerr(3);
					Xerr(0)=0.5-(double)X.position.x;
					Xerr(1)=0.5-(double)X.position.y;
					Xerr(2)=0.5-(double)X.position.z;
					//Eigen::VectorXd Couple(J.columns());
					Eigen::VectorXd Couple(LWRDOF);
					Eigen::VectorXd calcul0(LWRDOF);
					for(int i=0;i<LWRDOF;i++){
						calcul0(i)=(double)Kd*(double)(JS.velocity[i]);
					}
					for(int i=0;i<Jac.rows();i++){
						double results=0;
						for(int j=0;j<3;j++){
							results+=Jac(i,j)*Kp*Xerr(j);
						}
						Couple(i) = results-calcul0(i);
					}
					for(int i=0; i<LWRDOF; i++){
						joint_eff_command.efforts[i] =Couple(i);
						std::cout<<"Couple "<<i<<" = "<<Couple(i)<<std::endl;
					}

        				if(requiresControlMode(30)){
           					oport_joint_efforts.write(joint_eff_command);
        				}
        				oport_joint_position.write(joint_position_command);
				}else{
					std::cout<<"JointState port non lu"<<std::endl;
				}

			}else{
				std::cout<<"Position cartésienne non lue"<<std::endl;
			}
	}else{
		std::cout<<"Pas de Jacobienne lue"<<std::endl;
	}
	

        
    }
    else{
       // std::cout << "No new fri_joint_state data" << std::endl; 
    }
}

void FriExampleJacobian::setT(double t){
    torque = t;
}

void FriExampleJacobian::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
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

void FriExampleJacobian::getFRIJointState(){
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
ORO_CREATE_COMPONENT(FriExampleJacobian)

