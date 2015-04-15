// Copyright (C) 2014 ISIR-CNRS
// Author: Guillaume Hamon

#include "friExampleJacobianRTNet.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleJacobianRTNet::FriExampleJacobianRTNet(std::string const& name) : FriRTNetExampleAbstract(name){
    //this->addOperation("setJointImpedance", &FriExampleJacobianRTNet::setJointImpedance, this, RTT::OwnThread);
    //this->addOperation("getFRIJointState", &FriExampleJacobianRTNet::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("setT", &FriExampleJacobianRTNet::setT, this, RTT::OwnThread);
    torque = 0.0;
    SetToZero(J);

    //Xdes.x=0.5;
    //Xdes.y=0.5;
    //Xdes.z=0.5;
    Kp=6.0;
    Kd=0.1;
    Joint_vel.resize(LWRDOF);
}

FriExampleJacobianRTNet::~FriExampleJacobianRTNet(){
}

bool FriExampleJacobianRTNet::doStart(){
    //setting stiffness
    /*std::cout << "Setting the stiffness and damping" << std::endl;
    std::vector<double> stiff(LWRDOF, 1.0);
    std::vector<double> damp(LWRDOF, 0.0);
    setJointImpedance(stiff, damp);*/
    friStart();
    return true;
}

bool FriExampleJacobianRTNet::configureHook(){
    setPeer("lwr");
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
    return true;
}

void FriExampleJacobianRTNet::updateHook(){
    
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


	RTT::FlowStatus jacobian_fs = jacobianPort.read(J);
	if(jacobian_fs==RTT::NewData){
	

		std::vector<double> joint_eff_command;
        	joint_eff_command.assign(LWRDOF, 0.0);

		RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
			if(cartPos_fs==RTT::NewData){
				RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(Joint_vel);
				if(joint_vel_fs==RTT::NewData){
					Eigen::MatrixXd Jac(6,7);
					Jac.noalias() = J.data;
					Jac.transposeInPlace();
					Eigen::VectorXd Xerr(3);
					Xerr(0)=0.5-(double)X.position.x;
					Xerr(1)=0.5-(double)X.position.y;
					Xerr(2)=0.5-(double)X.position.z;
					Eigen::VectorXd Couple(LWRDOF);
					Eigen::VectorXd calcul0(LWRDOF);

					for(int i=0;i<LWRDOF;i++){
						calcul0(i)=(double)Kd*(double)(Joint_vel[i]);
					}


					for(int i=0;i<Jac.rows();i++){
						double results=0;
						for(int j=0;j<3;j++){
							results+=Jac(i,j)*Kp*Xerr(j);
						}
						Couple(i) = results-calcul0(i);
					}
					for(int i=0; i<LWRDOF; i++){
						joint_eff_command[i] =Couple(i);
						std::cout<<"Couple "<<i<<" = "<<Couple(i)<<std::endl;
					}

        				if(requiresControlMode(30)){
						oport_add_joint_trq.write(joint_eff_command);
        				}
        				oport_joint_position.write(joint_position_command);
				}else{
					std::cout<<"JointPosition port non lu"<<std::endl;
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

void FriExampleJacobianRTNet::setT(double t){
    torque = t;
}

/*void FriExampleJacobianRTNet::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
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
}*/

ORO_CREATE_COMPONENT(FriExampleJacobianRTNet)

