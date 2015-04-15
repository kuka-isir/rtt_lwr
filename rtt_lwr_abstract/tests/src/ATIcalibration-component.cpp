/* ATIcalibration.cpp herits from friRTNetExampleAbstract.cpp             */
/* Author: Guillaume Hamon , hamon@isir.upmc.fr, ISIR-CNRS copyright 2014 */

#include "ATIcalibration-component.hpp"

ATIcalibration::ATIcalibration(std::string const& name) : FriRTNetExampleAbstract(name)
{
 	this->addPort("ATI_i", iport_ATI_values); // gets ATI F/T sensor values
 	this->addPort("ATI_calibration_results", oport_calibration_results); // send the measurement to process sensor calibration
 	this->addPort("BiasOrder_o",oport_bias_order); // Ask ATISensor component to order a bias command to the sensor
	this->addPort("addJntTorque_o",oport_add_joint_torque); // used to perform load compensation
	this->addPort("estExtJntTrq_i",iport_est_ext_joint_torque); // gets the external force/load estimation
	//this->addPort("msrJntTrq_i", iport_msr_joint_torque); // only used for debugging
	this->addPort("ATIforce_o", oport_ati_force);
	this->addPort("KUKAforce_o", oport_kuka_force);
	this->addPort("ATItorque_o", oport_ati_torque);
	this->addPort("KUKAtorque_o", oport_kuka_torque);
 	this->addOperation("setFRIRate", &ATIcalibration::setFRIRate, this, RTT::OwnThread);

 	FRIRate=0.001; // FRI period of 20 ms, can be changed with setFRIRate(double period_ms) function, should match the period in the KRL script
	velocity_limit=0.2;//0.2; //T1: 250mm/s max along the end effector, arbitrary value of 0.2 rad/s at the joints, switching to cartesian impedance mode should optimize this (TO DO)
 	end_calibration=false;
 	t=0;
	n=0;
 	valeurZ.resize(6); // Fx,Fy,Fz,Tx,Ty,Tz
 	valeurX.resize(6);
 	valeurY.resize(6);
 	tf_min.resize(LWRDOF);
 	position1.resize(LWRDOF);
 	position2.resize(LWRDOF);
 	position3.resize(LWRDOF);
	position4.resize(LWRDOF);
	position5.resize(LWRDOF);
 	JState_init.resize(LWRDOF);
 	joints_position_command.resize(LWRDOF);
 	joints_position_command_interp.resize(LWRDOF);
	external_torque.resize(LWRDOF);
	//msr_torque.resize(LWRDOF);
	JState.resize(LWRDOF);

	std::cout << "ATIcalibration constructed !" <<std::endl;
}

ATIcalibration::~ATIcalibration(){
}

bool ATIcalibration::configureHook(){
	//vectors initializations
	double w[7]={0,0,0,1.57,0,-1.57,0};
	position1.assign(&w[0],&w[0]+7);
	w[5]=0;
	position2.assign(&w[0],&w[0]+7);
	w[6]=1.57;
	position3.assign(&w[0],&w[0]+7);

	double w2[7]={0,1,0.5,1,0.5,0.5,0};
	position5.assign(&w2[0],&w2[0]+7);

	for(i=0;i<7;i++)
	{
		std::cout<<position1[i]<<" "<<std::endl;
		position4[i]=0;
	//	msr_torque[i]=0;
        	external_torque[i] = 0;
 	}

 	joints_position_command = position1;
  	std::cout << "ATIcalibration configured !" <<std::endl;
  	return true;
}

bool ATIcalibration::doStart(){

 RTT::FlowStatus joint_state_fs=iport_msr_joint_pos.read(JState_init);
// if(joint_state_fs == RTT::NewData){
 	for(i=0;i<7;i++){
 		tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
 	}
 	tf=tf_min[0];
 	for(i=1;i<7;i++){
		if(tf_min[i]>tf){
			tf=tf_min[i];
		}
 	}
	RTT::FlowStatus estExtTrq_fs=iport_est_ext_joint_torque.read(external_torque);
	if (estExtTrq_fs==RTT::NewData){
		for( i=0;i<7;i++ )
    		{
        		external_torque[i]=external_torque[i];
		}

		oport_add_joint_torque.write(external_torque);
	}
	friStart();
	std::cout << "ATIcalibration started !" <<std::endl;
	//std::cout << "tf= " <<tf<<std::endl;
	return true;
/* }else
 {
	std::cout << "Cannot read robot position, fail to start" << std::endl;
        return false;
 }*/
}

void ATIcalibration::updateHook(){
	RTT::os::TimeService::ticks timestamp = RTT::os::TimeService::Instance()->getTicks();
 	fri_frm_krl = m_fromFRI.get();
 	if(fri_frm_krl.intData[0] == 1){ //command mode

 		RTT::FlowStatus joint_state_fs =iport_msr_joint_pos.read(JState);

		RTT::FlowStatus estExtTrq_fs=iport_est_ext_joint_torque.read(external_torque);
		if (estExtTrq_fs==RTT::NewData){
			for(i=0; i<7; i++ )
    			{
        			external_torque[i]=external_torque[i];
   			}
			oport_add_joint_torque.write(external_torque);
		}

 		if(joint_state_fs == RTT::NewData){
 			if(!end_calibration)
 			{
				if(joints_position_command == position1 && t==tf)
 				{
					// first position reached, saving sensor values
					iport_ATI_values.read(valeurZ);
					std::cout<< "Readings at position 1 (Z) = "<< " Fx= "<< valeurZ[0] <<" Fy= "<< valeurZ[1]<<" Fz= "<< valeurZ[2] << " Tx= "<<valeurZ[3]<<" Ty= "<<valeurZ[4] << " Tz= "<<valeurZ[5] <<std::endl;
					std::cout<< "sending bias order "<<std::endl;
					oport_bias_order.write(true);
					// changing command to position 2
					joints_position_command = position2;
					JState_init=JState;
					t=0;
					for(i=0;i<7;i++){
						tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
					}
					tf=tf_min[0];
		 			for(i=1;i<7;i++){
						if(tf_min[i]>tf){
							tf=tf_min[i];
						}
 					}

 				}else
 				{
					if(joints_position_command == position2 && t == tf)
 					{
						// second position reached, saving sensor values
        					iport_ATI_values.read(valeurX);
						std::cout<< "Readings at position 2 (X) = "<< " Fx= "<< valeurX[0] <<" Fy= "<< valeurX[1]<<" Fz= "<< valeurX[2] << " Tx= "<<valeurX[3]<<" Ty= "<<valeurX[4] << " Tz= "<<valeurX[5] <<std::endl;
        					// changing command to position 3
        					joints_position_command = position3;
						JState_init=JState;
						t=0;
						for(i=0;i<7;i++){
							tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
						}
						tf=tf_min[0];
						for(i=1;i<7;i++){
							if(tf_min[i]>tf){
								tf=tf_min[i];
							}
 						}

 				}else
				{
					if(joints_position_command == position3 && t==tf)
        				{
                				// third position reached, saving sensor values
                				iport_ATI_values.read(valeurY);
                				std::cout<< "valeur lu position 3 (Y) = "<< " Fx= "<< valeurY[0] <<" Fy= "<< valeurY[1]<<" Fz= "<< valeurY[2] << " Tx= "<<valeurY[3]<<" Ty= "<<valeurY[4] << " Tz= "<<valeurY[5] <<std::endl;
                				// end of calibration
						JState_init=JState;
						end_calibration = true;
        				}
				}
 			}
 			}else
 			{ // calibration ended


				if(joints_position_command != position5){

					// calibration ended, send sensor measurements to  ATISensor component which will process sensor's weight compensation computations
					Eigen::MatrixXd results(3,6);
					results.row(0)=Eigen::VectorXd::Map(&valeurZ[0],valeurZ.size());
					results.row(1)=Eigen::VectorXd::Map(&valeurX[0],valeurX.size());
					results.row(2)=Eigen::VectorXd::Map(&valeurY[0],valeurY.size());
					oport_calibration_results.write(results);

					/********* test ****************** goes back to home position, verifying that Fnorm value is close to zero (active weight compensation) */
					joints_position_command = position5;
					JState_init=JState;
					t=0;
					for(i=0;i<7;i++){
						tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
					}
					tf=tf_min[0];
					for(i=1;i<7;i++){
						if(tf_min[i]>tf){
							tf=tf_min[i];
						}
 					}
				}
					/*********** test ****************/
				if (t==tf){
					// home position reached, ending component life
					/********* Test comparaison force estimée / force capteur **********************/
					if (n==500){
						geometry_msgs::Wrench ExtTCPWrench;
						std::vector<double> ATIvalue;
						ATIvalue.resize(6);
						iport_cart_wrench.read(ExtTCPWrench);
						iport_ATI_values.read(ATIvalue);

						std::cout<<"ATI Fx = "<<ATIvalue[0]<<" Kuka Estimation Fx = "<< ExtTCPWrench.force.x<<std::endl;
						std::cout<<"ATI Fy = "<<ATIvalue[1]<<" Kuka Estimation Fy = "<< ExtTCPWrench.force.y<<std::endl;
						std::cout<<"ATI Fz = "<<ATIvalue[2]<<" Kuka Estimation Fz = "<< ExtTCPWrench.force.z<<std::endl;
						std::cout<<"ATI Tx = "<<ATIvalue[3]<<" Kuka Estimation Tx = "<< ExtTCPWrench.torque.x<<std::endl;
						std::cout<<"ATI Ty = "<<ATIvalue[4]<<" Kuka Estimation Ty = "<< ExtTCPWrench.torque.y<<std::endl;
						std::cout<<"ATI Tz = "<<ATIvalue[5]<<" Kuka Estimation Tz = "<< ExtTCPWrench.torque.z<<std::endl;
						std_msgs::Float32 ATIForce, ATITorque, EstForce, EstTorque;
						ATIForce.data=std::sqrt(pow(ATIvalue[0],2)+pow(ATIvalue[1],2)+pow(ATIvalue[2],2));
						ATITorque.data=std::sqrt(pow(ATIvalue[3],2)+pow(ATIvalue[4],2)+pow(ATIvalue[5],2));
						EstForce.data=std::sqrt(pow(ExtTCPWrench.force.x,2)+pow(ExtTCPWrench.force.y,2)+pow(ExtTCPWrench.force.z,2));
                                                EstTorque.data=std::sqrt(pow(ExtTCPWrench.torque.x,2)+pow(ExtTCPWrench.torque.y,2)+pow(ExtTCPWrench.torque.z,2));
						std::cout<<"ATI F = "<<ATIForce.data<<" Kuka Estimation F = "<< EstForce.data<<std::endl;
                                                std::cout<<"ATI T = "<<ATITorque.data<<" Kuka Estimation T = "<< EstTorque.data<<std::endl;
						std::cout<<std::endl;
						oport_ati_force.write(ATIForce);
						oport_ati_torque.write(ATITorque);
						oport_kuka_force.write(EstForce);
						oport_kuka_torque.write(EstTorque);
						n=0;
					}
					n++;
				/*********** fin test ******************/

					//FriRTNetExampleAbstract::stop();
					//return;
				}
 			}
 			// polynomoiale interpolation of 5th degree to perform continuous position, velocity and acceleration
 			for(i=0;i<7;i++){
 				joints_position_command_interp[i]=JState_init[i]+(joints_position_command[i]-JState_init[i])*(10*pow(t/tf,3)-15*pow(t/tf,4)+6*pow(t/tf,5));
 			}

			if(oport_joint_position.connected()){
 				oport_joint_position.write(joints_position_command_interp);
			}

 			t+=FRIRate;
 			if(t>tf)
 			{
				t=tf;
 			}
 		}

 	}
	RTT::Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince( timestamp );
	//std::cout<<"Temps : "<<elapsed<<std::endl;
}

void ATIcalibration::setFRIRate(double period_ms){
	FRIRate=period_ms;
	std::cout << "FRI period set to "<< FRIRate << " ms" <<std::endl;
}


/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ATIcalibration)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ATIcalibration)
