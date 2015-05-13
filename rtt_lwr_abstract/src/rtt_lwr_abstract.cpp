// Copyright 2015 ISIR
// Author: Antoine Hoarau

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
using namespace lwr;

const double RTTLWRAbstract::BDFCoeffs[] = {60/147,-360/147,450/147,-400/147,225/147,-72/147,10/147};

RTTLWRAbstract::RTTLWRAbstract(std::string const& name) : RTT::TaskContext(name),jnt_pos_bdf(BDFORDER),BDFi(0),robot_name("lwr"){
    
    //this->addAttribute("fromKRL", m_fromKRL);
    //this->addAttribute("toKRL", m_toKRL);
    this->addAttribute("robot_name",robot_name);
    
    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
    //this->ports()->addPort("KRL_CMD", port_KRL_CMD).doc("");
    this->ports()->addPort("toKRL",port_ToKRL).doc("");
    this->ports()->addPort("fromKRL",port_FromKRL).doc("");
    
    this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
    this->ports()->addPort("RobotState", port_RobotState).doc("");
    this->ports()->addPort("FRIState", port_FRIState).doc("");
    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
    this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc("");
    this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
    this->ports()->addPort("Jacobian", port_Jacobian).doc("");
    this->ports()->addPort("JointTorque", port_JointTorque).doc("");
    this->ports()->addPort("GravityTorque", port_GravityTorque);
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");

    this->addOperation("getFRIMode", &RTTLWRAbstract::getFRIMode, this, RTT::OwnThread);

    this->addOperation("setControlStrategy", &RTTLWRAbstract::setControlStrategy, this, RTT::OwnThread);

    this->addOperation("friStart", &RTTLWRAbstract::friStart, this, RTT::OwnThread);
    this->addOperation("friStop", &RTTLWRAbstract::friStop, this, RTT::OwnThread);
    this->addOperation("friReset", &RTTLWRAbstract::friReset, this, RTT::OwnThread);
    this->addOperation("stopKrlScript", &RTTLWRAbstract::stopKrlScript, this, RTT::OwnThread);
    this->addOperation("getCartesianPosition", &RTTLWRAbstract::getCartesianPosition, this, RTT::OwnThread);
    this->addOperation("getJointPosition", &RTTLWRAbstract::getJointPosition, this, RTT::OwnThread);
    this->addOperation("getJacobian", &RTTLWRAbstract::getJacobian, this, RTT::OwnThread);
    this->addOperation("getMassMatrix", &RTTLWRAbstract::getMassMatrix, this, RTT::OwnThread);
    this->addOperation("getGravityTorque", &RTTLWRAbstract::getGravityTorque, this, RTT::OwnThread);
    this->addOperation("getJointTorque", &RTTLWRAbstract::getJointTorque, this, RTT::OwnThread);
    this->addOperation("getCartesianWrench", &RTTLWRAbstract::getCartesianWrench, this, RTT::OwnThread);
    
  /*  this->addOperation("sendJointPosition", &RTTLWRAbstract::sendJointPosition, this, RTT::OwnThread);
    this->addOperation("sendJointVelocities", &RTTLWRAbstract::sendJointPosition, this, RTT::OwnThread);
    this->addOperation("sendAddJointTorque", &RTTLWRAbstract::sendAddJointTorque, this, RTT::OwnThread);
    this->addOperation("sendCartesianPose", &RTTLWRAbstract::sendCartesianPose, this, RTT::OwnThread);
    this->addOperation("sendCartesianTwist", &RTTLWRAbstract::sendCartesianTwist, this, RTT::OwnThread);
    this->addOperation("connectOJointPosition", &RTTLWRAbstract::connectOJointPosition, this, RTT::OwnThread);
    this->addOperation("connectOJointVelocities", &RTTLWRAbstract::connectOJointVelocities, this, RTT::OwnThread);
    this->addOperation("connectIRobotState", &RTTLWRAbstract::connectIRobotState, this, RTT::OwnThread);
    this->addOperation("connectIFriState", &RTTLWRAbstract::connectIFriState, this, RTT::OwnThread);
    this->addOperation("connectIMsrJntPos", &RTTLWRAbstract::connectIMsrJntPos, this, RTT::OwnThread);
    this->addOperation("connectICmdJntPos", &RTTLWRAbstract::connectICmdJntPos, this, RTT::OwnThread);
    this->addOperation("connectICmdJntPosFriOffset", &RTTLWRAbstract::connectICmdJntPosFriOffset, this, RTT::OwnThread);
    this->addOperation("connectIMsrCartPos", &RTTLWRAbstract::connectIMsrCartPos, this, RTT::OwnThread);
    this->addOperation("connectICmdCartPos", &RTTLWRAbstract::connectICmdCartPos, this, RTT::OwnThread);
    this->addOperation("connectICmdCartPosFriOffset", &RTTLWRAbstract::connectICmdCartPosFriOffset, this, RTT::OwnThread);
    this->addOperation("connectIMsrJntVel", &RTTLWRAbstract::connectIMsrJntVel, this, RTT::OwnThread);
    this->addOperation("connectIMsrJntTrq", &RTTLWRAbstract::connectIMsrJntTrq, this, RTT::OwnThread);
    this->addOperation("connectIEstExtJntTrq", &RTTLWRAbstract::connectIEstExtJntTrq, this, RTT::OwnThread);
    this->addOperation("connectIEstExtTcpWrench", &RTTLWRAbstract::connectIEstExtTcpWrench, this, RTT::OwnThread);
    this->addOperation("connectIEvents", &RTTLWRAbstract::connectIEvents, this, RTT::OwnThread);
    this->addOperation("connectIMassMatrix", &RTTLWRAbstract::connectIMassMatrix, this, RTT::OwnThread);
    this->addOperation("connectIJacobian", &RTTLWRAbstract::connectIJacobian, this, RTT::OwnThread);
    this->addOperation("connectIGravity", &RTTLWRAbstract::connectIGravity, this, RTT::OwnThread);
    this->addOperation("connectOJointTorque", &RTTLWRAbstract::connectOJointTorque, this, RTT::OwnThread);
    this->addOperation("connectOCartesianPose", &RTTLWRAbstract::connectOCartesianPose, this, RTT::OwnThread);
    this->addOperation("connectOCartesianTwist", &RTTLWRAbstract::connectOCartesianTwist, this, RTT::OwnThread);

    this->addOperation("connectOCartesianWrench", &RTTLWRAbstract::connectOCartesianWrench, this, RTT::OwnThread);
    this->addOperation("connectODesJntImpedance", &RTTLWRAbstract::connectODesJntImpedance, this, RTT::OwnThread);*/
}

RTTLWRAbstract::~RTTLWRAbstract(){
}

bool RTTLWRAbstract::configureHook(){

    jnt_vel_bdf.resize(LBR_MNJ);
    jnt_pos.resize(LBR_MNJ);
    jnt_pos_old.resize(LBR_MNJ);
    jnt_vel.resize(LBR_MNJ);
    jnt_trq.resize(LBR_MNJ);
    jnt_trq_raw.resize(LBR_MNJ);
    jnt_pos_fri_offset.resize(LBR_MNJ);
    M.resize(LBR_MNJ,LBR_MNJ);
    jnt_pos_cmd.resize(LBR_MNJ);
    jnt_trq_cmd.resize(LBR_MNJ);
    J.resize(LBR_MNJ);

    port_JointPositionCommand.setDataSample(jnt_pos_cmd);
    port_JointTorqueCommand.setDataSample(jnt_trq_cmd);
    port_CartesianImpedanceCommand.setDataSample(cart_imp_cmd);
    port_CartesianWrenchCommand.setDataSample(cart_wrench_cmd);
    port_CartesianPositionCommand.setDataSample(cart_pos_cmd);
    port_JointImpedanceCommand.setDataSample(jnt_imp_cmd);
    port_JointPositionCommand.setDataSample(jnt_pos_cmd);
    port_JointTorqueCommand.setDataSample(jnt_trq_cmd);
    port_ToKRL.setDataSample(fri_to_krl);
    //port_KRL_CMD.setDataSample();
    
    //initialize the arrays that will be send to KRL
    for(int i=0; i<FRI_USER_SIZE; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
    
    if(!hasPeer(robot_name))
    {
        RTT::log(RTT::Fatal) << robot_name<<" peer could not be found"<<RTT::endlog();
        return false;
    }
    this->peer = getPeer(robot_name);
    RTT::ConnPolicy policy = RTT::ConnPolicy::data();
    port_CartesianWrench.connectTo(this->peer->getPort("CartesianWrench"),policy);
    port_RobotState.connectTo(this->peer->ports()->getPort("RobotState"),policy);
    port_FRIState.connectTo(this->peer->getPort("FRIState"),policy);
    port_JointVelocity.connectTo(this->peer->getPort("JointVelocity"),policy);
    port_CartesianVelocity.connectTo(this->peer->getPort("CartesianVelocity"),policy);
    port_CartesianPosition.connectTo(this->peer->getPort("CartesianPosition"),policy);
    port_MassMatrix.connectTo(this->peer->getPort("MassMatrix"),policy);
    port_Jacobian.connectTo(this->peer->getPort("Jacobian"),policy);
    port_JointTorque.connectTo(this->peer->getPort("JointTorque"),policy);
    port_GravityTorque.connectTo(this->peer->getPort("GravityTorque"),policy);
    port_JointPosition.connectTo(this->peer->getPort("JointPosition"),policy);
    
    //m_toKRL = peer->attributes()->getAttribute("toKRL");
    //m_fromKRL= peer->attributes()->getAttribute("fromKRL");
    port_ToKRL.connectTo(this->peer->getPort("toKRL"),policy);
    port_FromKRL.connectTo(this->peer->getPort("fromKRL"),policy);
    port_JointTorqueRaw(this->peer->getPort("JointTorqueRaw"),policy);
    port_JointPositionFRIOffset(this->peer->getPort("JointPositionFRIOffset"),policy);
    
    
    bool all_connected = true;
    all_connected &= port_CartesianWrench.connected();
    all_connected &= port_RobotState.connected();
    all_connected &= port_FRIState.connected();
    all_connected &= port_JointVelocity.connected();
    all_connected &= port_CartesianVelocity.connected();
    all_connected &= port_CartesianPosition.connected();
    all_connected &= port_MassMatrix.connected();
    all_connected &= port_Jacobian.connected();
    all_connected &= port_JointTorque.connected();
    all_connected &= port_GravityTorque.connected();
    all_connected &= port_JointPosition.connected();
    all_connected &= port_ToKRL.connected();
    all_connected &= port_FromKRL.connected();
    all_connected &= port_JointTorqueRaw.connected();
    all_connected &= port_JointPositionFRIOffset.connected();

    return all_connected;
}

bool RTTLWRAbstract::startHook(){
    initializeCommand();
    return doStart();
}

bool RTTLWRAbstract::doStart(){
    friStart();
    return true;
}

void RTTLWRAbstract::stopHook(){
    //Reset all commands sent to the fri
    initializeCommand();
    doStop();
}

void RTTLWRAbstract::doStop(){
    friStop();
    //stopKrlScript();
}

void RTTLWRAbstract::cleanupHook(){}

void RTTLWRAbstract::setControlStrategy(int mode){
    switch(mode){
        case 1:
        case 2:
            fri_to_krl.intData[1] = fri_int32_t(10);
            break;
        case 3:
        case 7:
            fri_to_krl.intData[1] = fri_int32_t(30);
            break;
        case 4:
        case 5:
        case 6:
            fri_to_krl.intData[1] = fri_int32_t(20);
            break;
                
        default:
            break;
    }
    port_ToKRL.write(fri_to_krl);
    
    /*if(mode != 1 && mode != 2 && mode != 3 && mode != 4 && mode != 5 && mode != 6 && mode != 7){
        std::cout << "Please set a valid control mode: " << std::endl;
        std::cout << "1: Joint Position" << std::endl;
        std::cout << "2: Joint Velocity" << std::endl;
        std::cout << "3: Joint Torque" << std::endl;
        std::cout << "4: Cartesian Position" << std::endl;
        std::cout << "5: Cartesian Force" << std::endl;
        std::cout << "6: Cartesian Twist" << std::endl;
        std::cout << "7: Joint Impedance" << std::endl; //perform joint impedance control (position + torque compensation of the load)
        return;
    }
    else{
       if (mode == 1 || mode == 2){
            this->
            fri_to_krl.intData[1] = 10;
            controlMode.data = FRI_CTRL_POSITION; // joint position control
        }
        else if (mode == 3 || mode == 7){
            fri_to_krl.intData[1] = 30;
            controlMode.data = 30; // joint impedance control
            control_mode_prop.set(7);
        }
        else if (mode == 4 || mode == 5 || mode == 6){
            fri_to_krl.intData[1] = 20;
            controlMode.data = 20; // cartesian impedance control
        }
        port_ToKRL.write(fri_to_krl);
    }*/
}

void RTTLWRAbstract::setToolKRL(int toolNumber){
	fri_to_krl.intData[2] = toolNumber;
	port_ToKRL.write(fri_to_krl);
}

int RTTLWRAbstract::getToolKRL(){
    port_FromKRL.read(fri_from_krl);
    return fri_from_krl.intData[2];
}

/*bool RTTLWRAbstract::requiresControlMode(int modeRequired){
    if (controlMode == modeRequired){
        return true;
    }
    else{
        std::cout << "Cannot proceed, current control mode is " << controlMode
            << " required control mode is " << modeRequired << std::endl;
        return false;
    }
}*/

FRI_STATE RTTLWRAbstract::getFRIMode(){
    port_FromKRL.read(fri_from_krl);
    switch(fri_from_krl.intData[0]){
        case 1:
            RTT::log(RTT::Info) << "FRI in Command Mode" << RTT::endlog();
            return FRI_STATE_CMD;
        case 2:
            RTT::log(RTT::Info) << "FRI in Monitor Mode" << RTT::endlog();
            return FRI_STATE_MON;
        default:
            RTT::log(RTT::Info) << "Cannot read FRI Mode" << RTT::endlog();
            return FRI_STATE_OFF;
        }
}

void RTTLWRAbstract::friStop(){

    //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=lwr::FRI_STOP;
    port_ToKRL.write(fri_to_krl);

    return;
}

void RTTLWRAbstract::friStart(){

    //Put 1 in $FRI_FRM_INT[1] to trigger fri_start()
    fri_to_krl.intData[0]=lwr::FRI_START;
    port_ToKRL.write(fri_to_krl);

    return;
}

void RTTLWRAbstract::friReset(){
    //initialize the arrays that will be send to KRL
    for(int i=0; i<FRI_USER_SIZE; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }

    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::stopKrlScript(){

    //Put 3 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=lwr::STOP_KRL_SCRIPT;
    port_ToKRL.write(fri_to_krl);
}


void RTTLWRAbstract::initializeCommand(){
    //Get current joint position and set it as desired position
    if (port_JointPositionCommand.connected()){
        Eigen::VectorXd measured_jointPosition;
        measured_jointPosition.resize(LBR_MNJ);
        RTT::FlowStatus measured_jointPosition_fs = port_JointPosition.read(measured_jointPosition);
        if (measured_jointPosition_fs == RTT::NewData){
            port_JointPositionCommand.write(measured_jointPosition);
        }
    }

    if(port_CartesianPositionCommand.connected()){
        //Get cartesian position and set it as desired position
        geometry_msgs::Pose cartPosData;
        RTT::FlowStatus cart_pos_fs = port_CartesianPosition.read(cartPosData);
        if (cart_pos_fs == RTT::NewData){
            port_CartesianPositionCommand.write(cartPosData);
        }
    }

    if(port_CartesianWrenchCommand.connected()){
        //Send 0 torque and force
        port_CartesianWrench.read(cart_wrench);
        geometry_msgs::Wrench cart_wrench_command; // Init at 0.0
        /*cart_wrench_command.force.x = 0.0;
        cart_wrench_command.force.y = 0.0;
        cart_wrench_command.force.z = 0.0;
        cart_wrench_command.torque.x = 0.0;
        cart_wrench_command.torque.y = 0.0;
        cart_wrench_command.torque.z = 0.0;*/

        port_CartesianWrenchCommand.write(cart_wrench_command);
    }

    if (port_JointTorqueCommand.connected()){
        //Send 0 joint torque
        Eigen::VectorXd joint_eff_command;
        joint_eff_command.resize(LBR_MNJ);
        port_JointTorqueCommand.write(joint_eff_command);
    }

    if (port_JointImpedanceCommand.connected()){
        lwr_fri::FriJointImpedance joint_impedance_command;
		for(unsigned int i = 0; i < LBR_MNJ; i++){
            joint_impedance_command.stiffness[i] = 35;
            joint_impedance_command.damping[i] = 2*std::sqrt(35);
		}

        port_JointImpedanceCommand.write(joint_impedance_command);
    }
}

geometry_msgs::Pose RTTLWRAbstract::getCartesianPosition(){
    if (port_CartesianPosition.connected() == false)
        RTT::log(RTT::Warning)<<"port_CartesianPosition not connected"<<RTT::endlog();

    geometry_msgs::Pose  msr_cart_pos;
    port_CartesianPosition.read(msr_cart_pos);
    return msr_cart_pos;
}
Eigen::VectorXd RTTLWRAbstract::getJointPosition(){
    if (port_JointPosition.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointPosition not connected"<<RTT::endlog();

    Eigen::VectorXd joint_position;
    joint_position.resize(LBR_MNJ);
    port_JointPosition.read(joint_position);
    return joint_position;
}

KDL::Jacobian RTTLWRAbstract::getJacobian(){
    if (port_Jacobian.connected() == false)
        RTT::log(RTT::Warning)<<"port_Jacobian not connected"<<RTT::endlog();

	KDL::Jacobian  kuka_jacobian_matrix;
    port_Jacobian.read(kuka_jacobian_matrix);
    return kuka_jacobian_matrix;
}

Eigen::MatrixXd RTTLWRAbstract::getMassMatrix(){
    if (port_MassMatrix.connected() == false)
        RTT::log(RTT::Warning)<<"port_MassMatrix not connected"<<RTT::endlog();

    Eigen::MatrixXd mass_matrix_eigen;
    mass_matrix_eigen.resize(LBR_MNJ,LBR_MNJ);
    port_MassMatrix.read(mass_matrix_eigen);
    return mass_matrix_eigen;
}

Eigen::VectorXd RTTLWRAbstract::getGravityTorque(){
    if (port_GravityTorque.connected() == false)
        RTT::log(RTT::Warning)<<"port_GravityTorque not connected"<<RTT::endlog();

    Eigen::VectorXd gravity_torque;
    gravity_torque.resize(LBR_MNJ);
    port_GravityTorque.read(gravity_torque);
    return gravity_torque;
}

Eigen::VectorXd RTTLWRAbstract::getJointTorque(){
    if (port_JointTorque.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointTorque not connected"<<RTT::endlog();

    Eigen::VectorXd joint_torque;
    joint_torque.resize(LBR_MNJ);
    port_JointTorque.read(joint_torque);
    return joint_torque;
}

geometry_msgs::Wrench RTTLWRAbstract::getCartesianWrench(){
    if (port_CartesianWrench.connected() == false)
        RTT::log(RTT::Warning)<<"port_CartesianWrench not connected"<<RTT::endlog();

    geometry_msgs::Wrench cart_wrench;
    port_CartesianWrench.read(cart_wrench);
    return cart_wrench;
}
/*
bool RTTLWRAbstract::connectIRobotState(){
    assert(peer);
    return iport_robot_state.connectTo(peer->getPort("RobotState"));
}

bool RTTLWRAbstract::connectIFriState(){
    assert(peer);
    return iport_Fri_state.connectTo(peer->getPort("FRIState"));
}

bool RTTLWRAbstract::connectIMsrJntPos(){
    assert(peer);
    return iport_msr_joint_pos.connectTo(peer->getPort("msrJntPos"));
}

bool RTTLWRAbstract::connectICmdJntPos(){
    assert(peer);
    return iport_cmd_joint_pos.connectTo(peer->getPort("cmdJntPos"));
}

bool RTTLWRAbstract::connectICmdJntPosFriOffset(){
    assert(peer);
    return iport_cmd_joint_pos_fri_offset.connectTo(peer->getPort("cmdJntPosFriOffset"));
}

bool RTTLWRAbstract::connectIMsrCartPos(){
    assert(peer);
    return iport_cart_pos.connectTo(peer->getPort("msrCartPos"));
}

bool RTTLWRAbstract::connectICmdCartPos(){
    assert(peer);
    return iport_cmd_cart_pos.connectTo(peer->getPort("cmdCartPos"));
}

bool RTTLWRAbstract::connectICmdCartPosFriOffset(){
    assert(peer);
    return iport_cmd_cart_pos_fri_offset.connectTo(peer->getPort("cmdCartPosFriOffset"));
}

bool RTTLWRAbstract::connectIMsrJntVel(){
    assert(peer);
    return iport_msr_joint_vel.connectTo(peer->getPort("msrJntVel"));
}

bool RTTLWRAbstract::connectIMsrJntTrq(){
    assert(peer);
    return iport_msr_joint_trq.connectTo(peer->getPort("msrJntTrq"));
}

bool RTTLWRAbstract::connectIEstExtJntTrq(){
    assert(peer);
    return iport_est_ext_joint_trq.connectTo(peer->getPort("estExtJntTrq"));
}

bool RTTLWRAbstract::connectIEstExtTcpWrench(){
    assert(peer);
    return iport_cart_wrench.connectTo(peer->getPort("estExtTcpWrench"));
}

bool RTTLWRAbstract::connectIEvents(){
    assert(peer);
    return iport_events.connectTo(peer->getPort("events"));
}

bool RTTLWRAbstract::connectIMassMatrix(){
    assert(peer);
    return iport_mass_matrix.connectTo(peer->getPort("massMatrix_o"));
}

bool RTTLWRAbstract::connectIJacobian(){
    assert(peer);
    return jacobianPort.connectTo(peer->getPort("Jacobian"));
}

bool RTTLWRAbstract::connectIGravity(){
    assert(peer);
    return gravityPort.connectTo(peer->getPort("gravity_o"));
}

bool RTTLWRAbstract::connectOJointPosition(){
    assert(peer);
    return oport_joint_position.connectTo(peer->getPort("desJntPos"));
}

bool RTTLWRAbstract::connectOJointVelocities(){
    assert(peer);
    return oport_joint_velocities.connectTo(peer->getPort("desJntVel"));
}

bool RTTLWRAbstract::connectOJointTorque(){
    assert(peer);
    return oport_add_joint_trq.connectTo(peer->getPort("desAddJntTrq"));
}

bool RTTLWRAbstract::connectOCartesianPose(){
    assert(peer);
    return oport_cartesian_pose.connectTo(peer->getPort("desCartPos"));
}

bool RTTLWRAbstract::connectOCartesianTwist(){
    assert(peer);
    return oport_cartesian_twist.connectTo(peer->getPort("desCartTwist"));
}

bool RTTLWRAbstract::connectOCartesianWrench(){
    assert(peer);
    return oport_cartesian_wrench.connectTo(peer->getPort("desAddTcpWrench"));
}

bool RTTLWRAbstract::connectODesJntImpedance(){
    assert(peer);
    return oport_joint_impedance.connectTo(peer->getPort("desJntImpedance"));
}
*/
/*
void RTTLWRAbstract::disconnectPort(std::string portname){
    RTT::base::PortInterface * p = getPort(portname);
    if (p != NULL){
        p->disconnect();
    }
}

void RTTLWRAbstract::sendJointPosition(Eigen::VectorXd &qdes){
    if (oport_joint_position.connected()){
        if(qdes.size() == LWRDOF){
            oport_joint_position.write(qdes);
        }
        else{
            std::cout << "Input wrong qdes vector size, should be 7" << std::endl;
        }
    }
    return;
}

void RTTLWRAbstract::sendJointVelocities(Eigen::VectorXd &qdotdes){
    if (oport_joint_velocities.connected()){
        if(qdotdes.size() == LWRDOF){
            oport_joint_velocities.write(qdotdes);
        }
        else{
            std::cout << "Input wrong qdotdes vector size, should be 7" << std::endl;
        }
    }
    return;
}

void RTTLWRAbstract::sendAddJointTorque(Eigen::VectorXd &tau){
    if (oport_add_joint_trq.connected()){
        if(tau.size() == LWRDOF){
            oport_add_joint_trq.write(tau);
        }
        else{
            std::cout << "Input wrong tau vector size, should be 7" << std::endl;
        }
    }
    return;
}

void RTTLWRAbstract::sendCartesianPose(Eigen::VectorXd &pose){
    if (oport_add_joint_trq.connected()){
        if(pose.size() == 7){
            geometry_msgs::Pose cart_pose;
            cart_pose.position.x = pose[0];
            cart_pose.position.y = pose[1];
            cart_pose.position.z = pose[2];
            cart_pose.orientation.w = pose[3];
            cart_pose.orientation.x = pose[4];
            cart_pose.orientation.y = pose[5];
            cart_pose.orientation.z = pose[6];
            oport_cartesian_pose.write(cart_pose);
        }
        else{
            std::cout << "Input wrong tau vector size, should be 7" << std::endl;
        }
    }
    return;
}

void RTTLWRAbstract::sendCartesianWrenchCommand(Eigen::VectorXd &twist){
    if (port_CartesianWrenchCommand.connected()){
        if(twist.size() == 6){
            geometry_msgs::Twist cart_twist;
            cart_twist.linear.x = twist[0];
            cart_twist.linear.y = twist[1];
            cart_twist.linear.z = twist[2];
            cart_twist.angular.x = twist[3];
            cart_twist.angular.y = twist[4];
            cart_twist.angular.z = twist[5];
            oport_cartesian_twist.write(cart_twist);
        }
        else{
            std::cout << "Input wrong tau vector size, should be 6" << std::endl;
        }
    }
    return;
}
*/
