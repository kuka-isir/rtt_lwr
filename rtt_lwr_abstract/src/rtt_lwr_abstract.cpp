// Copyright 2015 ISIR
// Author: Antoine Hoarau

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
using namespace lwr;

RTTLWRAbstract::RTTLWRAbstract(std::string const& name) : RTT::TaskContext(name),robot_name("lwr"),jnt_pos_bdf(BDFORDER){
    
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
    
    this->addOperation("sendJointVelocities", &RTTLWRAbstract::sendJointPosition, this, RTT::OwnThread);
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
    port_JointTorqueRaw.connectTo(this->peer->getPort("JointTorqueRaw"),policy);
    port_JointPositionFRIOffset.connectTo(this->peer->getPort("JointPositionFRIOffset"),policy);
    
    
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

void RTTLWRAbstract::setControlStrategy(const unsigned int mode){
    switch(mode){
        case 10:
            fri_to_krl.intData[1] = fri_int32_t(10);
            break;
        case 30:
            fri_to_krl.intData[1] = fri_int32_t(30);
            break;
        case 20:
            fri_to_krl.intData[1] = fri_int32_t(20);
            break;
                
        default:
            break;
    }
    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::setToolKRL(const unsigned int toolNumber){
	fri_to_krl.intData[2] = toolNumber;
	port_ToKRL.write(fri_to_krl);
}

int RTTLWRAbstract::getToolKRL(){
    port_FromKRL.read(fri_from_krl);
    return fri_from_krl.intData[2];
}

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
            joint_impedance_command.stiffness[i] = 250; // Defaults in Kuka manual
            joint_impedance_command.damping[i] = 0.7; // Defaults in Kuka manual
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

void RTTLWRAbstract::getJacobian(KDL::Jacobian& jacobian){
    if (port_Jacobian.connected() == false)
        RTT::log(RTT::Warning)<<"port_Jacobian not connected"<<RTT::endlog();
    port_Jacobian.read(jacobian);
}

void RTTLWRAbstract::getMassMatrix(Eigen::MatrixXd& mass_matrix){
    if (port_MassMatrix.connected() == false)
        RTT::log(RTT::Warning)<<"port_MassMatrix not connected"<<RTT::endlog();
    port_MassMatrix.read(mass_matrix);
}

void RTTLWRAbstract::getGravityTorque(Eigen::VectorXd& gravity_torque){
    if (port_GravityTorque.connected() == false)
        RTT::log(RTT::Warning)<<"port_GravityTorque not connected"<<RTT::endlog();
    port_GravityTorque.read(gravity_torque);
}

void RTTLWRAbstract::getJointTorque(Eigen::VectorXd& joint_torque){
    if (port_JointTorque.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointTorque not connected"<<RTT::endlog();
    port_JointTorque.read(joint_torque);
}

void RTTLWRAbstract::getCartesianWrench(geometry_msgs::Wrench& cart_wrench){
    if (port_CartesianWrench.connected() == false)
        RTT::log(RTT::Warning)<<"port_CartesianWrench not connected"<<RTT::endlog();
    port_CartesianWrench.read(cart_wrench);
}

bool RTTLWRAbstract::sendJointCommand(RTT::OutputPort< Eigen::VectorXd >& port_cmd, const Eigen::VectorXd& jnt_cmd)
{
    if (port_cmd.connected()){
        if(jnt_cmd.rows() == LBR_MNJ){
            port_cmd.write(jnt_cmd);
        }else{
            RTT::log(RTT::Error)<<"Can\'t write to "<<port_cmd.getName()<<", size error : jnt_cmd.rows="<<jnt_cmd.rows()<<", should be LBR_MNJ="<<LBR_MNJ<<RTT::endlog();
            return false;
        }
    }else{
        RTT::log(RTT::Error)<<"Port "<<port_cmd.getName()<<" not connected"<<RTT::endlog();
        return false;
    }
    return true;
}


void RTTLWRAbstract::sendJointPosition(Eigen::VectorXd& joint_position_cmd){
    this->sendJointCommand(this->port_JointPositionCommand,joint_position_cmd);
}

void RTTLWRAbstract::estimateVelocityBDF(unsigned int order,const double dt,const boost::circular_buffer<Eigen::VectorXd>& x_states,Eigen::VectorXd& xd)
{
    using namespace boost::assign;
    
    std::vector<double> coeffs_x;
    double coeff_y;
    
    switch(order){
        case 1:
            coeffs_x += 1.,-1.;
            coeff_y = 1.;
            break;
        case 2:
            coeffs_x += 1.,-4./3.,1./3.;
            coeff_y = 2./3.;
            break;
        case 3:
            coeffs_x += 1.,-18./11.,9./11.,-2./11.;
            coeff_y = 6./11.;
            break;
        case 4:
            coeffs_x += 1.,-48./25.,36./25.,-16./25.,3./25.;
            coeff_y = 12./25.;
            break;
        case 5:
            coeffs_x += 1.,-300./137.,300./137.,-200./137.,75./137.,-12./137.;
            coeff_y = 60./137.;
            break;
        case 6:
            coeffs_x += 1.,-360./147.,450./147.,-400./147.,225./147.,-72./147.,10./147.;
            coeff_y = 60./147.;
            break;
        default:
            return;
    }
            
    for(unsigned int j = 0; j < x_states[0].size() ; ++j) // For each Joint j
    {
        double sum = 0.0;
        unsigned int last_idx = x_states.size();
        for(unsigned int k = 0; k < coeffs_x.size() && k < last_idx; ++k) // For each State k
        {
            sum += coeffs_x[k]*x_states[last_idx-k-1][j];
        }
        xd[j] = sum/(dt*coeff_y);            
    }
}
