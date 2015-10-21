// Copyright 2015 ISIR
// Author: Antoine Hoarau

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
using namespace lwr;

RTTLWRAbstract::RTTLWRAbstract(std::string const& name) : 
RTT::TaskContext(name),
robot_name("lwr"),
root_link("link_0"),
tip_link("link_7"),
gravity_vector(0.,0.,-9.81289),
jnt_pos(LBR_MNJ),
jnt_pos_old(LBR_MNJ),
jnt_vel(LBR_MNJ),
jnt_trq(LBR_MNJ),
jnt_trq_raw(LBR_MNJ),
jnt_pos_fri_offset(LBR_MNJ),
mass(LBR_MNJ,LBR_MNJ),
jnt_pos_cmd(LBR_MNJ),
jnt_trq_cmd(LBR_MNJ),
J_tip_base(LBR_MNJ),
jnt_grav(LBR_MNJ),
use_sim_clock(false),
connect_all_ports_at_startup(true)
{
    this->addProperty("robot_name",robot_name);
    this->addProperty("root_link", root_link).doc("");
    this->addProperty("tip_link", tip_link).doc("");
    this->addProperty("robot_description",robot_description).doc("The URDF description");
    this->addProperty("connect_all_ports_at_startup",connect_all_ports_at_startup);
    this->addProperty("use_sim_clock",use_sim_clock);
    
    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

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
    this->addOperation("getFRIControlMode", &RTTLWRAbstract::getFRIControlMode, this, RTT::OwnThread);
    this->addOperation("getFRIQuality", &RTTLWRAbstract::getFRIQuality, this, RTT::OwnThread);
    this->addOperation("setJointPositionControlMode", &RTTLWRAbstract::setJointPositionControlMode, this, RTT::OwnThread);
    this->addOperation("setJointImpedanceControlMode", &RTTLWRAbstract::setJointImpedanceControlMode, this, RTT::OwnThread);
    this->addOperation("setCartesianImpedanceControlMode", &RTTLWRAbstract::setCartesianImpedanceControlMode, this, RTT::OwnThread);
    this->addOperation("isJointPositionControlMode",&RTTLWRAbstract::isJointPositionControlMode, this, RTT::OwnThread);
    this->addOperation("isJointImpedanceControlMode",&RTTLWRAbstract::isJointImpedanceControlMode, this, RTT::OwnThread);
    this->addOperation("isCartesianImpedanceControlMode",&RTTLWRAbstract::isCartesianImpedanceControlMode, this, RTT::OwnThread);
    
    this->addOperation("connectAllPorts", &RTTLWRAbstract::connectAllPorts, this, RTT::OwnThread);
    this->addOperation("friStart", &RTTLWRAbstract::friStart, this, RTT::OwnThread);
    this->addOperation("friStop", &RTTLWRAbstract::friStop, this, RTT::OwnThread);
    this->addOperation("friReset", &RTTLWRAbstract::friReset, this, RTT::OwnThread);
    this->addOperation("stopKrlScript", &RTTLWRAbstract::stopKrlScript, this, RTT::OwnThread);
    
}

RTTLWRAbstract::~RTTLWRAbstract(){
}

bool RTTLWRAbstract::init(bool connect_all_ports){
    jnt_pos_cmd.setZero();
    jnt_trq_cmd.setZero();
        
    //initialize the arrays that will be send to KRL
    for(int i=0; i<FRI_USER_SIZE; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
    
    std::fill(jnt_imp_cmd.stiffness.begin(),jnt_imp_cmd.stiffness.end(),0.0);
    std::fill(jnt_imp_cmd.damping.begin(),jnt_imp_cmd.damping.end(),0.0);
    
    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,kdl_tree,kdl_chain))
        return false;

    rtt_ros_kdl_tools::printChain(kdl_chain);
    
    for(unsigned int i=0;i<kdl_chain.getNrOfSegments();++i)
    {
        const std::string name = kdl_chain.getSegment(i).getName();
        seg_names_idx.add(name,i+1);
        RTT::log(RTT::Warning) << "Segment " << i << "-> " << name << " idx -> "<< seg_names_idx[name] <<RTT::endlog();
    }
    
    if(use_sim_clock){
        RTT::Logger::Instance()->in(getName());
        RTT::log(RTT::Warning) << "Using ROS Sim Clock" << RTT::endlog();
        //rtt_rosclock::use_ros_clock_topic();
        rtt_rosclock::enable_sim();
        rtt_rosclock::set_sim_clock_activity(this);
    }
    
    id_dyn_solver.reset(new KDL::ChainDynParam(kdl_chain,gravity_vector));
    id_rne_solver.reset(new KDL::ChainIdSolver_RNE(kdl_chain,gravity_vector));
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
    
    jnt_pos_kdl.resize(kdl_chain.getNrOfJoints());
    jnt_vel_kdl.resize(kdl_chain.getNrOfJoints());
    jnt_pos_vel_kdl.resize(kdl_chain.getNrOfSegments());
    jnt_trq_kdl.resize(kdl_chain.getNrOfJoints());
    gravity_kdl.resize(kdl_chain.getNrOfJoints());
    coriolis_kdl.resize(kdl_chain.getNrOfJoints());
    f_ext_kdl.resize(kdl_chain.getNrOfSegments());
    
    std::fill(f_ext_kdl.begin(),f_ext_kdl.end(),KDL::Wrench::Zero());
    SetToZero(jnt_pos_kdl);
    SetToZero(jnt_vel_kdl);
    SetToZero(jnt_pos_vel_kdl);
    SetToZero(gravity_kdl);
    SetToZero(coriolis_kdl);

    // Set the Robot name (lwr or lwr_sim) For other components to know it
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        this->getProvider<rtt_rosparam::ROSParam>("rosparam");
    rosparam->getRelative("robot_name");
    

    RTT::log(RTT::Warning) << "KDL Chain Joints : " << kdl_chain.getNrOfJoints()<< RTT::endlog();
    RTT::log(RTT::Warning) << "KDL Chain Segments : " << kdl_chain.getNrOfSegments()<< RTT::endlog();

    if(connect_all_ports)
        return connectAllPorts(robot_name);
    return true;
}
bool RTTLWRAbstract::getAllComponentRelative()
{
    // Get RosParameters if available
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        this->getProvider<rtt_rosparam::ROSParam>("rosparam");
        
    if(rosparam) {
        const RTT::PropertyBag::Properties &properties =  this->properties()->getProperties();
        for(RTT::PropertyBag::Properties::const_iterator it = properties.begin();
            it != properties.end();++it)
        {
            if(rosparam->getParam(getName() +"/"+(*it)->getName(),(*it)->getName()))
                RTT::log(RTT::Info) << getName() +"/"+(*it)->getName() << " => "<< this->getProperty((*it)->getName())<< RTT::endlog();
        }
    }else{ 
        return false;
    }
    return true;
}

bool RTTLWRAbstract::connectAllPorts(const std::string& robot_name)
{
    if(!hasPeer(robot_name))
    {
        RTT::log(RTT::Fatal) << robot_name<<" peer could not be found, can't connect all ports"<<RTT::endlog();
        return false;
    }
    this->peer = getPeer(robot_name);
    RTT::ConnPolicy policy = RTT::ConnPolicy::data();


    RTT::log(RTT::Info) << "Connectings all ports ..." << RTT::endlog();
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

    //port_JointTorqueRaw.connectTo(this->peer->getPort("JointTorqueRaw"),policy);
    //port_JointPositionFRIOffset.connectTo(this->peer->getPort("JointPositionFRIOffset"),policy);
    
    
    port_CartesianImpedanceCommand.connectTo(this->peer->getPort("CartesianImpedanceCommand"),policy);
    port_CartesianPositionCommand.connectTo(this->peer->getPort("CartesianPositionCommand"),policy);
    port_CartesianWrenchCommand.connectTo(this->peer->getPort("CartesianWrenchCommand"),policy);
    port_JointImpedanceCommand.connectTo(this->peer->getPort("JointImpedanceCommand"),policy);
    port_JointPositionCommand.connectTo(this->peer->getPort("JointPositionCommand"),policy);
    port_JointTorqueCommand.connectTo(this->peer->getPort("JointTorqueCommand"),policy);
    port_ToKRL.connectTo(this->peer->getPort("toKRL"),policy);
    port_FromKRL.connectTo(this->peer->getPort("fromKRL"),policy);

    RTT::log(RTT::Info) << "Setting data samples" << RTT::endlog();

    port_JointPositionCommand.setDataSample(jnt_pos_cmd);
    port_JointTorqueCommand.setDataSample(jnt_trq_cmd);
    port_CartesianImpedanceCommand.setDataSample(cart_imp_cmd);
    port_CartesianWrenchCommand.setDataSample(cart_wrench_cmd);
    port_CartesianPositionCommand.setDataSample(cart_pos_cmd);
    port_JointImpedanceCommand.setDataSample(jnt_imp_cmd);
    port_ToKRL.setDataSample(fri_to_krl);
    
    RTT::log(RTT::Info) << "Verifying connections " << RTT::endlog();
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
    //all_connected &= port_JointTorqueRaw.connected();
    //all_connected &= port_JointPositionFRIOffset.connected();
    
    all_connected &= port_CartesianImpedanceCommand.connected();
    all_connected &= port_CartesianPositionCommand.connected();
    all_connected &= port_CartesianWrenchCommand.connected();
    all_connected &= port_JointImpedanceCommand.connected();
    all_connected &= port_JointPositionCommand.connected();
    all_connected &= port_JointTorqueCommand.connected();
    RTT::log(RTT::Info) << "OK!" <<all_connected<< RTT::endlog();
    return all_connected;
}
bool RTTLWRAbstract::updateState(){
    bool res=true;
    res &= getJointPosition(jnt_pos);
    res &= getJointVelocity(jnt_vel);
    jnt_pos_kdl.data = jnt_pos;
    jnt_vel_kdl.data = jnt_vel;
    jnt_pos_vel_kdl.q.data = jnt_pos;
    jnt_pos_vel_kdl.qdot.data = jnt_vel;
    return res;
}
bool RTTLWRAbstract::startHook(){
    //initializeCommand();
    return doStart();
}

bool RTTLWRAbstract::doStart(){
    friStart();
    return true;
}

void RTTLWRAbstract::stopHook(){
    //Reset all commands sent to the fri
    //initializeCommand();
    doStop();
}

void RTTLWRAbstract::doStop(){
    friStop();
    //stopKrlScript();
}
bool RTTLWRAbstract::isCommandMode()
{
    port_FRIState.readNewest(fri_state);
    return static_cast<FRI_STATE>(fri_state.state)==FRI_STATE_CMD;
}
bool RTTLWRAbstract::isMonitorMode()
{
    port_FRIState.readNewest(fri_state);
    return static_cast<FRI_STATE>(fri_state.state)==FRI_STATE_MON;
}
bool RTTLWRAbstract::isPowerOn()
{
    port_RobotState.readNewest(robot_state);
    return robot_state.power!=0;
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
    return 0;
    if(/*port_FromKRL.connected() == false ||*/ port_FromKRL.readNewest(fri_from_krl) == RTT::NoData)
        return 0;
    return fri_from_krl.intData[2];
}

FRI_STATE RTTLWRAbstract::getFRIMode(){
    return FRI_STATE_CMD;
    if(/*port_FromKRL.connected() == false || */ port_FromKRL.readNewest(fri_from_krl) == RTT::NoData)
        return FRI_STATE_MON;
    FRI_STATE state = static_cast<FRI_STATE>(fri_from_krl.intData[0]);
    switch(state){
        case FRI_STATE_CMD:
            RTT::log(RTT::Info) << "FRI in Command Mode" << RTT::endlog();
            break;
        case FRI_STATE_MON:
            RTT::log(RTT::Info) << "FRI in Monitor Mode" << RTT::endlog();
            break;
        default:
            RTT::log(RTT::Error) << "Cannot read FRI Mode ("<<fri_from_krl.intData[0]<<")" << RTT::endlog();
            return FRI_STATE_MON;
        }
     return state;
}

FRI_QUALITY RTTLWRAbstract::getFRIQuality()
{
    if(port_FRIState.readNewest(fri_state)==RTT::NoData)
        return FRI_QUALITY_UNACCEPTABLE;
    FRI_QUALITY quality = static_cast<FRI_QUALITY>(fri_state.quality);
    switch(quality){
        case FRI_QUALITY_UNACCEPTABLE:
            RTT::log(RTT::Error) << "FRI Quality Unacceptable" << RTT::endlog();
            break;
        case FRI_QUALITY_BAD:
            RTT::log(RTT::Warning) << "FRI Quality Bad" << RTT::endlog();
            break;
        case FRI_QUALITY_OK:
            RTT::log(RTT::Warning) << "FRI Quality OK" << RTT::endlog();
            break;
        case FRI_QUALITY_PERFECT:
            RTT::log(RTT::Warning) << "FRI Quality Perfect" << RTT::endlog();
            break;
        default:
            RTT::log(RTT::Info) << "Cannot read FRI Quality (" <<quality<<")"<< RTT::endlog();
            return FRI_QUALITY_UNACCEPTABLE;
        }
    return quality;
}
FRI_CTRL RTTLWRAbstract::getFRIControlMode()
{
    if(port_RobotState.readNewest(robot_state)==RTT::NoData)
        return FRI_CTRL_OTHER;
    FRI_CTRL control_mode = static_cast<FRI_CTRL>(robot_state.control);
    switch(control_mode){
        case FRI_CTRL_POSITION:
            RTT::log(RTT::Info) << "FRI Joint Position Control Mode" << RTT::endlog();
            break;
        case FRI_CTRL_CART_IMP:
            RTT::log(RTT::Info) << "FRI Cartesian Impedance Control Mode" << RTT::endlog();
            break;
        case FRI_CTRL_JNT_IMP:
            RTT::log(RTT::Info) << "FRI Joint Impedance Control Mode" << RTT::endlog();
            break;
        case FRI_CTRL_OTHER:
            RTT::log(RTT::Warning) << "FRI Other Control Mode" << RTT::endlog();
            break;
        default:
            RTT::log(RTT::Error) << "Cannot read FRI Control mode (" <<control_mode<<")"<< RTT::endlog();
            return FRI_CTRL_OTHER;
        }
    return control_mode;
}

void RTTLWRAbstract::friStop(){

    //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=lwr::FRI_STOP;
    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::friStart(){

    //Put 1 in $FRI_FRM_INT[1] to trigger fri_start()
    fri_to_krl.intData[0]=lwr::FRI_START;
    port_ToKRL.write(fri_to_krl);
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

bool RTTLWRAbstract::getCartesianPosition(geometry_msgs::Pose& cart_position){
    return readData(port_CartesianPosition,cart_position);
}
bool RTTLWRAbstract::getJointPosition(Eigen::VectorXd& joint_position){
    return readData(port_JointPosition,joint_position);
}

bool RTTLWRAbstract::getJacobian(KDL::Jacobian& jacobian){
    return readData(port_Jacobian,jacobian);
}

bool RTTLWRAbstract::getMassMatrix(Eigen::MatrixXd& mass_matrix){
    return readData(port_MassMatrix,mass_matrix);
}

bool RTTLWRAbstract::getGravityTorque(Eigen::VectorXd& gravity_torque){
    return readData(port_GravityTorque,gravity_torque);
}

bool RTTLWRAbstract::getJointTorque(Eigen::VectorXd& joint_torque){
    return readData(port_JointTorque,joint_torque);
}

bool RTTLWRAbstract::getCartesianVelocity(geometry_msgs::Twist& cart_twist){
    return readData(port_CartesianVelocity,cart_twist);
}

bool RTTLWRAbstract::getCartesianWrench(geometry_msgs::Wrench& cart_wrench){
    return readData(port_CartesianWrench,cart_wrench);
}
bool RTTLWRAbstract::getJointVelocity(Eigen::VectorXd& joint_velocity){
    return readData(port_JointVelocity,joint_velocity);
}
bool RTTLWRAbstract::getJointTorqueRaw(Eigen::VectorXd& joint_torque_raw){
    return readData(port_JointTorqueRaw,joint_torque_raw);
}
void RTTLWRAbstract::sendJointImpedance(const lwr_fri::FriJointImpedance& joint_impedance_cmd){
    port_JointImpedanceCommand.write(joint_impedance_cmd);
}

void RTTLWRAbstract::sendJointPosition(const Eigen::VectorXd& joint_position_cmd){
    port_JointPositionCommand.write(joint_position_cmd);
}

void RTTLWRAbstract::sendJointTorque(const Eigen::VectorXd& joint_torque_cmd){
    port_JointTorqueCommand.write(joint_torque_cmd);
}

void RTTLWRAbstract::setJointTorqueControlMode(){
        setJointImpedanceControlMode();
        std::fill(jnt_imp_cmd.stiffness.begin(),jnt_imp_cmd.stiffness.end(),0.0);
        std::fill(jnt_imp_cmd.damping.begin(),jnt_imp_cmd.damping.end(),0.0);
        sendJointImpedance(jnt_imp_cmd);
}

void RTTLWRAbstract::sendCartesianPosition(const geometry_msgs::Pose& cartesian_pose)
{
    port_CartesianPositionCommand.write(cartesian_pose);
}

void RTTLWRAbstract::sendCartesianWrench(const geometry_msgs::Wrench& cartesian_wrench)
{
    port_CartesianWrenchCommand.write(cartesian_wrench);
}

void RTTLWRAbstract::setJointPositionControlMode(){setControlStrategy(10*FRI_CTRL_POSITION);}
void RTTLWRAbstract::setJointImpedanceControlMode(){setControlStrategy(10*FRI_CTRL_JNT_IMP);}
void RTTLWRAbstract::setCartesianImpedanceControlMode(){setControlStrategy(10*FRI_CTRL_CART_IMP);}
bool RTTLWRAbstract::isJointImpedanceControlMode(){ return getFRIControlMode() == FRI_CTRL_JNT_IMP;}
bool RTTLWRAbstract::isCartesianImpedanceControlMode(){ return getFRIControlMode() == FRI_CTRL_CART_IMP;}
bool RTTLWRAbstract::isJointPositionControlMode(){ return getFRIControlMode() == FRI_CTRL_POSITION;}
const unsigned int RTTLWRAbstract::getNrOfJoints()const{return n_joints;}