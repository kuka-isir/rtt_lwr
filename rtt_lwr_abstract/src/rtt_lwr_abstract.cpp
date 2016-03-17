// Copyright 2015 ISIR
// Author: Antoine Hoarau

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
#include <rtt_ros_kdl_tools/mqueue_connpolicy.hpp>
using namespace lwr;
using namespace KDL;
using namespace Eigen;
using namespace RTT;

RTTLWRAbstract::RTTLWRAbstract(std::string const& name) :
RTT::TaskContext(name),
gravity_vector(0.,0.,-9.81289),
use_sim_time(false),
is_sim(false),
connect_all_ports_at_startup(true)
{
    this->addProperty("robot_name",robot_name);
    this->addProperty("root_link", root_link).doc("");
    this->addProperty("tip_link", tip_link).doc("");
    this->addProperty("gravity_vector", gravity_vector).doc("");
    this->addProperty("robot_description",robot_description).doc("The URDF description");
    this->addProperty("connect_all_ports_at_startup",connect_all_ports_at_startup);
    this->addProperty("use_sim_time",use_sim_time);
    this->addProperty("is_sim",is_sim);

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
    this->addOperation("connectAllPortsMQueue", &RTTLWRAbstract::connectAllPortsMQueue, this, RTT::OwnThread);
    //this->addOperation("connectAllPortsMQueueStream", &RTTLWRAbstract::connectAllPorts, this, RTT::OwnThread);
    this->addOperation("friStart", &RTTLWRAbstract::friStart, this, RTT::OwnThread);
    this->addOperation("friStop", &RTTLWRAbstract::friStop, this, RTT::OwnThread);
    this->addOperation("friReset", &RTTLWRAbstract::friReset, this, RTT::OwnThread);
    this->addOperation("stopKrlScript", &RTTLWRAbstract::stopKrlScript, this, RTT::OwnThread);

}

RTTLWRAbstract::~RTTLWRAbstract(){
}

bool RTTLWRAbstract::init(bool connect_all_ports){
    // Set the Robot name (lwr or lwr_sim) For other components to know it
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        this->getProvider<rtt_rosparam::ROSParam>("rosparam");


    if(rosparam)
    {
        rosparam->getRelative("robot_name");
        rosparam->getRelative("root_link");
        rosparam->getRelative("tip_link");
        rosparam->getRelative("robot_description");
        rosparam->getRelative("gravity_vector");
        rosparam->getRelative("is_sim");
        rosparam->getParam(getName() + "connect_all_ports_at_startup","connect_all_ports_at_startup");
        rosparam->getAbsolute("use_sim_time");
        getAllComponentRelative();
    }else{
        RTT::log(RTT::Error) << "ROS Param could not be loaded "<< RTT::endlog();
        return false;
    }

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

    RTT::log(RTT::Warning) << "KDL Chain Joints : " << kdl_chain.getNrOfJoints()<< RTT::endlog();
    RTT::log(RTT::Warning) << "KDL Chain Segments : " << kdl_chain.getNrOfSegments()<< RTT::endlog();

    try{
        if(use_sim_time){
            RTT::Logger::Instance()->in(getName());
            RTT::log(RTT::Warning) << "Using ROS Sim Clock" << RTT::endlog();
            //rtt_rosclock::use_ros_clock_topic();
            //rtt_rosclock::enable_sim();
            //rtt_rosclock::set_sim_clock_activity(this);
        }
    }catch(const std::exception& e)
    {
        log(Warning) << e.what() << endlog();
    }

    id_dyn_solver.reset(new KDL::ChainDynParam(kdl_chain,gravity_vector));
    id_rne_solver.reset(new KDL::ChainIdSolver_RNE(kdl_chain,gravity_vector));
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));

    jnt_pos_kdl.resize(kdl_chain.getNrOfJoints());
    jnt_vel_kdl.resize(kdl_chain.getNrOfJoints());
    jnt_acc_kdl.resize(kdl_chain.getNrOfJoints());
    jnt_pos_vel_kdl.resize(kdl_chain.getNrOfSegments());
    jnt_trq_kdl.resize(kdl_chain.getNrOfJoints());
    gravity_kdl.resize(kdl_chain.getNrOfJoints());
    coriolis_kdl.resize(kdl_chain.getNrOfJoints());
    f_ext_kdl.resize(kdl_chain.getNrOfSegments());

    jnt_pos.resize(kdl_chain.getNrOfJoints());
    jnt_pos_old.resize(kdl_chain.getNrOfJoints());
    jnt_trq.resize(kdl_chain.getNrOfJoints());
    jnt_trq_raw.resize(kdl_chain.getNrOfJoints());
    jnt_pos_fri_offset.resize(kdl_chain.getNrOfJoints());
    jnt_grav.resize(kdl_chain.getNrOfJoints());
    jnt_vel.resize(kdl_chain.getNrOfJoints());
    jnt_pos_cmd.resize(kdl_chain.getNrOfJoints());
    jnt_trq_cmd.resize(kdl_chain.getNrOfJoints());

    std::fill(f_ext_kdl.begin(),f_ext_kdl.end(),KDL::Wrench::Zero());
    SetToZero(jnt_pos_kdl);
    SetToZero(jnt_vel_kdl);
    SetToZero(jnt_acc_kdl);
    SetToZero(jnt_pos_vel_kdl);
    SetToZero(gravity_kdl);
    SetToZero(coriolis_kdl);

    jnt_pos_cmd.setZero();
    jnt_trq_cmd.setZero();
    port_JointPositionCommand.setDataSample(jnt_pos_cmd);
    port_JointTorqueCommand.setDataSample(jnt_trq_cmd);
    port_CartesianImpedanceCommand.setDataSample(cart_imp_cmd);
    port_CartesianWrenchCommand.setDataSample(cart_wrench_cmd);
    port_CartesianPositionCommand.setDataSample(cart_pos_cmd);
    port_JointImpedanceCommand.setDataSample(jnt_imp_cmd);
    port_ToKRL.setDataSample(fri_to_krl);

    // Use connectAllPorts("lwr","mycontroller",ConnPolicy()) in ops
    if(false && connect_all_ports || connect_all_ports_at_startup)
        if(!connectAllPorts(robot_name))
            connectAllPortsMQueue(robot_name);

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
            else
                RTT::log(RTT::Warning) << "No param found for "<<getName() +"/"+(*it)->getName()<< RTT::endlog();
        }
    }else{
        RTT::log(RTT::Error) << "ROS Param could not be loaded "<< RTT::endlog();
        return false;
    }
    return true;
}
bool RTTLWRAbstract::connectPorts(const std::string& robot_name,RTT::ConnPolicy cp)
{
    if(!hasPeer(robot_name))
    {
        RTT::log(RTT::Fatal) << robot_name<<" peer could not be found, can't connect all ports"<<RTT::endlog();
        return false;
    }
    RTT::TaskContext * peer = NULL;
    peer = getPeer(robot_name);
    if(peer == NULL) return false;

    RTT::log(RTT::Info) << "Connectings all ports ..." << RTT::endlog();
    bool all_connected = true;

    all_connected &=port_CartesianWrench.connectTo(peer->getPort("CartesianWrench"),cp);
    all_connected &=port_RobotState.connectTo(peer->ports()->getPort("RobotState"),cp);
    all_connected &=port_FRIState.connectTo(peer->getPort("FRIState"),cp);
    all_connected &=port_JointVelocity.connectTo(peer->getPort("JointVelocity"),cp);
    all_connected &=port_CartesianVelocity.connectTo(peer->getPort("CartesianVelocity"),cp);
    all_connected &=port_CartesianPosition.connectTo(peer->getPort("CartesianPosition"),cp);
    all_connected &=port_MassMatrix.connectTo(peer->getPort("MassMatrix"),cp);
    all_connected &=port_Jacobian.connectTo(peer->getPort("Jacobian"),cp);
    all_connected &=port_JointTorque.connectTo(peer->getPort("JointTorque"),cp);
    all_connected &=port_GravityTorque.connectTo(peer->getPort("GravityTorque"),cp);
    all_connected &=port_JointPosition.connectTo(peer->getPort("JointPosition"),cp);
    all_connected &=port_FromKRL.connectTo(peer->getPort("fromKRL"),cp);

    all_connected &=port_CartesianImpedanceCommand.connectTo(peer->getPort("CartesianImpedanceCommand"),cp);
    all_connected &=port_CartesianPositionCommand.connectTo(peer->getPort("CartesianPositionCommand"),cp);
    all_connected &=port_CartesianWrenchCommand.connectTo(peer->getPort("CartesianWrenchCommand"),cp);
    all_connected &=port_JointImpedanceCommand.connectTo(peer->getPort("JointImpedanceCommand"),cp);
    all_connected &=port_JointPositionCommand.connectTo(peer->getPort("JointPositionCommand"),cp);
    all_connected &=port_JointTorqueCommand.connectTo(peer->getPort("JointTorqueCommand"),cp);
    all_connected &=port_ToKRL.connectTo(peer->getPort("toKRL"),cp);

    if(!all_connected)
        RTT::log(RTT::Error) << "Not all ports are connected !" << RTT::endlog();
    return all_connected;
}
bool RTTLWRAbstract::connectAllPorts(const std::string& robot_name)
{
    return this->connectPorts(robot_name,RTT::ConnPolicy::data());
}

bool RTTLWRAbstract::connectAllPortsMQueue(const std::string& robot_name)
{
    return this->connectPorts(robot_name,RTT::MQConnPolicy::mq_data());
}

bool RTTLWRAbstract::updateState(){
    RTT::FlowStatus fs_q =  getJointPosition(jnt_pos);
    RTT::FlowStatus fs_qd = getJointVelocity(jnt_vel);
    jnt_pos_kdl.data = jnt_pos;
    jnt_vel_kdl.data = jnt_vel;
    jnt_pos_vel_kdl.q.data = jnt_pos;
    jnt_pos_vel_kdl.qdot.data = jnt_vel;
    return fs_q == RTT::NewData && fs_qd == RTT::NewData;
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
    if(is_sim) return true;
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
    if(is_sim) return true;
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
    if(is_sim) return;
    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::setToolKRL(const unsigned int toolNumber){
	fri_to_krl.intData[2] = toolNumber;
    if(is_sim) return;
	port_ToKRL.write(fri_to_krl);
}

int RTTLWRAbstract::getToolKRL(){
    if(port_FromKRL.readNewest(fri_from_krl) == RTT::NoData)
        return 0;
    else if(is_sim)
        fri_from_krl = fri_to_krl;
    return fri_from_krl.intData[2];
}

FRI_STATE RTTLWRAbstract::getFRIMode(){
    if(port_FromKRL.readNewest(fri_from_krl) == RTT::NoData)
        return FRI_STATE_MON;
    else if(is_sim)
        fri_from_krl = fri_to_krl;
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
    if(is_sim) return FRI_CTRL_JNT_IMP;
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

void RTTLWRAbstract::friStop()
{
    if(is_sim) return;
    //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=lwr::FRI_STOP;
    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::friStart()
{
    if(is_sim) return;
    //Put 1 in $FRI_FRM_INT[1] to trigger fri_start()
    fri_to_krl.intData[0]=lwr::FRI_START;
    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::friReset()
{
    if(is_sim) return;
    //initialize the arrays that will be send to KRL
    for(int i=0; i<FRI_USER_SIZE; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
    port_ToKRL.write(fri_to_krl);
}

void RTTLWRAbstract::stopKrlScript()
{
    if(is_sim) return;
    //Put 3 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=lwr::STOP_KRL_SCRIPT;
    port_ToKRL.write(fri_to_krl);
}

RTT::FlowStatus RTTLWRAbstract::getCartesianPosition(geometry_msgs::Pose& cart_position){
    return readData(port_CartesianPosition,cart_position);
}
RTT::FlowStatus RTTLWRAbstract::getJointPosition(Eigen::VectorXd& joint_position){
    return readData(port_JointPosition,joint_position);
}

RTT::FlowStatus RTTLWRAbstract::getJacobian(KDL::Jacobian& jacobian){
    return readData(port_Jacobian,jacobian);
}

RTT::FlowStatus RTTLWRAbstract::getMassMatrix(Eigen::MatrixXd& mass_matrix){
    return readData(port_MassMatrix,mass_matrix);
}

RTT::FlowStatus RTTLWRAbstract::getGravityTorque(Eigen::VectorXd& gravity_torque){
    return readData(port_GravityTorque,gravity_torque);
}

RTT::FlowStatus RTTLWRAbstract::getJointTorque(Eigen::VectorXd& joint_torque){
    return readData(port_JointTorque,joint_torque);
}

RTT::FlowStatus RTTLWRAbstract::getCartesianVelocity(geometry_msgs::Twist& cart_twist){
    return readData(port_CartesianVelocity,cart_twist);
}

RTT::FlowStatus RTTLWRAbstract::getCartesianWrench(geometry_msgs::Wrench& cart_wrench){
    return readData(port_CartesianWrench,cart_wrench);
}
RTT::FlowStatus RTTLWRAbstract::getJointVelocity(Eigen::VectorXd& joint_velocity){
    return readData(port_JointVelocity,joint_velocity);
}
RTT::FlowStatus RTTLWRAbstract::getJointTorqueRaw(Eigen::VectorXd& joint_torque_raw){
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
