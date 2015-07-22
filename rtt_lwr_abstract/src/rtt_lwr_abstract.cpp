// Copyright 2015 ISIR
// Author: Antoine Hoarau

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
using namespace lwr;

RTTLWRAbstract::RTTLWRAbstract(std::string const& name) : 
RTT::TaskContext(name),
robot_name("lwr"),
jnt_pos_bdf(7),
n_joints(LBR_MNJ),
root_link("link_0"),
tip_link("link_7"),
gravity_vector(0.,0.,-9.81289),
jnt_vel_bdf(LBR_MNJ),
jnt_pos(LBR_MNJ),
jnt_pos_old(LBR_MNJ),
jnt_vel(LBR_MNJ),
jnt_trq(LBR_MNJ),
jnt_trq_raw(LBR_MNJ),
jnt_pos_fri_offset(LBR_MNJ),
mass(LBR_MNJ,LBR_MNJ),
jnt_pos_cmd(LBR_MNJ),
jnt_trq_cmd(LBR_MNJ),
J(LBR_MNJ),
jnt_grav(LBR_MNJ)
{
    this->addProperty("robot_name",robot_name);
    this->addProperty("root_link", root_link).doc("");
    this->addProperty("tip_link", tip_link).doc("");
    this->addProperty("robot_description",robot_description).doc("The URDF description");
    this->addProperty("n_joints",n_joints);
    
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

    this->addOperation("friStart", &RTTLWRAbstract::friStart, this, RTT::OwnThread);
    this->addOperation("friStop", &RTTLWRAbstract::friStop, this, RTT::OwnThread);
    this->addOperation("friReset", &RTTLWRAbstract::friReset, this, RTT::OwnThread);
    this->addOperation("stopKrlScript", &RTTLWRAbstract::stopKrlScript, this, RTT::OwnThread);
    
}

RTTLWRAbstract::~RTTLWRAbstract(){
}

bool RTTLWRAbstract::configureHook(){
    jnt_pos_cmd.setZero();
    jnt_trq_cmd.setZero();
        
    //initialize the arrays that will be send to KRL
    for(int i=0; i<FRI_USER_SIZE; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
    
    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,kdl_tree,kdl_chain))
        return false;
        
    for(unsigned int i=0;i<kdl_chain.getNrOfSegments();++i)
    {
        const std::string name = kdl_chain.getSegment(i).getName();
        seg_names_idx[name] = i+1;
        RTT::log(RTT::Warning) << "Segment " << i << "-> " << name << " idx -> "<< seg_names_idx[name] <<RTT::endlog();
    }
    
    
    ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv_nso(kdl_chain));
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

    // Set the Robot name (lwr or lwr_sim) For other components to know it
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
        this->getProvider<rtt_rosparam::ROSParam>("rosparam");
    rosparam->getRelative("robot_name");
    
    return connectAllPorts(robot_name);
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
    
    
    port_CartesianImpedanceCommand.connectTo(this->peer->getPort("CartesianImpedanceCommand"),policy);
    port_CartesianPositionCommand.connectTo(this->peer->getPort("CartesianPositionCommand"),policy);
    port_CartesianWrenchCommand.connectTo(this->peer->getPort("CartesianWrenchCommand"),policy);
    port_JointImpedanceCommand.connectTo(this->peer->getPort("JointImpedanceCommand"),policy);
    port_JointPositionCommand.connectTo(this->peer->getPort("JointPositionCommand"),policy);
    port_JointTorqueCommand.connectTo(this->peer->getPort("JointTorqueCommand"),policy);
    
    port_JointPositionCommand.setDataSample(jnt_pos_cmd);
    port_JointTorqueCommand.setDataSample(jnt_trq_cmd);
    port_CartesianImpedanceCommand.setDataSample(cart_imp_cmd);
    port_CartesianWrenchCommand.setDataSample(cart_wrench_cmd);
    port_CartesianPositionCommand.setDataSample(cart_pos_cmd);
    port_JointImpedanceCommand.setDataSample(jnt_imp_cmd);
    port_ToKRL.setDataSample(fri_to_krl);
    
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
    
    all_connected &= port_CartesianImpedanceCommand.connected();
    all_connected &= port_CartesianPositionCommand.connected();
    all_connected &= port_CartesianWrenchCommand.connected();
    all_connected &= port_JointImpedanceCommand.connected();
    all_connected &= port_JointPositionCommand.connected();
    all_connected &= port_JointTorqueCommand.connected();
    return all_connected;
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
    port_FRIState.read(fri_state);
    return static_cast<FRI_STATE>(fri_state.state)==FRI_STATE_CMD;
}
bool RTTLWRAbstract::isMonitorMode()
{
    port_FRIState.read(fri_state);
    return static_cast<FRI_STATE>(fri_state.state)==FRI_STATE_MON;
}
bool RTTLWRAbstract::isPowerOn()
{
    port_RobotState.read(robot_state);
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
    port_FromKRL.read(fri_from_krl);
    return fri_from_krl.intData[2];
}

FRI_STATE RTTLWRAbstract::getFRIMode(){
    port_FromKRL.read(fri_from_krl);
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
            break;
        }
     return state;
}

FRI_QUALITY RTTLWRAbstract::getFRIQuality()
{
    port_FRIState.read(fri_state);
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
            break;
        }
    return quality;
}
FRI_CTRL RTTLWRAbstract::getFRIControlMode()
{
    port_RobotState.read(robot_state);
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
            break;
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

bool RTTLWRAbstract::sendCartesianPosition(const geometry_msgs::Pose& cartesian_pose)
{
    port_CartesianPositionCommand.write(cartesian_pose);
    return true;
}
bool RTTLWRAbstract::sendCartesianWrench(const geometry_msgs::Wrench& cartesian_wrench)
{
    port_CartesianWrenchCommand.write(cartesian_wrench);
    return true;
}

bool RTTLWRAbstract::getCartesianPosition(geometry_msgs::Pose& cart_position){
    if (port_CartesianPosition.connected() == false)
        RTT::log(RTT::Warning)<<"port_CartesianPosition not connected"<<RTT::endlog();

    return port_CartesianPosition.read(cart_position)  != RTT::NoData;
}
bool RTTLWRAbstract::getJointPosition(Eigen::VectorXd& joint_position){
    if (port_JointPosition.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointPosition not connected"<<RTT::endlog();

    return port_JointPosition.read(joint_position) != RTT::NoData;
}

bool RTTLWRAbstract::getJacobian(KDL::Jacobian& jacobian){
    if (port_Jacobian.connected() == false)
        RTT::log(RTT::Warning)<<"port_Jacobian not connected"<<RTT::endlog();
    return port_Jacobian.read(jacobian) != RTT::NoData;
}

bool RTTLWRAbstract::getMassMatrix(Eigen::MatrixXd& mass_matrix){
    if (port_MassMatrix.connected() == false)
        RTT::log(RTT::Warning)<<"port_MassMatrix not connected"<<RTT::endlog();
    return port_MassMatrix.read(mass_matrix) != RTT::NoData;
}

bool RTTLWRAbstract::getGravityTorque(Eigen::VectorXd& gravity_torque){
    if (port_GravityTorque.connected() == false)
        RTT::log(RTT::Warning)<<"port_GravityTorque not connected"<<RTT::endlog();
    return port_GravityTorque.read(gravity_torque) != RTT::NoData;
}

bool RTTLWRAbstract::getJointTorque(Eigen::VectorXd& joint_torque){
    if (port_JointTorque.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointTorque not connected"<<RTT::endlog();
    return port_JointTorque.read(joint_torque) != RTT::NoData;
}

bool RTTLWRAbstract::getCartesianVelocity(geometry_msgs::Twist& cart_twist){
    if (port_CartesianVelocity.connected() == false)
        RTT::log(RTT::Warning)<<port_CartesianVelocity.getName()<<" not connected"<<RTT::endlog();
    return port_CartesianVelocity.read(cart_twist) != RTT::NoData;
}

bool RTTLWRAbstract::getCartesianWrench(geometry_msgs::Wrench& cart_wrench){
    if (port_CartesianWrench.connected() == false)
        RTT::log(RTT::Warning)<<"port_CartesianWrench not connected"<<RTT::endlog();
    return port_CartesianWrench.read(cart_wrench) != RTT::NoData;
}
bool RTTLWRAbstract::getJointVelocity(Eigen::VectorXd& joint_velocity)
{
    if (port_JointVelocity.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointVelocity not connected"<<RTT::endlog();
    return port_JointVelocity.read(joint_velocity) != RTT::NoData;
}
bool RTTLWRAbstract::getJointTorqueRaw(Eigen::VectorXd& joint_torque_raw)
{
    if (port_JointTorqueRaw.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointTorqueRaw not connected"<<RTT::endlog();
    return port_JointTorqueRaw.read(joint_torque_raw) != RTT::NoData;
}
bool RTTLWRAbstract::sendJointImpedance(const lwr_fri::FriJointImpedance& joint_impedance_cmd)
{
    if(port_JointImpedanceCommand.connected() == false)
        RTT::log(RTT::Warning)<<"port_JointImpedanceCommand not connected"<<RTT::endlog();
    port_JointImpedanceCommand.write(joint_impedance_cmd);
    return true;
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


bool RTTLWRAbstract::sendJointPosition(const Eigen::VectorXd& joint_position_cmd){
    return this->sendJointCommand(this->port_JointPositionCommand,joint_position_cmd);
}
bool RTTLWRAbstract::sendJointTorque(const Eigen::VectorXd& joint_torque_cmd)
{
    return this->sendJointCommand(this->port_JointTorqueCommand,joint_torque_cmd);
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
