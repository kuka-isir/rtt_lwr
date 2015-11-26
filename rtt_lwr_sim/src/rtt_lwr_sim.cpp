// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include <Eigen/Dense>
#include <ros/service.h>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace Eigen;

LWRSim::LWRSim(std::string const& name):TaskContext(name),
n_joints_(0),
urdf_str_(""),
robot_name_(""),
root_link("link_0"),
tip_link("link_7"),
use_sim_clock(true),
dr_max_(0.1),
safety_checks_(false),
connect_to_rtt_gazebo_at_configure(true),
using_corba(false),
service_timeout_s(20.0),
gravity_vector(0.,0.,-9.81289)
{
    //this->addAttribute("fromKRL", m_fromKRL);
    this->addProperty("using_corba", using_corba).doc("");
    this->addProperty("service_timeout_s", service_timeout_s).doc("");
    //this->addAttribute("toKRL", m_toKRL);

    this->addProperty("fri_port", prop_fri_port).doc("");
    this->addProperty("joint_offset", prop_joint_offset).doc("");

    this->addProperty("root_link", root_link).doc("");
    this->addProperty("tip_link", tip_link).doc("");
    this->addProperty("robot_description",urdf_str_).doc("The URDF of the Kuka");
    this->addProperty("dr_max",dr_max_).doc("The max rot angle cmd beetween two frames");
    this->addProperty("n_joints",n_joints_);
    this->addProperty("use_sim_clock",use_sim_clock);
    this->addProperty("safety_checks",safety_checks_);
    this->addProperty("robot_name",robot_name_).doc("The name of the robot lwr/lwr_sim");

    this->ports()->addPort("JointPositionGazeboCommand", port_JointPositionGazeboCommand).doc("");
    this->ports()->addPort("JointVelocityGazeboCommand", port_JointVelocityGazeboCommand).doc("");
    this->ports()->addPort("JointTorqueGazeboCommand", port_JointTorqueGazeboCommand).doc("");

    this->ports()->addPort("JointPositionGazebo", port_JointPositionGazebo).doc("");
    this->ports()->addPort("JointVelocityGazebo", port_JointVelocityGazebo).doc("");
    this->ports()->addPort("JointTorqueGazebo", port_JointTorqueGazebo).doc("");
    this->ports()->addPort("JointStateGazebo", port_JointStateGazebo).doc("");

    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

    this->ports()->addPort("toKRL",port_ToKRL).doc("");
    //this->ports()->addPort("KRL_CMD", port_KRL_CMD).doc("");
    this->ports()->addPort("fromKRL",port_FromKRL).doc("");

    this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
    this->ports()->addPort("CartesianWrenchStamped", port_CartesianWrenchStamped).doc("");
    this->ports()->addPort("RobotState", port_RobotState).doc("");
    this->ports()->addPort("FRIState", port_FRIState).doc("");
    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
    this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc("");
    this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
    this->ports()->addPort("Jacobian", port_Jacobian).doc("");
    this->ports()->addPort("JointTorque", port_JointTorque).doc("");
    this->ports()->addPort("GravityTorque", port_GravityTorque).doc("");
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");
    //this->ports()->addPort("JointTorqueRaw", port_JointTorqueRaw).doc("");
    //this->ports()->addPort("JointPositionFRIOffset", port_JointPositionFRIOffset).doc("");

    this->ports()->addPort("JointState",port_JointState).doc("");
    this->ports()->addPort("JointStateCommand",port_JointStateCommand).doc("");
    this->ports()->addPort("JointStateGravity",port_JointStateGravity).doc("");


    this->addProperty("kp",kp_);
    this->addProperty("kd",kd_);
    this->addProperty("kg",kg_);
    this->addProperty("kc",kc_);
    this->addProperty("kcd",kcd_);

    this->addOperation("setJointImpedance",&LWRSim::setJointImpedance,this,OwnThread);
    this->addOperation("connectToGazeboCORBA",&LWRSim::connectToGazeboCORBA,this,OwnThread);
    this->addOperation("setCartesianImpedance",&LWRSim::setCartesianImpedance,this,OwnThread);
    this->addOperation("setGravityMode",&LWRSim::setGravityMode,this,OwnThread);
    this->addOperation("resetJointImpedanceGains",&LWRSim::resetJointImpedanceGains,this,OwnThread);

    this->addOperation("setJointImpedanceMode",&LWRSim::setJointImpedanceMode,this,OwnThread);
    this->addOperation("setCartesianImpedanceMode",&LWRSim::setCartesianImpedanceMode,this,OwnThread);

    this->addOperation("setInitialJointPosition",&LWRSim::setInitialJointPosition,this,OwnThread);

    this->provides("debug")->addAttribute("read_start",this->read_start);
    this->provides("debug")->addAttribute("write_start",this->write_start);
    this->provides("debug")->addAttribute("read_duration",this->read_duration);
    this->provides("debug")->addAttribute("id_duration",this->id_duration);
    this->provides("debug")->addAttribute("fk_duration",this->fk_duration);
    this->provides("debug")->addAttribute("ik_duration",this->ik_duration);
    this->provides("debug")->addAttribute("write_duration",this->write_duration);
    this->provides("debug")->addAttribute("updatehook_duration",this->updatehook_duration);
    this->provides("debug")->addAttribute("period_sim",this->period_sim_);

    this->addOperation("waitForROSService",&LWRSim::waitForROSService,this,OwnThread);
}

bool LWRSim::waitForROSService(std::string service_name)
{
    return ros::service::waitForService(service_name, service_timeout_s*1E3);
}

void LWRSim::resetJointImpedanceGains()
{
    kp_default_[0] = 450.0;   kd_default_[0] = 1.0;
    kp_default_[1] = 450.0;   kd_default_[1] = 1.0;
    kp_default_[2] = 200.0;   kd_default_[2] = 0.7;
    kp_default_[3] = 200.0;   kd_default_[3] = 0.7;
    kp_default_[4] = 200.0;   kd_default_[4] = 0.7;
    kp_default_[5] = 20.0;    kd_default_[5] = 0.1;
    kp_default_[6] = 10.0;    kd_default_[6] = 0.0;
    this->setJointImpedance(kp_default_,kd_default_);
}
void LWRSim::resetCartesianImpedanceGains()
{
    kc_default_[0] = 1000.0;   kcd_default_[0] = 0.7;
    kc_default_[1] = 1000.0;   kcd_default_[1] = 0.7;
    kc_default_[2] = 1000.0;   kcd_default_[2] = 0.7;
    kc_default_[3] = 200.0;   kcd_default_[3] = 0.7;
    kc_default_[4] = 200.0;   kcd_default_[4] = 0.7;
    kc_default_[5] = 200.0;    kcd_default_[5] = 0.7;
    this->setCartesianImpedance(kc_default_,kcd_default_);
}
void LWRSim::setInitialJointPosition(const std::vector< double > j_init)
{
    if(!j_init.size() == LBR_MNJ){
        log(Error) << "Invalid size (should be " << LBR_MNJ << ")" << endlog();
        return;
    }
    joint_state_cmd_.position = j_init;

    if(using_corba)
    {
        log(Warning) << "Writing to CORBA " << joint_state_cmd_ << endlog();
        port_JointPositionGazeboCommand.write(j_init);
    }
    log(Warning) << getName() + " asking Joint Position to " << joint_state_cmd_ << endlog();
    init_pos_requested = true;
}

bool LWRSim::configureHook(){
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        log(Error) << "Could not load rosparam service." <<endlog();
        return false;
    }

    // Set the Robot name (lwr or lwr_sim) For other components to know it
    rosparam->getRelative("robot_name");
    rosparam->getRelative("root_link");
    rosparam->getRelative("tip_link");

    log(Info)<<"root_link : "<<root_link<<endlog();
    log(Info)<<"tip_link : "<<tip_link<<endlog();

    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,kdl_tree_,kdl_chain_))
    {
        log(Error) << "Error while loading the URDF with params : "<<robot_name_<<" "<<root_link<<" "<<tip_link <<endlog();
        return false;
    }

    id_dyn_solver.reset(new ChainDynParam(kdl_chain_,gravity_vector));
    id_rne_solver.reset(new ChainIdSolver_RNE(kdl_chain_,gravity_vector));
    fk_vel_solver.reset(new ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver.reset(new ChainJntToJacSolver(kdl_chain_));

    // Store the number of degrees of freedom of the chain
    n_joints_ = kdl_chain_.getNrOfJoints();

    log(Debug)<<"LWR njoints : "<<LBR_MNJ<<" Segments : "<<kdl_chain_.getNrOfSegments()<<endlog();

    if(n_joints_ =! LBR_MNJ)
    {
        log(Error)<<"Number of joints error : found "<<LBR_MNJ<<endlog();
        return false;
    }

    // In from gazebo
    joint_position_gazebo.resize(LBR_MNJ,0.0);
    joint_velocity_gazebo.resize(LBR_MNJ,0.0);
    joint_torque_gazebo.resize(LBR_MNJ,0.0);
    // Out to gazebo
    joint_position_gazebo_cmd.resize(LBR_MNJ,0.0);
    joint_velocity_gazebo_cmd.resize(LBR_MNJ,0.0);
    joint_torque_gazebo_cmd.resize(LBR_MNJ,0.0);

    port_JointPositionGazeboCommand.setDataSample(joint_position_gazebo_cmd);
    port_JointVelocityGazeboCommand.setDataSample(joint_velocity_gazebo_cmd);
    port_JointTorqueGazeboCommand.setDataSample(joint_torque_gazebo_cmd);
    // Fake LWR
    jnt_pos_.resize(LBR_MNJ);
    jnt_pos_fri_offset.resize(LBR_MNJ);
    jnt_pos_old_.resize(LBR_MNJ);
    jnt_vel_.resize(LBR_MNJ);
    jnt_trq_coriolis_kdl_.resize(LBR_MNJ);
    jnt_trq_.resize(LBR_MNJ);
    jnt_trq_raw_.resize(LBR_MNJ);
    grav_trq_.resize(LBR_MNJ);
    jnt_pos_cmd_.resize(LBR_MNJ);
    jnt_trq_cmd_.resize(LBR_MNJ);
    jac_.resize(LBR_MNJ);
    jac_.data.setZero();
    mass_.resize(LBR_MNJ,LBR_MNJ);
    H.resize(LBR_MNJ);
    jnt_trq_gazebo_cmd_.resize(LBR_MNJ);
    prop_joint_offset.resize(LBR_MNJ);
    std::fill(prop_joint_offset.begin(),prop_joint_offset.end(),0.0);

    jnt_trq_gazebo_cmd_.setZero();
    jnt_pos_.setZero();
    jnt_pos_fri_offset.setZero();
    jnt_pos_old_.setZero();
    jnt_vel_.setZero();
    jnt_trq_.setZero();
    jnt_trq_raw_.setZero();
    grav_trq_.setZero();
    jnt_pos_cmd_.setZero();
    jnt_trq_cmd_.setZero();
    mass_.setZero();// = MatrixXd::Zero(LBR_MNJ,LBR_MNJ);

    kp_.resize(LBR_MNJ);
    kd_.resize(LBR_MNJ);
    kg_.resize(LBR_MNJ);
    kg_.setConstant(1.0);
    kp_default_.resize(LBR_MNJ);
    kd_default_.resize(LBR_MNJ);
    kc_.resize(6);
    kcd_.resize(6);
    kc_default_.resize(6);
    kcd_default_.resize(6);

    for(int i=0;i<FRI_USER_SIZE;i++)
    {
        fri_from_krl.realData[i]=0;
        fri_from_krl.intData[i]=0;
    }
    fri_from_krl.boolData=0;
    fri_from_krl.fill=0;

    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_cmd_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_gravity_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_dyn_);

    port_JointState.setDataSample(joint_state_);
    port_JointStateCommand.setDataSample(joint_state_cmd_);
    port_JointStateGravity.setDataSample(joint_state_gravity_);
    port_JointStateDynamics.setDataSample(joint_state_dyn_);

    port_JointPosition.setDataSample(jnt_pos_);
    //port_JointPositionFRIOffset.setDataSample(jnt_pos_fri_offset);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);
    //port_JointTorqueRaw.setDataSample(jnt_trq_raw_);
    port_GravityTorque.setDataSample(grav_trq_);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_);
    port_FromKRL.setDataSample(fri_from_krl);


    port_JointState.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states"));

    port_JointStateGravity.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states_gravity"));
    port_JointStateDynamics.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states_dynamics"));

    port_CartesianPositionStamped.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/cartesian_pose"));
    //port_CartesianVelocity.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_twist"));
    //port_CartesianWrench.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_wrench"));

    //port_CartesianWrenchStamped.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/cartesian_wrench"));

    q.resize(LBR_MNJ);
    f_ext.resize(kdl_chain_.getNrOfSegments());
    G.resize(LBR_MNJ);
    qdot.resize(LBR_MNJ);
    qddot.resize(LBR_MNJ);
    jnt_trq_kdl_.resize(LBR_MNJ);

    robot_state.control = static_cast<fri_uint16_t>(FRI_CTRL_POSITION);
    fri_state.quality = static_cast<fri_uint16_t>(FRI_QUALITY_PERFECT);
    fri_to_krl.intData[0] = FRI_STATE_MON;
    fri_from_krl.intData[0] = FRI_STATE_MON;

    pos_limits_.resize(LBR_MNJ);
    vel_limits_.resize(LBR_MNJ);
    trq_limits_.resize(LBR_MNJ);
    pos_limits_ << 170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD;
    vel_limits_ << 112.5*TORAD,112.5*TORAD,112.5*TORAD,112.5*TORAD,180*TORAD,112.5*TORAD,112.5*TORAD;
    trq_limits_ << 200,200,100,100,100,30,30;


    resetJointImpedanceGains();
    resetCartesianImpedanceGains();

    // This is necessary to plot the data as gazebo sets the clock
    // Note: Connections are still made and working, it's again just for plotting/recording
    if(false && use_sim_clock){
        Logger::Instance()->in(getName());
        log(Warning) << "Using ROS Sim Clock" << endlog();
        rtt_rosclock::use_ros_clock_topic();
        rtt_rosclock::enable_sim();
        rtt_rosclock::set_sim_clock_activity(this);
    }
}

bool LWRSim::connectToGazeboCORBA(const std::string& gazebo_deployer_name, const std::string& gazebo_robot_comp_name)
{
    if(this->hasPeer(gazebo_deployer_name)){
            TaskContext* peer = NULL;
            peer = this->getPeer(gazebo_deployer_name);
            if(peer == NULL) return false;
            // Getting information from gazebo
            ConnPolicy policy = ConnPolicy::data();
            port_JointPositionGazeboCommand.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointPositionCommand"),policy);
            port_JointVelocityGazeboCommand.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointVelocityCommand"),policy);
            port_JointTorqueGazeboCommand.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointTorqueCommand"),policy);

            port_JointPositionGazebo.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointPosition"),policy);
            port_JointVelocityGazebo.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointVelocity"),policy);
            port_JointTorqueGazebo.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointTorque"),policy);
            return true;
        }else{
            log(Error)<<"Couldn't find "<<gazebo_deployer_name<<" deployer name"<<endlog();
            return false;
        }
}

bool LWRSim::safetyChecks(const VectorXd& position,
                          const VectorXd& velocity,
                          const VectorXd& torque)
{
    return safetyCheck(position,pos_limits_,"Position") &&
    safetyCheck(velocity,vel_limits_,"Velocity") &&
    safetyCheck(torque,trq_limits_,"Torque");
}

bool LWRSim::safetyCheck(const VectorXd& v,
                         const VectorXd& limits,
                         const std::string& name)
{
    if(v.size() != LBR_MNJ)
    {
        log(Error) << name<<" vector size error "<<v.size()<<"!="<<LBR_MNJ<<endlog();
        return false;
    }

    bool ret=true;
    for(unsigned i=0;i<v.size();i++)
    {
        if(std::abs(v[i]) > limits[i])
        {
           log(Error) << name<<" limit exceded at J"<<i<<" : "<<v[i]<<" / limit "<<limits[i]<<endlog();
           ret = false;
        }
    }
    return ret;
}

bool LWRSim::setJointImpedance(const VectorXd& stiffness, const VectorXd& damping)
{
    if(! (stiffness.size() == LBR_MNJ && damping.size() == LBR_MNJ))
    {
        log(Error) << "Size error, not setting impedance (stiff size "
                            <<stiffness.size()<<" damp size "<<damping.size()
                            <<", should be "<<LBR_MNJ<<")"<<endlog();
        return false;
    }

    for(unsigned j=0;j<LBR_MNJ;++j)
    {
        jnt_imp_.stiffness[j] = stiffness[j];
        jnt_imp_.damping[j] = damping[j];
    }

    kp_ = stiffness;
    kd_ = damping;

    return true;
}
bool LWRSim::setGravityMode()
{
    VectorXd s(LBR_MNJ);
    s.setZero();
    VectorXd d(LBR_MNJ);
    d.setZero();
    return this->setJointImpedance(s,d);
}

bool LWRSim::setCartesianImpedance(const Matrix< double, 6, 1 >& cart_stiffness, const Matrix< double, 6, 1 >& cart_damping)
{
    if(cart_damping.size() != 6 || cart_damping.size() != 6)
    {
        log(Error) << "Size error, not setting cartesian impedance (stiff size "
                            <<cart_stiffness.size()<<" damp size "<<cart_damping.size()
                            <<", should be 6 and 6)"<<endlog();
        return false;
    }

    cart_imp_.stiffness.linear.x = cart_stiffness[0];
    cart_imp_.stiffness.linear.y = cart_stiffness[1];
    cart_imp_.stiffness.linear.z = cart_stiffness[2];

    cart_imp_.stiffness.angular.x = cart_stiffness[3];
    cart_imp_.stiffness.angular.y = cart_stiffness[4];
    cart_imp_.stiffness.angular.z = cart_stiffness[5];

    cart_imp_.damping.linear.x = cart_damping[0];
    cart_imp_.damping.linear.y = cart_damping[1];
    cart_imp_.damping.linear.z = cart_damping[2];

    cart_imp_.damping.angular.x = cart_damping[3];
    cart_imp_.damping.angular.y = cart_damping[4];
    cart_imp_.damping.angular.z = cart_damping[5];

    kc_ = cart_stiffness;
    kcd_ = cart_damping;

    return true;
}

void LWRSim::updateJointImpedance(const lwr_fri::FriJointImpedance& impedance)
{
    kp_[0] = clamp(impedance.stiffness[0],0.0,450.0);   kd_[0] = clamp(impedance.damping[0],0.0,1.0);
    kp_[1] = clamp(impedance.stiffness[1],0.0,450.0);   kd_[1] = clamp(impedance.damping[1],0.0,1.0);
    kp_[2] = clamp(impedance.stiffness[2],0.0,200.0);   kd_[2] = clamp(impedance.damping[2],0.0,0.7);
    kp_[3] = clamp(impedance.stiffness[3],0.0,200.0);   kd_[3] = clamp(impedance.damping[3],0.0,0.7);
    kp_[4] = clamp(impedance.stiffness[4],0.0,200.0);   kd_[4] = clamp(impedance.damping[4],0.0,0.7);
    kp_[5] = clamp(impedance.stiffness[5],0.0,10.0);   kd_[5] = clamp(impedance.damping[5],0.0,0.1);
    kp_[6] = clamp(impedance.stiffness[6],0.0,10.0);   kd_[6] = clamp(impedance.damping[6],0.0,0.0);

}

void LWRSim::updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance)
{
    kc_[0] = cart_impedance.stiffness.linear.x;
    kc_[1] = cart_impedance.stiffness.linear.y;
    kc_[2] = cart_impedance.stiffness.linear.z;

    kc_[3] = cart_impedance.stiffness.angular.x;
    kc_[4] = cart_impedance.stiffness.angular.y;
    kc_[5] = cart_impedance.stiffness.angular.z;

    kcd_[0] = cart_impedance.damping.linear.x;
    kcd_[1] = cart_impedance.damping.linear.y;
    kcd_[2] = cart_impedance.damping.linear.z;

    kcd_[3] = cart_impedance.damping.angular.x;
    kcd_[4] = cart_impedance.damping.angular.y;
    kcd_[5] = cart_impedance.damping.angular.z;

    cart_imp_ = cart_impedance;
}


void LWRSim::setJointImpedanceMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_JNT_IMP;
}
void LWRSim::setCartesianImpedanceMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_CART_IMP;
}

void LWRSim::updateHook() {
    log(Debug) << "LWR SIM Update at "<<rtt_rosclock::host_now()<< endlog();
    static double last_update_time_sim;
    double rtt_time_ = 1E-9*os::TimeService::ticks2nsecs(os::TimeService::Instance()->getTicks());
    period_sim_ = rtt_time_ - last_update_time_sim;
    last_update_time_sim = rtt_time_;

    read_start = rtt_rosclock::host_now().toSec();
    // Read From gazebo simulation
    if(using_corba && (port_JointPositionGazebo.connected()
        || port_JointVelocityGazebo.connected()
        || port_JointTorqueGazebo.connected()))
    {
        FlowStatus fs_p = port_JointPositionGazebo.read(joint_position_gazebo);
        FlowStatus fs_v = port_JointVelocityGazebo.read(joint_velocity_gazebo);
        FlowStatus fs_g = port_JointTorqueGazebo.read(joint_torque_gazebo);

        if(fs_g == NoData || fs_p == NoData || fs_v == NoData){
            log(Warning)  << getName() << ": NoData" << endlog();
            return;
        }
    }else{// NOTE: Using ROS instead
        if(NoData != port_JointStateGazebo.read(joint_state_))
        {
            joint_position_gazebo = joint_state_.position;
            joint_velocity_gazebo = joint_state_.velocity;
            joint_torque_gazebo = joint_state_.effort;
        }

    }

    joint_state_cmd_.header.frame_id = "";

    jnt_trq_cmd_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
    jnt_pos_cmd_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
    cart_pos_cmd_fs = port_CartesianPositionCommand.read(cart_pos_cmd_);
    cart_wrench_cmd_fs = port_CartesianWrenchCommand.read(cart_wrench_cmd_);
    FlowStatus fs_fri_to_krl = port_ToKRL.read(fri_to_krl);
    FlowStatus fs_jnt_imp_cmd = port_JointImpedanceCommand.read(jnt_imp_cmd_);
    FlowStatus fs_cart_imp_cmd = port_CartesianImpedanceCommand.read(cart_imp_cmd_);

    fri_state.timestamp = read_start;

    if(fs_fri_to_krl == NewData)
    {
        for(int i=0;i<FRI_USER_SIZE;i++)
        {
            fri_from_krl.realData[i]=fri_to_krl.realData[i];
            fri_from_krl.intData[i]=fri_to_krl.intData[i];
        }
        fri_from_krl.boolData=fri_to_krl.boolData;
    }
    switch(fri_to_krl.intData[1]){
        case 10:
            robot_state.control = FRI_CTRL_POSITION;
            break;
        case 30:
            robot_state.control = FRI_CTRL_JNT_IMP;
            break;
        case 20:
            robot_state.control = FRI_CTRL_CART_IMP;
            break;
        default:
            robot_state.control = FRI_CTRL_OTHER;
            break;
        }
    switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_POSITION:
            case FRI_CTRL_JNT_IMP:
            case FRI_CTRL_CART_IMP:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_CMD;
                robot_state.power = 1;
                break;
            case FRI_CTRL_OTHER:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_MON;
                robot_state.power = 0;
                break;
            default:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_OFF;
                robot_state.power = 0;
                break;
        }

    jnt_pos_ = VectorXd::Map(joint_position_gazebo.data(),LBR_MNJ);
    jnt_vel_ = VectorXd::Map(joint_velocity_gazebo.data(),LBR_MNJ);
    jnt_trq_ = VectorXd::Map(joint_torque_gazebo.data(),LBR_MNJ);

    if(safety_checks_)
        safetyChecks(jnt_pos_,jnt_vel_,jnt_trq_);

    // Waiting for CartesianVelocityCommand
    Xd_cmd_.setZero();

    if(fs_jnt_imp_cmd == NewData)
        updateJointImpedance(jnt_imp_cmd_);

    if(fs_cart_imp_cmd == NewData)
        updateCartesianImpedance(cart_imp_cmd_);

    read_duration = (rtt_rosclock::host_now().toSec() - read_start);

    q.q.data = jnt_pos_;
    q.qdot.data = jnt_vel_;
    qdot.data = jnt_vel_;
    qddot.data.setConstant(0.0);

    id_dyn_solver->JntToMass(q.q,H);
    mass_ = H.data;

    std::fill(f_ext.begin(),f_ext.end(),Wrench::Zero());

    id_rne_solver->CartToJnt(q.q,qdot,qddot,f_ext,jnt_trq_kdl_);

    id_dyn_solver->JntToGravity(q.q,G);
    id_dyn_solver->JntToCoriolis(q.q,q.qdot,jnt_trq_coriolis_kdl_);

    Map<VectorXd>(joint_state_dyn_.position.data(),LBR_MNJ) = G.data;
    Map<VectorXd>(joint_state_dyn_.velocity.data(),LBR_MNJ) = jnt_trq_coriolis_kdl_.data;
    Map<VectorXd>(joint_state_dyn_.effort.data(),LBR_MNJ) = jnt_trq_kdl_.data;

    grav_trq_ = G.data;

    jnt_to_jac_solver->JntToJac(this->q.q,jac_,kdl_chain_.getNrOfSegments());
    fk_vel_solver->JntToCart(q,ee_framevel_kdl_,kdl_chain_.getNrOfSegments());
    ee_twist_kdl_ = ee_framevel_kdl_.GetTwist();
    ee_frame_kdl_ = ee_framevel_kdl_.GetFrame();

    tf::poseKDLToMsg(ee_frame_kdl_,cart_pos_);
    tf::twistKDLToMsg(ee_twist_kdl_,cart_twist_);

    cart_pos_stamped_.header.frame_id = root_link;
    cart_pos_stamped_.pose = cart_pos_;


    //ee_twist_des_kdl_.SetToZero();
    SetToZero(ee_twist_des_kdl_);
    ee_twist_diff_kdl_ = diff(ee_twist_kdl_,ee_twist_des_kdl_);
    tf::twistKDLToEigen(ee_twist_diff_kdl_,Xd_err_);

    if(cart_pos_cmd_fs == NewData){
        tf::poseMsgToKDL(cart_pos_cmd_,ee_frame_des_kdl_);
        ee_frame_diff_kdl_ = diff(ee_frame_kdl_,ee_frame_des_kdl_);
        tf::twistKDLToEigen(ee_frame_diff_kdl_,X_err_);
    }


    F_cmd_.setZero();
    if(cart_wrench_cmd_fs == NewData)
        tf::wrenchMsgToEigen(cart_wrench_cmd_,F_cmd_);

    if(jnt_trq_cmd_fs != NewData)
        jnt_trq_cmd_.setZero();

    cart_wrench_stamped_.header.frame_id = tip_link;
    cart_wrench_stamped_.wrench = cart_wrench_;

    // Send at least Gravity

      switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_JNT_IMP:
                // Joint Impedance Control
                jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * G.data;

                // Joint Impedance part
                if(jnt_pos_cmd_fs == NewData){
                    jnt_trq_gazebo_cmd_ += kp_.asDiagonal()*(jnt_pos_cmd_ - jnt_pos_) - kd_.asDiagonal()*jnt_vel_ ;
                }
                // Additional torque
                if(jnt_trq_cmd_fs != NoData)
                    jnt_trq_gazebo_cmd_ += jnt_trq_cmd_;

                Map<VectorXd>(joint_torque_gazebo_cmd.data(),LBR_MNJ) = jnt_trq_gazebo_cmd_;
                port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                joint_state_cmd_.header.frame_id = "EFFORT_CMD";

                break;
            case FRI_CTRL_POSITION:
                //Position Control
                jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * G.data;

                if(jnt_pos_cmd_fs == NewData){
                    jnt_trq_gazebo_cmd_ += kp_default_.asDiagonal()*(jnt_pos_cmd_-jnt_pos_) - kd_default_.asDiagonal()*jnt_vel_ ;
                }

                Map<VectorXd>(joint_torque_gazebo_cmd.data(),LBR_MNJ) = jnt_trq_gazebo_cmd_;
                port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                joint_state_cmd_.header.frame_id = "EFFORT_CMD";

                break;
            case FRI_CTRL_CART_IMP:
                //Cartesian Impedance Control
                jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * G.data;

                if(cart_pos_cmd_fs == NewData){
                    jnt_trq_gazebo_cmd_ += jac_.data.transpose()*(kc_.asDiagonal()*(X_err_) + F_cmd_ + kcd_.asDiagonal()*(Xd_err_));
                }

                Map<VectorXd>(joint_torque_gazebo_cmd.data(),LBR_MNJ) = jnt_trq_gazebo_cmd_;
                port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                joint_state_cmd_.header.frame_id = "EFFORT_CMD";

                break;
            default:
                break;
      }

    now = rtt_rosclock::host_now();
    write_start = now.toSec();
    //Update status
    cart_pos_stamped_.header.stamp = now;
    cart_wrench_stamped_.header.stamp = now;
    joint_state_.header.stamp = now;

    joint_state_cmd_.header.stamp = now;
    joint_state_gravity_.header.stamp = now;
    joint_state_dyn_.header.stamp = now;

    Map<VectorXd>(joint_state_gravity_.effort.data(),LBR_MNJ) = grav_trq_;

    Map<VectorXd>(joint_state_.position.data(),LBR_MNJ) = jnt_pos_;
    Map<VectorXd>(joint_state_.velocity.data(),LBR_MNJ) = jnt_vel_;
    Map<VectorXd>(joint_state_.effort.data(),LBR_MNJ) = jnt_trq_;

    if(jnt_pos_cmd_fs == NewData /*&& init_pos_requested==false*/)
        Map<VectorXd>(joint_state_cmd_.position.data(),LBR_MNJ) = jnt_pos_cmd_;

    joint_state_cmd_.effort = joint_torque_gazebo_cmd;

    port_JointState.write(joint_state_);
    port_JointStateDynamics.write(joint_state_dyn_);

    port_JointPosition.write(jnt_pos_);
    port_JointVelocity.write(jnt_vel_);
    port_JointTorque.write(jnt_trq_);
    port_GravityTorque.write(grav_trq_);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianPositionStamped.write(cart_pos_stamped_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);
    port_CartesianWrenchStamped.write(cart_wrench_stamped_);

    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_);

    if(init_pos_requested){
        joint_state_cmd_.header.frame_id="POSITION_CMD";
        init_pos_requested=false;
    }
    port_JointStateCommand.write(joint_state_cmd_);
    port_FromKRL.write(fri_from_krl);

    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);

    write_duration = (rtt_rosclock::host_now().toSec() - write_start);
    updatehook_duration = (rtt_rosclock::host_now().toSec() - read_start);

}
