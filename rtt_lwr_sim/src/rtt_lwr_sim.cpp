// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include <rtt_ros_kdl_tools/mqueue_connpolicy.hpp>
#include <Eigen/Dense>
#include <ros/service.h>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

LWRSim::LWRSim(std::string const& name):TaskContext(name),
urdf_str_(""),
robot_name_(""),
root_link_(""),
tip_link_(""),
robot_ns_(""),
rtt_sem_(0),
use_sim_clock(true),
safety_checks_(false),
connect_to_rtt_gazebo_at_configure(true),
set_joint_pos_no_dynamics_(false),
set_brakes_(false),
sync_with_cmds_(true),
nb_loops_(0),
gravity_vector(0.,0.,-9.81289)
{
    this->provides("gazebo")->addOperation("configure",&LWRSim::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&LWRSim::gazeboUpdateHook,this,RTT::ClientThread);
    this->addOperation("setLinkGravityMode",&LWRSim::setLinkGravityMode,this,RTT::ClientThread);
    this->addOperation("ready",&LWRSim  ::readyService,this,RTT::ClientThread);
    //this->addAttribute("fromKRL", m_fromKRL);
    //this->addProperty("using_corba", using_corba).doc("");
    this->addProperty("synchronize_with_commands", sync_with_cmds_).doc("");
    this->addProperty("kp_default", kp_default_).doc("");
    this->addProperty("kd_default", kd_default_).doc("");
    this->addProperty("kcp_default", kcp_default_).doc("");
    this->addProperty("kcd_default", kcd_default_).doc("");
    //this->addAttribute("toKRL", m_toKRL);

    this->addProperty("fri_port", prop_fri_port).doc("");
    //this->addProperty("joint_offset", prop_joint_offset).doc("");

    this->addProperty("root_link", root_link_).doc("");
    this->addProperty("set_brakes", set_brakes_).doc("");
    this->addProperty("tip_link", tip_link_).doc("");
    this->addProperty("robot_description",urdf_str_).doc("");
    this->addProperty("robot_ns",robot_ns_).doc("");
    this->addProperty("tf_prefix",tf_prefix_).doc("");
    this->addProperty("gravity_vector",gravity_vector).doc("");
    this->addProperty("use_sim_clock",use_sim_clock).doc("");
    this->addProperty("safety_checks",safety_checks_).doc("");
    this->addProperty("robot_name",robot_name_).doc("The name of the robot lwr/lwr_sim");

    /*this->ports()->addPort("SyncStatus",port_sync_status).doc("");
    this->ports()->addEventPort("SyncCommand",port_sync_cmd).doc("");*/

    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

    this->ports()->addPort("toKRL",port_ToKRL).doc("");
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

    this->ports()->addPort("JointStates",port_JointStates).doc("");
    this->ports()->addPort("JointStatesCommand",port_JointStatesCommand).doc("");
    this->ports()->addPort("JointStatesDynamicsDecomposition",port_JointStatesDynamics).doc("");

    this->addProperty("kp",kp_);
    this->addProperty("kd",kd_);
    this->addProperty("kg",kg_);
    this->addProperty("kcp",kcp_);
    this->addProperty("kcd",kcd_);

    this->addOperation("setJointImpedance",&LWRSim::setJointImpedance,this,OwnThread);
    this->addOperation("setCartesianImpedance",&LWRSim::setCartesianImpedance,this,OwnThread);
    this->addOperation("setGravityMode",&LWRSim::setGravityMode,this,OwnThread);
    this->addOperation("resetJointImpedanceGains",&LWRSim::resetJointImpedanceGains,this,OwnThread);

    this->addOperation("setJointTorqueControlMode",&LWRSim::setJointTorqueControlMode,this,OwnThread);
    this->addOperation("setJointImpedanceControlMode",&LWRSim::setJointImpedanceControlMode,this,OwnThread);
    this->addOperation("setCartesianImpedanceControlMode",&LWRSim::setCartesianImpedanceControlMode,this,OwnThread);

    this->addOperation("setInitialJointPosition",&LWRSim::setInitialJointPosition,this,OwnThread);

}
void LWRSim::setJointTorqueControlMode(){
        if(!isConfigured())
        {
            log(Error) << "Please configure first, doing nothing." << endlog();
        }
        setJointImpedanceControlMode();
        Eigen::VectorXd kp(joints_idx_.size()),kd(joints_idx_.size());
        kp.setConstant(0.0);
        kd.setConstant(0.0);
        setJointImpedance(kp,kd);
}
bool LWRSim::readyService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res)
{
    return true;
}

void LWRSim::setLinkGravityMode(const std::string& link_name,bool gravity_mode)
{
    // HACK: I want to remove gravity for ati_link (force torque sensor), but
    // <gravity> tag does not work for some reason, so I'm doin' it here.
    // FIXME

    for(gazebo::physics::Link_V::iterator it = this->model_links_.begin();
        it != this->model_links_.end();++it)
        {
            if((*it)->GetName() == link_name)
            {
                gravity_mode_.insert(std::make_pair((*it),gravity_mode));
                RTT::log(RTT::Warning)<<"Setting link "<<link_name<<" to "<<((*it)->GetGravityMode()? "true":"false")<<RTT::endlog();
            }
        }
}

bool LWRSim::gazeboConfigureHook(gazebo::physics::ModelPtr model)
{
    if(model.get() == NULL) {
        RTT::log(RTT::Error)<<"No model could be loaded"<<RTT::endlog();
        return false;
    }

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info)<<"Model has "<<gazebo_joints_.size()<<" joints"<<RTT::endlog();
    RTT::log(RTT::Info)<<"Model has "<<model_links_.size()<<" links"<<RTT::endlog();

    //NOTE: Get the joint names and store their indices
    // Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
    int idx = 0;
    for(gazebo::physics::Joint_V::iterator jit=gazebo_joints_.begin();
        jit != gazebo_joints_.end();++jit,++idx)
    {

        const std::string name = (*jit)->GetName();
        // NOTE: Remove fake fixed joints (revolute with upper==lower==0
        // NOTE: This is not used anymore thanks to <disableFixedJointLumping>
        // Gazebo option (ati_joint is fixed but gazebo can use it )

        if((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u))
        {
            RTT::log(RTT::Warning)<<"Not adding (fake) fixed joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
            continue;
        }
        joints_idx_.push_back(idx);
        joint_names_.push_back(name);
        RTT::log(RTT::Info)<<"Adding joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
    }

    if(joints_idx_.size() == 0)
    {
        RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
        return false;
    }

    RTT::log(RTT::Info)<<"Gazebo model found "<<joints_idx_.size()<<" joints "<<RTT::endlog();

        // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        log(Error) << "Could not load rosparam service." <<endlog();
        return false;
    }

    // We are in the /gazebo namespace,
    // So in the params we have :
    // /gazebo/robot_description
    // /gazebo/robot_name
    // /gazebo/root_link
    // /gazebo/tip_link
    bool ret = true;
    ret &= rosparam->getRelative("robot_name");
    ret &= rosparam->getRelative("root_link");
    ret &= rosparam->getRelative("tip_link");
    rosparam->getRelative("tf_prefix");
    ret &= rosparam->getRelative("robot_description");

    /*if(!tf_prefix_.empty() && *tf_prefix_.rbegin() != '/') tf_prefix_+='/';
    if(tf_prefix_ == "/") tf_prefix_="";

    root_link_ = tf_prefix_ + root_link_;
    tip_link_ = tf_prefix_ + tip_link_;*/

    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,kdl_tree_,kdl_chain_))
    {
        log(Error) << "Error while loading the URDF with params :"
        <<"\n -- root_link  : "<<root_link_
        <<"\n -- tip_link   : "<<tip_link_
        /*<<"\n -- tf_prefix   : "<<tf_prefix_*/
        <<endlog();
        return false;
    }

    id_dyn_solver.reset(new ChainDynParam(kdl_chain_,gravity_vector));
    id_rne_solver.reset(new ChainIdSolver_RNE(kdl_chain_,gravity_vector));
    fk_vel_solver.reset(new ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver.reset(new ChainJntToJacSolver(kdl_chain_));


    // Fake LWR
    jnt_pos_.resize(joints_idx_.size());
    jnt_pos_brakes_.resize(joints_idx_.size());
    jnt_pos_no_dyn_.resize(joints_idx_.size());
    jnt_pos_fri_offset_.resize(joints_idx_.size());
    jnt_pos_old_.resize(joints_idx_.size());
    jnt_vel_.resize(joints_idx_.size());
    jnt_trq_coriolis_kdl_.resize(joints_idx_.size());
    jnt_trq_.resize(joints_idx_.size());
    jnt_trq_raw_.resize(joints_idx_.size());
    grav_trq_.resize(joints_idx_.size());
    jnt_pos_cmd_.resize(joints_idx_.size());
    jnt_trq_cmd_.resize(joints_idx_.size());
    jac_.resize(joints_idx_.size());
    jac_.data.setZero();
    mass_kdl_.resize(joints_idx_.size());
    jnt_trq_gazebo_cmd_.resize(joints_idx_.size());
    prop_joint_offset.resize(joints_idx_.size());
    std::fill(prop_joint_offset.begin(),prop_joint_offset.end(),0.0);

    jnt_trq_gazebo_cmd_.setZero();
    jnt_pos_.setZero();
    jnt_pos_fri_offset_.setZero();
    jnt_pos_old_.setZero();
    jnt_vel_.setZero();
    jnt_trq_.setZero();
    jnt_trq_raw_.setZero();
    grav_trq_.setZero();
    jnt_pos_cmd_.setZero();
    jnt_trq_cmd_.setZero();

    kp_.resize(joints_idx_.size());
    kd_.resize(joints_idx_.size());
    kg_.resize(joints_idx_.size());
    kg_.setConstant(1.0);
    kp_default_.resize(joints_idx_.size());
    kd_default_.resize(joints_idx_.size());
    kcp_.resize(6);
    kcd_.resize(6);
    kcp_default_.resize(6);
    kcd_default_.resize(6);

    ret &= rosparam->getParam(getName() + "/kp_default","kp_default"); // searches in lwr_sim/<param-name>
    ret &= rosparam->getParam(getName() + "/kd_default","kd_default");
    ret &= rosparam->getParam(getName() + "/kcp_default","kcp_default");
    ret &= rosparam->getParam(getName() + "/kcd_default","kcd_default");

    for(int i=0;i<FRI_USER_SIZE;i++)
    {
        fri_from_krl.realData[i]=0;
        fri_from_krl.intData[i]=0;
    }
    fri_from_krl.boolData=0;
    fri_from_krl.fill=0;

    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_states_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_states_cmd_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_states_dyn_);

    jnt_pos_kdl_.resize(joints_idx_.size());
    f_ext_.resize(kdl_chain_.getNrOfSegments());
    jnt_vel_kdl_.resize(joints_idx_.size());
    jnt_acc_kdl_.resize(joints_idx_.size());
    jnt_trq_kdl_.resize(joints_idx_.size());
    jnt_trq_grav_kdl_.resize(joints_idx_.size());

    robot_state.control = static_cast<fri_uint16_t>(FRI_CTRL_POSITION);
    fri_state.quality = static_cast<fri_uint16_t>(FRI_QUALITY_PERFECT);
    fri_to_krl.intData[0] = FRI_STATE_MON;
    fri_from_krl.intData[0] = FRI_STATE_MON;

    pos_limits_.resize(joints_idx_.size());
    vel_limits_.resize(joints_idx_.size());
    trq_limits_.resize(joints_idx_.size());
    pos_limits_ << 170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD;
    vel_limits_ << 112.5*TORAD,112.5*TORAD,112.5*TORAD,112.5*TORAD,180*TORAD,112.5*TORAD,112.5*TORAD;
    trq_limits_ << 200,200,100,100,100,30,30;

    port_JointStates.setDataSample(joint_states_);
    port_JointStatesCommand.setDataSample(joint_states_cmd_);
    port_JointStatesDynamics.setDataSample(joint_states_dyn_);

    port_JointPosition.setDataSample(jnt_pos_);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);
    port_GravityTorque.setDataSample(jnt_trq_grav_kdl_.data);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_kdl_.data);
    port_FromKRL.setDataSample(fri_from_krl);

    resetJointImpedanceGains();
    resetCartesianImpedanceGains();

    log(Info) << getName() << " done configuring gazebo" << endlog();
    return ret;
}

void LWRSim::resetJointImpedanceGains()
{
    this->setJointImpedance(kp_default_,kd_default_);
}
void LWRSim::resetCartesianImpedanceGains()
{
    this->setCartesianImpedance(kcp_default_,kcd_default_);
}
void LWRSim::setInitialJointPosition(const std::vector<double>& jnt_pos_cmd)
{
    if(!jnt_pos_cmd.size() == joints_idx_.size()){
        log(Error) << "Invalid size ( found "<<jnt_pos_cmd.size() << " should be " << joints_idx_.size() << ")" << endlog();
        return;
    }

    jnt_pos_no_dyn_ = VectorXd::Map(jnt_pos_cmd.data(),jnt_pos_cmd.size());
    set_joint_pos_no_dynamics_ = true;
}

bool LWRSim::configureHook(){
    return true;
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
    if(v.size() != joints_idx_.size())
    {
        log(Error) << name<<" vector size error "<<v.size()<<"!="<<joints_idx_.size()<<endlog();
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
    if(! (stiffness.size() == joints_idx_.size() && damping.size() == joints_idx_.size()))
    {
        log(Error) << "Size error, not setting impedance (stiff size "
                            <<stiffness.size()<<" damp size "<<damping.size()
                            <<", should be "<<joints_idx_.size()<<")"<<endlog();
        return false;
    }

    for(unsigned j=0;j<joints_idx_.size();++j)
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
    VectorXd s(joints_idx_.size());
    s.setZero();
    VectorXd d(joints_idx_.size());
    d.setZero();
    return this->setJointImpedance(s,d);
}

bool LWRSim::setCartesianImpedance(const VectorXd& cart_stiffness, const VectorXd& cart_damping)
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

    kcp_ = cart_stiffness;
    kcd_ = cart_damping;

    return true;
}

void LWRSim::updateJointImpedance(const lwr_fri::FriJointImpedance& impedance)
{
    for(unsigned int i=0;i<joints_idx_.size()
        && i<impedance.damping.size()
        && i<impedance.stiffness.size();i++)
    {
        if(impedance.stiffness[i] >=0)
            kp_[i] = impedance.stiffness[i];
        if(impedance.damping[i] >= 0)
            kd_[i] = impedance.damping[i];
    }
}

void LWRSim::updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance)
{
    kcp_[0] = cart_impedance.stiffness.linear.x;
    kcp_[1] = cart_impedance.stiffness.linear.y;
    kcp_[2] = cart_impedance.stiffness.linear.z;

    kcp_[3] = cart_impedance.stiffness.angular.x;
    kcp_[4] = cart_impedance.stiffness.angular.y;
    kcp_[5] = cart_impedance.stiffness.angular.z;

    kcd_[0] = cart_impedance.damping.linear.x;
    kcd_[1] = cart_impedance.damping.linear.y;
    kcd_[2] = cart_impedance.damping.linear.z;

    kcd_[3] = cart_impedance.damping.angular.x;
    kcd_[4] = cart_impedance.damping.angular.y;
    kcd_[5] = cart_impedance.damping.angular.z;

    cart_imp_ = cart_impedance;
}


void LWRSim::setJointImpedanceControlMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_JNT_IMP;
}
void LWRSim::setCartesianImpedanceControlMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_CART_IMP;
}

void LWRSim::updateHook()
{
    RTT::os::MutexLock lock(gazebo_mutex_);
    log(RTT::Debug) << getName() << " UpdateHook() "<< TimeService::Instance()->getNSecs() << endlog();
    static TimeService::nsecs last_tstart,tstart,tstart_wait;
    tstart = TimeService::Instance()->getNSecs();
    // Reset commands from users
    jnt_trq_cmd_.setZero();
    if(set_brakes_ == false) jnt_pos_cmd_ = jnt_pos_;
    Xd_cmd_.setZero();
    F_cmd_.setZero();

    // Do set pos if asked
    if(set_joint_pos_no_dynamics_)
    {
        for(unsigned j=0; j<joints_idx_.size(); j++)
            gazebo_joints_[joints_idx_[j]]->SetPosition(0,jnt_pos_no_dyn_[j]);
        // Set jnt pos
        jnt_pos_cmd_ = jnt_pos_ = jnt_pos_no_dyn_;
        set_joint_pos_no_dynamics_ = false;
    }


    FlowStatus jnt_trq_cmd_fs
                    ,jnt_pos_cmd_fs
                    ,cart_pos_cmd_fs
                    ,cart_wrench_cmd_fs
                    ,fri_to_krl_cmd_fs
                    ,jnt_imp_cmd_fs
                    ,cart_imp_cmd_fs;

    fri_to_krl_cmd_fs = port_ToKRL.readNewest(fri_to_krl);
    fri_state.timestamp = rtt_rosclock::host_now().toNSec();

    // Update Robot Internal State to mimic KRC
    // Update cmds to FRI
    if(fri_to_krl_cmd_fs == NewData)
    {
        for(int i=0;i<FRI_USER_SIZE;i++)
        {
            fri_from_krl.realData[i]=fri_to_krl.realData[i];
            fri_from_krl.intData[i]=fri_to_krl.intData[i];
        }
        fri_from_krl.boolData=fri_to_krl.boolData;
    }
    // Update robot status
    switch(fri_to_krl.intData[1])
    {
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
    // Update robot power state + fri state
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

    if(safety_checks_)
        safetyChecks(jnt_pos_,jnt_vel_,jnt_trq_);

    bool exit_loop = false;
    int n_wait=0;
    tstart_wait = TimeService::Instance()->getNSecs();
    do
    {
        jnt_trq_cmd_fs = port_JointTorqueCommand.readNewest(jnt_trq_cmd_);
        jnt_pos_cmd_fs = port_JointPositionCommand.readNewest(jnt_pos_cmd_);
        cart_pos_cmd_fs = port_CartesianPositionCommand.readNewest(cart_pos_cmd_);

        switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_JNT_IMP:
                if(jnt_trq_cmd_fs == NewData)
                    exit_loop = true;
                break;
            case FRI_CTRL_OTHER:
                exit_loop = true;
                break;
            case FRI_CTRL_POSITION:
                if(jnt_trq_cmd_fs == NewData || jnt_pos_cmd_fs == NewData)
                    exit_loop = true;
                break;
            case FRI_CTRL_CART_IMP:
                if(cart_pos_cmd_fs == NewData)
                    exit_loop = true;
                break;
            default:
                break;
        }
        if(0 && !exit_loop)
        {
            log(RTT::Debug) << getName() << " UpdateHook() : waiting "<< TimeService::Instance()->getNSecs()
            <<" - pos:"<<jnt_trq_cmd_fs
            <<" - trq:"<<jnt_pos_cmd_fs
            <<" - crt:"<<cart_pos_cmd_fs
            <<" - set_joint_pos_no_dynamics_:"<<set_joint_pos_no_dynamics_
            << endlog();
        }
        if(!exit_loop) n_wait++;
    }while(sync_with_cmds_ && !exit_loop && nb_loops_++ > 2);

    TimeService::nsecs tduration_wait = TimeService::Instance()->getNSecs(tstart_wait);

    cart_wrench_cmd_fs = port_CartesianWrenchCommand.readNewest(cart_wrench_cmd_);
    jnt_imp_cmd_fs = port_JointImpedanceCommand.readNewest(jnt_imp_cmd_);
    cart_imp_cmd_fs = port_CartesianImpedanceCommand.readNewest(cart_imp_cmd_);

    // Update joint impedance gains
    if(jnt_imp_cmd_fs == NewData)
        updateJointImpedance(jnt_imp_cmd_);

    // Update cartesian impedance gains
    if(cart_imp_cmd_fs == NewData)
        updateCartesianImpedance(cart_imp_cmd_);

    // Copy State to KDL
    jnt_pos_kdl_.data = jnt_pos_;
    jnt_vel_kdl_.data = jnt_vel_;
    jnt_acc_kdl_.data.setConstant(0.0);

    // Compute MassMatrix
    id_dyn_solver->JntToMass(jnt_pos_kdl_,mass_kdl_);

    // Compute Inverse dynamics [q,qdot] => T
    std::fill(f_ext_.begin(),f_ext_.end(),Wrench::Zero());
    id_rne_solver->CartToJnt(jnt_pos_kdl_,jnt_vel_kdl_,jnt_acc_kdl_,f_ext_,jnt_trq_kdl_);

    // Compute Gravity terms G(q)
    id_dyn_solver->JntToGravity(jnt_pos_kdl_,jnt_trq_grav_kdl_);

    // Compute Coriolis terms b(q,qdot)
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl_,jnt_vel_kdl_,jnt_trq_coriolis_kdl_);

    Map<VectorXd>(joint_states_dyn_.position.data(),joints_idx_.size()) = jnt_trq_grav_kdl_.data;
    Map<VectorXd>(joint_states_dyn_.velocity.data(),joints_idx_.size()) = jnt_trq_coriolis_kdl_.data;
    Map<VectorXd>(joint_states_dyn_.effort.data(),joints_idx_.size()) = jnt_trq_kdl_.data;

    jnt_to_jac_solver->JntToJac(jnt_pos_kdl_,jac_,kdl_chain_.getNrOfSegments());
    fk_vel_solver->JntToCart(JntArrayVel(jnt_pos_kdl_,jnt_vel_kdl_),ee_framevel_kdl_,kdl_chain_.getNrOfSegments());
    ee_twist_kdl_ = ee_framevel_kdl_.GetTwist();
    ee_frame_kdl_ = ee_framevel_kdl_.GetFrame();

    tf::poseKDLToMsg(ee_frame_kdl_,cart_pos_);
    tf::twistKDLToMsg(ee_twist_kdl_,cart_twist_);

    cart_pos_stamped_.header.frame_id = root_link_;
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


    tf::wrenchMsgToEigen(cart_wrench_cmd_,F_cmd_);


    cart_wrench_stamped_.header.frame_id = tip_link_;
    cart_wrench_stamped_.wrench = cart_wrench_;

    jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * jnt_trq_grav_kdl_.data;

    switch(static_cast<FRI_CTRL>(robot_state.control)){
        case FRI_CTRL_JNT_IMP:
            // Joint Impedance part
            jnt_trq_gazebo_cmd_ += kp_.asDiagonal()*(jnt_pos_cmd_ - jnt_pos_) - kd_.asDiagonal()*jnt_vel_ ;
            // Additional torque
            jnt_trq_gazebo_cmd_ += jnt_trq_cmd_;

            set_brakes_ = (jnt_trq_cmd_fs != NewData);

            break;
        case FRI_CTRL_OTHER:
        case FRI_CTRL_POSITION:
            // Joint Position (Joint Imp with kd fixed)
            jnt_trq_gazebo_cmd_ += kp_default_.asDiagonal()*(jnt_pos_cmd_-jnt_pos_) - kd_default_.asDiagonal()*jnt_vel_ ;

            set_brakes_ = (jnt_pos_cmd_fs != NewData);
            break;
        case FRI_CTRL_CART_IMP:
            //Cartesian Impedance Control
            jnt_trq_gazebo_cmd_ += jac_.data.transpose()*(kcp_.asDiagonal()*(X_err_ + F_cmd_) + kcd_.asDiagonal()*(Xd_err_));

            set_brakes_ = (cart_pos_cmd_fs != NewData);

            break;
        default:
            break;
    }

    ros::Time now = rtt_rosclock::host_now();
    //Update status
    cart_pos_stamped_.header.stamp = now;
    cart_wrench_stamped_.header.stamp = now;
    joint_states_.header.stamp = now;

    joint_states_cmd_.header.stamp = now;
    joint_states_dyn_.header.stamp = now;

    Map<VectorXd>(joint_states_.position.data(),joints_idx_.size()) = jnt_pos_;
    Map<VectorXd>(joint_states_.velocity.data(),joints_idx_.size()) = jnt_vel_;
    Map<VectorXd>(joint_states_.effort.data(),joints_idx_.size()) = jnt_trq_;

    Map<VectorXd>(joint_states_cmd_.position.data(),joints_idx_.size()) = jnt_pos_cmd_;
    Map<VectorXd>(joint_states_cmd_.effort.data(),joints_idx_.size()) = jnt_trq_gazebo_cmd_ - jnt_trq_grav_kdl_.data;

    port_JointStates.write(joint_states_);
    port_JointStatesCommand.write(joint_states_cmd_);
    port_JointStatesDynamics.write(joint_states_dyn_);

    port_JointPosition.write(jnt_pos_);
    port_JointVelocity.write(jnt_vel_);
    port_JointTorque.write(jnt_trq_);
    port_GravityTorque.write(jnt_trq_grav_kdl_.data);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianPositionStamped.write(cart_pos_stamped_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);
    port_CartesianWrenchStamped.write(cart_wrench_stamped_);

    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_kdl_.data);

    port_FromKRL.write(fri_from_krl);

    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);

    rtt_sem_.signal();


    TimeService::nsecs tduration = TimeService::Instance()->getNSecs(tstart);
    log(RTT::Debug) << getName() << " UpdateHook()  "
    <<tstart
    <<"\n - jnt_trq_cmd_fs:" << jnt_trq_cmd_fs
    <<"\n - jnt_pos_cmd_fs:" << jnt_pos_cmd_fs
    <<"\n - jnt_trq_cmd_:"<<jnt_trq_cmd_.transpose()
    <<"\n - sync_with_cmds:"<<sync_with_cmds_
    <<"\n - Waited for cmds :"<<n_wait
    <<"\n - Waited for cmds ns :"<<tduration_wait
    <<"\n - set_brakes:"<<set_brakes_
    <<"\n - robot_state.control:"<<robot_state.control
    <<"\n -- kp: "<<kp_.transpose()
    <<"\n -- kd: "<<kd_.transpose()
    <<"\n -- duration: "<<tduration<< endlog();
    return;
}
void LWRSim::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(trylock.isSuccessful() == false) {
        log(RTT::Debug) << getName() << " gazeboUpdateHook() : mutex locked "<< TimeService::Instance()->getNSecs() << endlog();
        rtt_sem_.wait();
        log(RTT::Debug) << getName() << " gazeboUpdateHook() : let's go!"<< TimeService::Instance()->getNSecs() << endlog();
    }
    // Checking if model is correct
    if(model.get() == NULL){
        log(RTT::Debug) << getName() << " gazeboUpdateHook() : model is NULL "<< TimeService::Instance()->getNSecs() << endlog();
        return;
    }
    log(RTT::Debug) << getName() << " gazeboUpdateHook() "<< TimeService::Instance()->getNSecs() << endlog();

    // Read From gazebo simulation
    for(unsigned j=0; j<joints_idx_.size(); j++) {
        jnt_pos_[j] = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
        jnt_vel_[j] = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);
        jnt_trq_[j] = gazebo_joints_[joints_idx_[j]]->GetForce(0u);
    }


    if(port_JointTorqueCommand.connected() || port_JointPositionCommand.connected())
    {
        // Enable gravity for everyone
       for(gazebo::physics::Link_V::iterator it = model_links_.begin();
             it != model_links_.end();++it)
             (*it)->SetGravityMode(true);

        // Set Gravity Mode or specified links
        for(std::map<gazebo::physics::LinkPtr,bool>::iterator it = this->gravity_mode_.begin();
            it != this->gravity_mode_.end();++it)
                    it->first->SetGravityMode(it->second);

        for(unsigned j=0; j<joints_idx_.size(); j++)
            gazebo_joints_[joints_idx_[j]]->SetForce(0,jnt_trq_gazebo_cmd_[j]);
    }
    else
    {
        // If no one is connected, stop gravity
       for(gazebo::physics::Link_V::iterator it = model_links_.begin();
             it != model_links_.end();++it)
             (*it)->SetGravityMode(false);
    }
    //log(RTT::Debug) << getName() << " gazeboUpdateHook() duration " << tduration << endlog();
}

ORO_CREATE_COMPONENT(lwr::LWRSim)
