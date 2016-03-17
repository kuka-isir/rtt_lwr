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
timeout_s(2.0),
gravity_vector(0.,0.,-9.81289)
{
    this->provides("gazebo")->addOperation("configure",&LWRSim::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&LWRSim::gazeboUpdateHook,this,RTT::ClientThread);
    this->addOperation("setLinkGravityMode",&LWRSim::setLinkGravityMode,this,RTT::ClientThread);
    this->addOperation("ready",&LWRSim  ::readyService,this,RTT::ClientThread);
    //this->addAttribute("fromKRL", m_fromKRL);
    //this->addProperty("using_corba", using_corba).doc("");
    this->addProperty("synchronize_with_commands", sync_with_cmds_).doc("");
    this->addProperty("synchronize_commands_timeout_s", timeout_s).doc("");
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
    // So in the params we have (default) :
    // /robot_description
    // /robot_name
    // /root_link
    // /tip_link
    bool ret = true;
    rosparam->getRelative("robot_name");
    rosparam->getRelative("root_link");
    rosparam->getRelative("tip_link");
    rosparam->getRelative("tf_prefix");
    rosparam->getRelative("robot_description");

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
    jnt_pos_cmd_.setZero();
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

ORO_CREATE_COMPONENT(lwr::LWRSim)
