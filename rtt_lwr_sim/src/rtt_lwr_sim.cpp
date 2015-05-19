// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include<Eigen/Core>
#include<Eigen/SVD>

namespace Eigen{
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
} 
}
// Wrench ~ inv(J^t)*T (pseudo inverse of Jacobian transpose dot torques)
namespace KDL{
    void MultiplyJacobian(const KDL::Jacobian& jac, const KDL::JntArray& torques, KDL::Wrench& est_wrench)
    {
        Eigen::VectorXd trq = torques.data;
        
        Eigen::MatrixXd Jt = jac.data;
        //Jt.transposeInPlace();
        
        Eigen::VectorXd w(6);
        Eigen::MatrixXd pinvJ = Eigen::pseudoInverse(Jt);
        w = Jt*trq;
        est_wrench.force.x(w(0));
        est_wrench.force.y(w(1));
        est_wrench.force.z(w(2));
        
        est_wrench.torque.x(w(3));
        est_wrench.torque.y(w(4));
        est_wrench.torque.z(w(5));
    }
}

namespace lwr{
KDL::Chain KukaLWR_DHnew(){
    using namespace KDL;
    Chain kukaLWR_DHnew;

    //joint 0
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::None),
                                     Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)
                                     ));
    //joint 1
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                              Vector::Zero(),
                                                                                                              RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

    //joint 2
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,-0.3120511,-0.0038871),
                                                                                                               RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

    //joint 3
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,-0.0015515,0.0),
                                                                                                               RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

    //joint 4
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,0.5216809,0.0),
                                                                                                               RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

    //joint 5
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                              Vector(0.0,0.0119891,0.0),
                                                                                                              RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

    //joint 6
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,0.0080787,0.0),
                                                                                                               RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::Identity(),
                                     RigidBodyInertia(2,
                                                      Vector::Zero(),
                                                      RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));
    return kukaLWR_DHnew;
}
void LWRSim::initJointStateMsg(sensor_msgs::JointState& js,const unsigned int n_joints,const std::string& robot_name)
{
    for(unsigned int j=0; j < n_joints; j++){
        std::ostringstream ss;
        ss << j;
        js.name.push_back(robot_name+"/"+robot_name+"_"+ss.str()+"_joint");
        js.position.push_back(0.0);
        js.velocity.push_back(0.0);
        js.effort.push_back(0.0);
    }
}
void LWRSim::initGains(std::vector< double >& kp, std::vector< double > kd)
{
    kp[0] = 450.0;   kd[0] = 1.0;
    kp[1] = 450.0;   kd[1] = 1.0;
    kp[2] = 200.0;   kd[2] = 0.7;
    kp[3] = 200.0;   kd[3] = 0.7;
    kp[4] = 200.0;   kd[4] = 0.7;
    kp[5] = 10.0;    kd[5] = 0.1;
    kp[6] = 10.0;    kd[6] = 0.0;
    this->setImpedance(kp,kd);
}

bool LWRSim::configureHook(){
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }

    // Set the Robot name (lwr or lwr_sim) For other components to know it
    rosparam->setPrivate("robot_name");
    rosparam->getPrivate("root_link");
    rosparam->getPrivate("tip_link");

    RTT::log(RTT::Info)<<"root_link : "<<root_link<<RTT::endlog();
    RTT::log(RTT::Info)<<"tip_link : "<<tip_link<<RTT::endlog();
    
    KDL::Vector gravity_vector(0.,0.,-9.81289);
    
    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,root_link,tip_link,kdl_tree_,kdl_chain_))
        return false;
    
    ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv_nso(kdl_chain_));
    id_dyn_solver.reset(new KDL::ChainDynParam(kdl_chain_,gravity_vector));
    id_rne_solver.reset(new KDL::ChainIdSolver_RNE(kdl_chain_,gravity_vector));
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            
      // Pointeur is not null
    kukaLWR_DHnew = KukaLWR_DHnew();
    // Overwrite kdl_chain_
    //kdl_chain_ = kukaLWR_DHnew;  
        
    // Store the number of degrees of freedom of the chain
    n_joints_ = kdl_chain_.getNrOfJoints();

    PeerList lpeers = this->getPeerList();
    for(PeerList::iterator p=lpeers.begin();p!=lpeers.end();++p)
        RTT::log(RTT::Debug)<<" Peer : "<<(*p)<<RTT::endlog();

    if(this->hasPeer("gazebo")){
        this->peer = this->getPeer("gazebo");
        // Getting information from gazebo
        std::string lwr_gazebo("lwr_gazebo");
        RTT::ConnPolicy policy = RTT::ConnPolicy::data();
        port_JointPositionGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointPositionCommand"),policy);
        port_JointVelocityGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointVelocityCommand"),policy);
        port_JointTorqueGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointTorqueCommand"),policy);

        port_JointPositionGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointPosition"),policy);
        port_JointVelocityGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointVelocity"),policy);
        port_JointTorqueGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointTorque"),policy);

        // The number of joints is 8 because it counts base_link, even though it's a fixed joint
        RTT::Property<unsigned int> njoints = this->peer->getPeer(lwr_gazebo)->getProperty("n_joints");
        RTT::log(RTT::Debug)<<"LWR njoints GAZEBO : "<<njoints.get()<<RTT::endlog();

    }else{
        RTT::log(RTT::Error)<<"Couldn't find gazebo and lwr_gazebo peer"<<RTT::endlog();
        return false;
    }

    RTT::log(RTT::Debug)<<"LWR njoints : "<<n_joints_<<RTT::endlog();
    
    if(n_joints_ != LBR_MNJ)
    {
        RTT::log(RTT::Error)<<"Number of joints error : found "<<n_joints_<<" but should be "<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    
    
    initGains(kp_,kd_);
    
    
    // In from gazebo
    joint_position_gazebo.resize(n_joints_,0.0);
    joint_velocity_gazebo.resize(n_joints_,0.0);
    joint_torque_gazebo.resize(n_joints_,0.0);
    // Out to gazebo
    joint_position_gazebo_cmd.resize(n_joints_,0.0);
    joint_velocity_gazebo_cmd.resize(n_joints_,0.0);
    joint_torque_gazebo_cmd.resize(n_joints_,0.0);
    // Fake LWR
    jnt_pos_.resize(n_joints_);
    jnt_pos_fri_offset.resize(n_joints_);
    jnt_pos_old_.resize(n_joints_);
    jnt_vel_.resize(n_joints_);
    jnt_trq_.resize(n_joints_);
    jnt_trq_raw_.resize(n_joints_);
    grav_trq_.resize(n_joints_);
    jnt_pos_cmd_.resize(n_joints_);
    jnt_trq_cmd_.resize(n_joints_);
    jac_.resize(n_joints_);
    jac_.data.setZero();
    mass_.resize(n_joints_,n_joints_);

    jnt_pos_.setZero();
    jnt_pos_fri_offset.setZero();
    jnt_pos_old_.setZero();
    jnt_vel_.setZero();
    jnt_trq_.setZero();
    jnt_trq_raw_.setZero();
    grav_trq_.setZero();
    jnt_pos_cmd_.setZero();
    jnt_trq_cmd_.setZero();
    mass_.setZero();// = Eigen::MatrixXd::Zero(n_joints_,n_joints_);

    initJointStateMsg(joint_state_,n_joints_,robot_name_);
    initJointStateMsg(joint_state_cmd_,n_joints_,robot_name_);
    initJointStateMsg(joint_state_filtered_,n_joints_,robot_name_);
    initJointStateMsg(joint_state_gravity_,n_joints_,robot_name_);

    port_JointState.setDataSample(joint_state_);
    port_JointStateFiltered.setDataSample(joint_state_filtered_);
    port_JointStateCommand.setDataSample(joint_state_cmd_);
    port_JointStateGravity.setDataSample(joint_state_gravity_);
    
    port_JointPosition.setDataSample(jnt_pos_);
    port_JointPositionFRIOffset.setDataSample(jnt_pos_fri_offset);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);
    port_JointTorqueRaw.setDataSample(jnt_trq_raw_);
    port_GravityTorque.setDataSample(grav_trq_);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_);
    
    port_JointState.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states"));
    port_JointStateFiltered.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_filtered"));
    port_JointStateCommand.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_cmd"));
    port_JointStateGravity.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_gravity"));
        
    port_CartesianPosition.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_pose"));
    port_CartesianVelocity.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_twist"));
    port_CartesianWrench.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_wrench"));
    
    q.resize(n_joints_);
    f_ext.resize(kdl_chain_.getNrOfSegments());
    G.resize(n_joints_);
    qdot.resize(n_joints_);
    qddot.resize(n_joints_);
    jnt_trq_kdl_.resize(n_joints_);
    
    robot_state.control = static_cast<FRI_CTRL>(FRI_CTRL_POSITION);
    fri_state.quality = static_cast<FRI_QUALITY>(FRI_QUALITY_PERFECT);
    fri_to_krl.intData[0] = FRI_STATE_MON;
    fri_from_krl.intData[0] = FRI_STATE_MON;
    
    return true;
}

bool LWRSim::safetyChecks(const std::vector<double>& position,const std::vector<double>& velocity,const std::vector<double>& torque)
{
    return positionSafetyCheck(position) &&
    velocitySafetyCheck(velocity) &&
    torqueSafetyCheck(torque);
}

bool LWRSim::positionSafetyCheck(const std::vector<double>& position)
{
    if(position.size() != LBR_MNJ)
    {
        RTT::log(RTT::Error) << "Vector size error "<<position.size()<<"!="<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    const double limits[] = {170,120,170,120,170,120,170};
    bool ret=true;
    for(unsigned i=0;i<position.size();i++)
    {
        if(std::abs(position[i]) > limits[i])
        {
           RTT::log(RTT::Error) << "Position limit exceded at J"<<i<<" : "<<position[i]<<" / limit "<<limits[i]<<RTT::endlog();
           ret = false;
        }
    }
    return ret;
}
bool LWRSim::velocitySafetyCheck(const std::vector<double>& velocity)
{
    if(velocity.size() != LBR_MNJ)
    {
        RTT::log(RTT::Error) << "Velocity size error "<<velocity.size()<<"!="<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    const double limits[] = {112.5,112.5,112.5,112.5,180,112.5,112.5};
    bool ret=true;
    for(unsigned i=0;i<velocity.size();i++)
    {
        if(std::abs(velocity[i]) > limits[i])
        {
           RTT::log(RTT::Error) << "Velocity limit exceded at J"<<i<<" : "<<velocity[i]<<" / limit "<<limits[i]<<RTT::endlog();
           ret = false;
        }
    }
    return ret;
}

bool LWRSim::torqueSafetyCheck(const std::vector<double>& torque)
{
    if(torque.size() != LBR_MNJ)
    {
        RTT::log(RTT::Error) << "Torque size error "<<torque.size()<<"!="<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    const double limits[] = {200,200,100,100,100,30,30};
    bool ret=true;
    for(unsigned i=0;i<torque.size();i++)
    {
        if(std::abs(torque[i]) > limits[i])
        {
           RTT::log(RTT::Error) << "Torque limit exceded at J"<<i<<" : "<<torque[i]<<" / limit "<<limits[i]<<RTT::endlog();
           ret = false;
        }
    }
    return ret;
}
bool LWRSim::setImpedance(const std::vector< double >& stiffness, const std::vector< double >& damping)
{
    if(! (stiffness.size() == LBR_MNJ && damping.size() == LBR_MNJ))
    {
        RTT::log(RTT::Error) << "Size error, not setting impedance (stiff size "
                            <<stiffness.size()<<" damp size "<<damping.size()
                            <<", should be "<<LBR_MNJ<<")"<<RTT::endlog();
        return false;
    }
    kd_=damping;
    kp_=stiffness;
    return true;
}

void LWRSim::updateHook() {
    read_start = rtt_rosclock::host_now().toSec();
    // Read From gazebo simulation
    RTT::FlowStatus fs_p = port_JointPositionGazebo.read(joint_position_gazebo);
    RTT::FlowStatus fs_v = port_JointVelocityGazebo.read(joint_velocity_gazebo);
    RTT::FlowStatus fs_g = port_JointTorqueGazebo.read(joint_torque_gazebo);

    fri_state.timestamp = read_start;

    if(port_ToKRL.read(fri_to_krl) != RTT::NoData)
    {
        fri_from_krl = fri_to_krl;
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
                break;
            case FRI_CTRL_OTHER:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_MON;
                break;
            default:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_OFF;
                break;
        }
    port_FromKRL.write(fri_from_krl);
    
    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);
    
    if(fs_g == RTT::NoData || fs_p == RTT::NoData || fs_v == RTT::NoData)
        return;
    
    //safetyChecks(joint_position_gazebo,joint_velocity_gazebo,joint_torque_gazebo);
    
    jnt_pos_ = Eigen::VectorXd::Map(joint_position_gazebo.data(),n_joints_);
    jnt_vel_ = Eigen::VectorXd::Map(joint_velocity_gazebo.data(),n_joints_);
    jnt_trq_ = Eigen::VectorXd::Map(joint_torque_gazebo.data(),n_joints_);

    // Read Commands from users
    jnt_trq_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
    jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
    port_JointImpedanceCommand.read(jnt_imp_cmd_);

    read_duration = (rtt_rosclock::host_now().toSec() - read_start);

    
    for(unsigned int j=0; j < n_joints_; j++){
        q.q(j) = jnt_pos_[j];
        q.qdot(j) = jnt_vel_[j];
        qdot(j) = 0.0;
        qddot(j) = 0.0;
        jnt_trq_kdl_(j) = jnt_trq_[j];
    }
    for(unsigned int j=0;j<kdl_chain_.getNrOfSegments();++j)
        f_ext[j] = KDL::Wrench::Zero();

    int ret = id_dyn_solver->JntToGravity(q.q,G);
    if(ret<0)
        RTT::log(RTT::Warning)<<"ERROR on cart : "<<ret<<RTT::endlog();

    grav_trq_ = G.data;

    jnt_to_jac_solver->JntToJac(this->q.q,jac_);
    KDL::FrameVel endToBase;
    fk_vel_solver->JntToCart(q,endToBase);
    KDL::Twist endTwist = endToBase.GetTwist();
    KDL::Frame endFrame = endToBase.GetFrame();
    KDL::Vector endPos = endFrame.p;
    KDL::Rotation endRot = endFrame.M;

    
    cart_pos_.position.x = endPos[0];
    cart_pos_.position.y = endPos[1];
    cart_pos_.position.z = endPos[2];
    
    endRot.GetQuaternion(cart_pos_.orientation.x,
                         cart_pos_.orientation.y,
                         cart_pos_.orientation.z,
                         cart_pos_.orientation.w
                        );
    
    cart_twist_.linear.x = endTwist.vel[0];
    cart_twist_.linear.y = endTwist.vel[1];
    cart_twist_.linear.z = endTwist.vel[2];
    
    cart_twist_.angular.x = endTwist.rot[0];
    cart_twist_.angular.y = endTwist.rot[1];
    cart_twist_.angular.z = endTwist.rot[2];

    KDL::MultiplyJacobian(jac_,jnt_trq_kdl_,cart_wrench_kdl_);
    cart_wrench_.force.x = cart_wrench_kdl_.force.x();
    cart_wrench_.force.y = cart_wrench_kdl_.force.y();
    cart_wrench_.force.z = cart_wrench_kdl_.force.z();
    cart_wrench_.torque.x = cart_wrench_kdl_.torque.x();
    cart_wrench_.torque.y = cart_wrench_kdl_.torque.y();
    cart_wrench_.torque.z = cart_wrench_kdl_.torque.z();
    //cart_wrench_; // TODO: GET DATA FROM ATI SENSOR
    
    id_dyn_solver->JntToMass(q.q,H);
    
    mass_ = H.data;
    
    if(jnt_pos_fs != RTT::NoData || jnt_trq_fs != RTT::NoData){
        //Impedance Control
        for(unsigned int j=0; j < n_joints_; j++) {
            joint_torque_gazebo_cmd[j] = kp_[j]*(jnt_pos_cmd_[j]-jnt_pos_[j]) - kd_[j]*jnt_vel_[j] + jnt_trq_cmd_[j] + kg_[j]*G(j);
        }
    }
    
    now = rtt_rosclock::host_now();
    write_start = now.toNSec();
    //Update status
    joint_state_.header.stamp = now;
    joint_state_filtered_.header.stamp = now;
    joint_state_cmd_.header.stamp = now;
    joint_state_gravity_.header.stamp = now;

    Eigen::Map<Eigen::VectorXd>(joint_state_gravity_.effort.data(),n_joints_) = grav_trq_;
    
    Eigen::Map<Eigen::VectorXd>(joint_state_.position.data(),n_joints_) = jnt_pos_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.velocity.data(),n_joints_) = jnt_vel_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.effort.data(),n_joints_) = jnt_trq_;

    Eigen::Map<Eigen::VectorXd>(joint_state_cmd_.position.data(),n_joints_) = jnt_pos_cmd_;
    joint_state_cmd_.effort = joint_torque_gazebo_cmd;

    for(unsigned int j=0;j<joint_position_gazebo.size();++j)
    {
        joint_state_filtered_.velocity[j] = velocity_smoothing_factor_*(jnt_pos_[j]-joint_state_filtered_.position[j])/this->getPeriod() + (1.0-velocity_smoothing_factor_)*joint_state_filtered_.velocity[j];
    }
    Eigen::Map<Eigen::VectorXd>(joint_state_filtered_.position.data(),n_joints_) = jnt_pos_;
    Eigen::Map<Eigen::VectorXd>(joint_state_filtered_.effort.data(),n_joints_) = jnt_trq_;
    
    port_JointState.write(joint_state_);
    port_JointStateFiltered.write(joint_state_filtered_);
    port_JointStateGravity.write(joint_state_gravity_);

    port_JointPosition.write(jnt_pos_);
    port_JointVelocity.write(jnt_vel_);
    port_JointTorque.write(jnt_trq_);
    port_GravityTorque.write(grav_trq_);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);

    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_);
    
    port_JointStateCommand.write(joint_state_cmd_);
    
    port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
    
    write_duration = (rtt_rosclock::host_now().toSec() - write_start);
    updatehook_duration = (rtt_rosclock::host_now().toSec() - read_start);

}
}
