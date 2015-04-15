// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>

namespace lwr{
using namespace KDL;
Chain KukaLWR_DHnew(){
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
bool LWRSim::configureHook(){
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }

    // Get the ROS parameters
    rosparam->getAllComponentPrivate();

    // Parse URDF from string
    if(!urdf_model_.initString(urdf_str_)) {
        RTT::log(RTT::Error) << "Could not Init URDF." <<RTT::endlog();
        return false;
    }
    RTT::log(RTT::Info) << "URDF : "<<urdf_str_ <<RTT::endlog();
    // Get a KDL tree from the robot URDF

    if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }


    std::string root_link(robot_name_+"/base_link"); //TODO: Get this from urdf
    std::string tip_link(robot_name_+"/lwr_7_link");

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_link, tip_link, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_link<<" --> "<<tip_link);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin();
             it != segment_map.end();
             it++ )
        {
            ROS_ERROR_STREAM( "    "<<(*it).first);
        }

        return false;
    }

    // Pointeur is not null
    kukaLWR_DHnew = KukaLWR_DHnew();
    // Overwrite kdl_chain_
    //kdl_chain_ = kukaLWR_DHnew;
    id_rne_solver = boost::shared_ptr<KDL::ChainIdSolver_RNE>(new KDL::ChainIdSolver_RNE(kdl_chain_,KDL::Vector(0.,0.,-9.81)));

    // Store the number of degrees of freedom of the chain
    n_joints_ = kdl_chain_.getNrOfJoints();

    PeerList lpeers = this->getPeerList();
    for(PeerList::iterator p=lpeers.begin();p!=lpeers.end();++p)
        RTT::log(RTT::Debug)<<" Peer : "<<(*p)<<RTT::endlog();

    if(this->hasPeer("gazebo")){
        this->peer = this->getPeer("gazebo");
        // Getting information from gazebo
        std::string lwr_gazebo("lwr_gazebo");

        port_JointPositionGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointPositionCommand"));
        port_JointVelocityGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointVelocityCommand"));
        port_JointTorqueGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointTorqueCommand"));

        port_JointPositionGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointPosition"));
        port_JointVelocityGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointVelocity"));
        port_JointTorqueGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointTorque"));

        // The number of joints is 8 because it counts base_link, even though it's a fixed joint
        RTT::Attribute<int> njoints = this->peer->getPeer(lwr_gazebo)->getAttribute("n_joints");
        //n_joints_ = njoints.get();//TODO : fix this
        RTT::log(RTT::Debug)<<"LWR njoints GAZEBO : "<<njoints.get()<<RTT::endlog();

        port_JointState.createStream(rtt_roscomm::topic("~"+this->getName()+"/joint_states"));
        port_JointStateFiltered.createStream(rtt_roscomm::topic("~"+this->getName()+"/joint_states_filtered"));
        port_JointStateCommand.createStream(rtt_roscomm::topic("~"+this->getName()+"/joint_states_cmd"));

    }else{
        RTT::log(RTT::Error)<<"Couldn't find gazebo and lwr_gazebo peer"<<RTT::endlog();
        //return false;
    }

    RTT::log(RTT::Debug)<<"LWR njoints : "<<n_joints_<<RTT::endlog();

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
    jnt_pos_old_.resize(n_joints_);
    jnt_vel_.resize(n_joints_);
    jnt_trq_.resize(n_joints_);
    grav_trq_.resize(n_joints_);
    jnt_pos_cmd_.resize(n_joints_);
    jnt_trq_cmd_.resize(n_joints_);
    jac_.resize(n_joints_);
    mass_.resize(n_joints_,n_joints_);

    jnt_pos_.setZero();
    jnt_pos_old_.setZero();
    jnt_vel_.setZero();
    jnt_trq_.setZero();
    grav_trq_.setZero();
    jnt_pos_cmd_.setZero();
    jnt_trq_cmd_.setZero();
    mass_ = Eigen::MatrixXd::Zero(n_joints_,n_joints_);

    for(unsigned int j=0; j < n_joints_; j++){
        std::ostringstream ss;
        ss << j;
        joint_state_.name.push_back(robot_name_+"/"+robot_name_+"_"+ss.str()+"_joint");
        joint_state_.position.push_back(0.0);
        joint_state_.velocity.push_back(0.0);
        joint_state_.effort.push_back(0.0);
    }
    for(unsigned int j=0; j < n_joints_; j++){
        std::ostringstream ss;
        ss << j;
        joint_state_filtered_.name.push_back(robot_name_+"/"+robot_name_+"_"+ss.str()+"_joint");
        joint_state_filtered_.position.push_back(0.0);
        joint_state_filtered_.velocity.push_back(0.0);
        joint_state_filtered_.effort.push_back(0.0);
    }
    for(unsigned int j=0; j < n_joints_; j++){
        std::ostringstream ss;
        ss << j;
        joint_state_cmd_.name.push_back(robot_name_+"/"+robot_name_+"_"+ss.str()+"_joint");
        joint_state_cmd_.position.push_back(0.0);
        joint_state_cmd_.velocity.push_back(0.0);
        joint_state_cmd_.effort.push_back(0.0);
    }

    port_JointState.setDataSample(joint_state_);
    port_JointStateFiltered.setDataSample(joint_state_filtered_);
    port_JointStateCommand.setDataSample(joint_state_cmd_);
    port_JointPosition.setDataSample(jnt_pos_);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);
    port_GravityTorque.setDataSample(grav_trq_);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_);
    return true;
}

void LWRSim::updateHook() {
    read_start = rtt_rosclock::host_now();
    // Read From gazebo simulation
    port_JointPositionGazebo.read(joint_position_gazebo);
    port_JointVelocityGazebo.read(joint_velocity_gazebo);
    port_JointTorqueGazebo.read(joint_torque_gazebo);

    for(unsigned int j=0; j < n_joints_; j++){
        jnt_pos_[j] = joint_position_gazebo[j];
        jnt_vel_[j] = joint_velocity_gazebo[j];
        jnt_trq_[j] = joint_torque_gazebo[j];
    }
    // Read Commands from users
    jnt_trq_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
    jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
    port_JointImpedanceCommand.read(jnt_imp_cmd_);

    read_duration = (rtt_rosclock::host_now() - read_start);

    KDL::JntArray q(kdl_chain_.getNrOfJoints());
    KDL::Wrenches f_ext(kdl_chain_.getNrOfSegments());
    KDL::JntArray G(kdl_chain_.getNrOfJoints());
    KDL::JntArray qdot(kdl_chain_.getNrOfJoints());
    KDL::JntArray qddot(kdl_chain_.getNrOfJoints());

    for(unsigned int j=0; j < n_joints_; j++){
        q(j) = joint_position_gazebo[j];
        qdot(j) = 0.0;
        qddot(j) = 0.0;
    }
    for(unsigned int j=0;j<kdl_chain_.getNrOfSegments();++j)
        f_ext[j] = KDL::Wrench::Zero();

    int ret = id_rne_solver->CartToJnt(q,qdot,qddot,f_ext,G);
    if(ret<0)
        RTT::log(RTT::Warning)<<"ERROR on cart : "<<ret<<RTT::endlog();

    for(unsigned int j=0; j < n_joints_; j++)
        grav_trq_[j] = G(j);

    for(unsigned int j=0; j < n_joints_; j++) {

        jnt_trq_cmd_[j] = kp_*(jnt_pos_cmd_[j]-jnt_pos_[j]) - kd_*jnt_vel_[j] + G(j);
        joint_torque_gazebo_cmd[j] = jnt_trq_cmd_(j);
    }
    now = rtt_rosclock::host_now();
    write_start = now;
    //Update status
    joint_state_.header.stamp = now;
    joint_state_filtered_.header.stamp = now;
    joint_state_cmd_.header.stamp = now;

    Eigen::Map<Eigen::VectorXd>(joint_state_.position.data(),n_joints_) = this->jnt_pos_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.velocity.data(),n_joints_) = this->jnt_vel_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.effort.data(),n_joints_) = this->jnt_trq_;

    Eigen::Map<Eigen::VectorXd>(joint_state_cmd_.position.data(),n_joints_) = this->jnt_pos_cmd_;
    Eigen::Map<Eigen::VectorXd>(joint_state_cmd_.effort.data(),n_joints_) = this->jnt_trq_cmd_;

    for(unsigned int j=0;j<joint_position_gazebo.size();++j)
    {

        joint_state_filtered_.velocity[j] = velocity_smoothing_factor_*(jnt_pos_[j]-joint_state_filtered_.position[j])/this->getPeriod() + (1.0-velocity_smoothing_factor_)*joint_state_filtered_.velocity[j];
        joint_state_filtered_.position[j] = jnt_pos_[j];
        joint_state_filtered_.effort[j] = jnt_trq_[j];

    }

    port_JointState.write(joint_state_);
    port_JointStateFiltered.write(joint_state_filtered_);
    port_JointStateCommand.write(joint_state_cmd_);

    port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);

    port_RobotState.write(m_msr_data.robot);
    port_FRIState.write(m_msr_data.intf);

    port_JointPosition.write(jnt_pos_);
    port_JointVelocity.write(jnt_vel_);
    port_JointTorque.write(jnt_trq_);
    port_GravityTorque.write(grav_trq_);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);

    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_);

    write_duration = (rtt_rosclock::host_now() - write_start);
    updatehook_duration = (rtt_rosclock::host_now() - read_start);

}
}
