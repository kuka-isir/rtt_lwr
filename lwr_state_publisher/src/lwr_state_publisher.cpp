// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#include <rtt/Component.hpp>
#include <sensor_msgs/JointState.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <urdf/model.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <boost/circular_buffer.hpp>
#include <boost/assign/std/vector.hpp>
#include <std_msgs/Float32.h>

class LWRStatePublisher : public RTT::TaskContext{
public:
    LWRStatePublisher(std::string const& name):
        robot_name_("lwr"),
        robot_namespace_("lwr"),
        link_prefix_("lwr"),
        n_joints_(7),
        peer(NULL),
        cnt_(0),
        jnt_pos_bdf(7,Eigen::Matrix<double,7,1>::Zero()),
        current_buff_idx(0),
        bdf_order_(3),
        ks_(1),
        RTT::TaskContext(name){
        this->addProperty("robot_name",robot_name_);
        this->addProperty("robot_namespace",robot_namespace_);
        this->addProperty("link_prefix",link_prefix_);
        this->ports()->addPort("JointPosition", port_JointPosition).doc("");
        this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
        this->ports()->addPort("JointTorque", port_JointTorque).doc("");
        this->ports()->addPort("JointState", port_JointState).doc("");
        this->ports()->addPort("port_JointStateVelocityBDF", port_JointStateVelocityBDF).doc("");
        //this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
        this->ports()->addPort("dt", port_dt).doc("");
        this->addAttribute("ks",ks_);
        this->addAttribute("BDFOrder",bdf_order_);

    }
    virtual bool configureHook(){
        RTT::log(RTT::Debug) << "Robot Name : "<<robot_name_ << RTT::endlog();
        if(this->hasPeer(robot_name_)){
            this->peer = this->getPeer(robot_name_);
            RTT::ConnPolicy policy = RTT::ConnPolicy::data();
            this->port_JointPosition.connectTo(this->peer->getPort("JointPosition"),policy);
            this->port_JointVelocity.connectTo(this->peer->getPort("JointVelocity"),policy);
            this->port_JointTorque.connectTo(this->peer->getPort("JointTorque"),policy);
            RTT::Property<unsigned int> n_joints_prop = this->peer->getProperty("n_joints");
            RTT::Property<std::string> robot_name_prop = this->peer->getProperty("robot_name");
            n_joints_ = n_joints_prop.get();
            robot_name_ = robot_name_prop.get();
            //std::string urdf_str = this->peer->getAttribute<std::string>("robot_description")->get();
            //this->urdf.initString(urdf_str);
            //this->port_JointTorqueCommand.connectTo(this->peer->getPort("JointPositionCommand"),policy);

        }else{
            RTT::log(RTT::Warning)<<"Couldn't find "<<robot_name_<<" peer"<<RTT::endlog();
            //return false;
        }

        this->port_JointState.createStream(rtt_roscomm::topic(robot_namespace_+"/joint_states"));
        this->port_JointStateVelocityBDF.createStream(rtt_roscomm::topic(robot_namespace_+"/joint_states_velocity_bdf"));
        this->port_dt.createStream(rtt_roscomm::topic(robot_namespace_+"/dt"));

        this->joint_position.resize(n_joints_);
        this->joint_velocity.resize(n_joints_);
        this->joint_velocity_bdf.resize(n_joints_);
        this->joint_torque.resize(n_joints_);
        //this->joint_torque_command.resize(n_joints_);
        jnt_pos_filt.resize(n_joints_);
        this->joint_position.setZero();
        this->joint_velocity.setZero();
        this->joint_velocity_bdf.setZero();
        this->joint_torque.setZero();
        //this->joint_torque_command.setZero();
        jnt_pos_filt.setZero();
        for(unsigned i=0;i<n_joints_;++i)
        {
            std::ostringstream ss;
            ss << i;
            this->joint_state.name.push_back(robot_namespace_+"/"+link_prefix_+"_"+ss.str()+"_joint");
            this->joint_state.position.push_back(0.0);
            this->joint_state.velocity.push_back(0.0);
            this->joint_state.effort.push_back(0.0);

            this->joint_state_velocity_bdf.name.push_back(robot_namespace_+"/"+link_prefix_+"_"+ss.str()+"_joint");
            this->joint_state_velocity_bdf.position.push_back(0.0);
            this->joint_state_velocity_bdf.velocity.push_back(0.0);
            this->joint_state_velocity_bdf.effort.push_back(0.0);
            
        }
        port_JointState.setDataSample(this->joint_state);
        port_JointStateVelocityBDF.setDataSample(joint_state_velocity_bdf);
        last = rtt_rosclock::host_now();
        return true;
    }

    virtual void updateHook() {
        cnt_++;
        
        this->joint_position_fs = port_JointPosition.read(joint_position);
        this->joint_velocity_fs = port_JointVelocity.read(joint_velocity);
        this->joint_torque_fs = port_JointTorque.read(joint_torque);

        
        if(! (this->joint_position_fs == RTT::NewData
                && this->joint_velocity_fs == RTT::NewData
                && this->joint_torque_fs == RTT::NewData))
        {
            //RTT::log(RTT::Warning)<<"No data"<<RTT::endlog();
            return;
        }
        
        now = rtt_rosclock::host_now();
        dt_ = (now-last).toSec();
        dt_out.data = dt_;
        last = now;
        
        this->joint_state.header.stamp = now;
    

        if(jnt_pos_bdf.size()){
          for(unsigned j=0;j<n_joints_;++j)
                jnt_pos_filt[j] = 0.9*joint_position[j]+0.1*jnt_pos_bdf[jnt_pos_bdf.size()-1][j];
        }else
          jnt_pos_filt = joint_position;
        this->jnt_pos_bdf.push_back(jnt_pos_filt);
        estimateVelocityBDF(bdf_order_,dt_,this->jnt_pos_bdf,this->joint_velocity_bdf);
        
        this->joint_state_velocity_bdf.header.stamp = now;
        
        for(unsigned j=0;j<n_joints_;++j)
                if(joint_velocity_bdf[j]==0)
                    joint_velocity_bdf[j] = joint_velocity[j];
                
        Eigen::Map<Eigen::VectorXd>(joint_state_velocity_bdf.position.data(),n_joints_) = jnt_pos_filt;
        Eigen::Map<Eigen::VectorXd>(joint_state_velocity_bdf.velocity.data(),n_joints_) = joint_velocity_bdf;
    
        Eigen::Map<Eigen::VectorXd>(joint_state.position.data(),n_joints_) = joint_position;
        Eigen::Map<Eigen::VectorXd>(joint_state.velocity.data(),n_joints_) = joint_velocity;
        Eigen::Map<Eigen::VectorXd>(joint_state.effort.data(),n_joints_) = joint_torque;
        
        
        /*for(unsigned i=0;i<jnt_pos_bdf.size();++i)
            RTT::log(RTT::Warning)<<"jnt_pos_bdf "<<i<<" : "<<jnt_pos_bdf[i].transpose()<<RTT::endlog();*/
        
        /*for(unsigned i=0;i<n_joints_;++i)
        {
            if(i>=0)
                this->joint_torque_command[i] = 60*3.14/180.0*sin(cnt_*getPeriod()*ks_);//*double(i+1));
            else
                this->joint_torque_command[i] = 0.0;
        }*/

        


        if(this->joint_state.header.stamp.isZero() && (this->joint_position_fs == RTT::NewData))
        {
            RTT::log(RTT::Error)<<"rtt_rosclock::host_now() returned 0, please check if /clock is published"<<RTT::endlog();
            this->joint_state.header.stamp = rtt_rosclock::rtt_now();
        }
        this->port_JointState.write(joint_state);
        this->port_JointStateVelocityBDF.write(joint_state_velocity_bdf);
        
        //this->port_JointTorqueCommand.write(joint_torque_command);
        this->port_dt.write(dt_out);
        this->getActivity()->trigger();
    }

public:
    static void estimateVelocityBDF(unsigned int order,const double dt,const boost::circular_buffer<Eigen::VectorXd>& x_states,Eigen::VectorXd& xd)
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
    
    boost::circular_buffer<Eigen::VectorXd> jnt_pos_bdf;
    RTT::OutputPort<std_msgs::Float32> port_dt;
    sensor_msgs::JointState joint_state,joint_state_velocity_bdf;
    RTT::OutputPort<sensor_msgs::JointState> port_JointState,port_JointStateVelocityBDF;
    RTT::InputPort<Eigen::VectorXd > port_JointPosition;
    RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
    RTT::InputPort<Eigen::VectorXd > port_JointTorque;

    //RTT::OutputPort<Eigen::VectorXd > port_JointTorqueCommand;

    RTT::FlowStatus joint_position_fs;
    RTT::FlowStatus joint_velocity_fs;
    RTT::FlowStatus joint_torque_fs;
    Eigen::VectorXd joint_position,jnt_pos_filt;
    Eigen::VectorXd joint_velocity,joint_velocity_bdf;
    unsigned int current_buff_idx;
    Eigen::VectorXd joint_torque;

    //Eigen::VectorXd joint_torque_command;

    RTT::TaskContext* peer;
    ros::Time now;
    int n_joints_;
    std::string robot_name_,robot_namespace_,link_prefix_;
    int cnt_;
    urdf::Model urdf;
    double dt_,ks_;
    ros::Time last;
    unsigned int bdf_order_;
    std_msgs::Float32 dt_out;
};

ORO_CREATE_COMPONENT(LWRStatePublisher)
