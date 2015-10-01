#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

//#include <Eigen/Dense>
#include <boost/graph/graph_concepts.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt_rosclock/rtt_rosclock.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/JointState.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <std_srvs/Empty.h>
#include <rtt_roscomm/rosservice.h>

class LWRGazeboComponent : public RTT::TaskContext
{
public:


    LWRGazeboComponent(std::string const& name) :
        RTT::TaskContext(name),
        steps_rtt_(0),
        steps_gz_(0),
        n_joints_(0),
        rtt_done(true),
        gazebo_done(false),
        new_data(true),
        cnt_lock_(100),
        last_steps_rtt_(0),
        set_new_pos(false),
        nb_no_data_(0),
        using_ros_topics(true),
        set_brakes(false),
        nb_static_joints(0),// HACK: The urdf has static tag for base_link, which makes it appear in gazebo as a joint
        last_gz_update_time_(0,0)
    {
        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&LWRGazeboComponent::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&LWRGazeboComponent::gazeboUpdateHook,this,RTT::ClientThread);
        this->addOperation("ready",&LWRGazeboComponent::readyService,this,RTT::ClientThread);

        this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
        this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
        this->ports()->addPort("JointVelocityCommand", port_JointVelocityCommand).doc("");
        
        this->ports()->addPort("JointStatesCommand", port_JointStatesCommand).doc("");
        this->ports()->addPort("JointStates", port_JointStates).doc("");
        
        port_JointStates.createStream(rtt_roscomm::topic("/"+getName()+"/joint_states"));
        port_JointStatesCommand.createStream(rtt_roscomm::topic("/"+getName()+"/joint_states_cmd"));
        
        this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
        this->ports()->addPort("JointTorque", port_JointTorque).doc("");
        this->ports()->addPort("JointPosition", port_JointPosition).doc("");

        this->addProperty("use_ros_topics",using_ros_topics);
        
        this->provides("debug")->addAttribute("jnt_pos",jnt_pos_);
        this->provides("debug")->addAttribute("jnt_vel",jnt_vel_);
        this->provides("debug")->addAttribute("jnt_trq",jnt_trq_);
        this->provides("debug")->addAttribute("gz_time",gz_time_);
        this->provides("debug")->addAttribute("write_duration",write_duration_);
        this->provides("debug")->addAttribute("read_duration",read_duration_);
        this->provides("debug")->addAttribute("rtt_time",rtt_time_);
        this->provides("debug")->addAttribute("steps_rtt",steps_rtt_);
        this->provides("debug")->addAttribute("steps_gz",steps_gz_);
        this->provides("debug")->addAttribute("period_sim",period_sim_);
        this->provides("debug")->addAttribute("period_wall",period_wall_);
        
        this->addProperty("n_joints",n_joints_);
        

          
    }

    // Let everyone know that rtt_gazebo is loaded
    bool readyService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res)
    {
        return true;
    }
    
    //! Called from gazebo
    virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model)
    {
        if(model.get() == NULL) {
            RTT::log(RTT::Error)<<"No model could be loaded"<<RTT::endlog();
            return false;
        }

        // Get the joints
        gazebo_joints_ = model->GetJoints();
        model_links_ = model->GetLinks();
        n_joints_ = 0;

        RTT::log(RTT::Info)<<"Model has "<<gazebo_joints_.size()<<" joints"<<RTT::endlog();
        RTT::log(RTT::Info)<<"Model has "<<model_links_.size()<<" links"<<RTT::endlog();
        // Get the joint names 
        // HACK: base_joint is inside gazebo_joints,
        // but it's a fixed joint. So I'm doing this hack to pass it
        for(gazebo::physics::Link_V::iterator it=model_links_.begin();
            it != model_links_.end();++it)
        {
            RTT::log(RTT::Info)<<"Link "<<(*it)->GetName()<<RTT::endlog();
            if((*it)->IsStatic())
            {
                //NOTE: Does NOT work for our problem, must use other hack
                nb_static_joints++;
                continue;
            }
            gazebo::physics::Joint_V joints = (*it)->GetChildJoints();
            
            for(gazebo::physics::Joint_V::iterator jit=joints.begin();
                jit != joints.end();++jit)
            {
                const std::string name = (*jit)->GetName();
                (*jit)->SetProvideFeedback(true);
                joint_names_.push_back(name);
                RTT::log(RTT::Info)<<"Adding joint "<<name<<RTT::endlog();
            }

        }
        
        //HACK: last solution to get rid of the static frame
        nb_static_joints = gazebo_joints_.size() - joint_names_.size();
        
        
        RTT::log(RTT::Info)<<"Model has "<<nb_static_joints<<" static joints"<<RTT::endlog();
        
        n_joints_ = joint_names_.size();
        
        if(n_joints_ == 0)
        {
            RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
            return false;
        }
        
        RTT::log(RTT::Info)<<"Gazebo model found "<<n_joints_<<" joints "<<RTT::endlog();

        for(unsigned j=0; j < n_joints_; j++){
            jnt_pos_cmd_.push_back(0.0);
            jnt_vel_cmd_.push_back(0.0);
            jnt_trq_cmd_.push_back(0.0);
            jnt_pos_.push_back(0.0);
            jnt_vel_.push_back(0.0);
            jnt_trq_.push_back(0.0);
        }

        js.effort = jnt_trq_;
        js.position = jnt_pos_;
        js.velocity = jnt_vel_;

        js.effort = jnt_trq_cmd_;
        js.position = jnt_pos_cmd_;
        js.velocity = jnt_vel_cmd_;
        
        js.name = joint_names_;
        js_cmd.name = joint_names_;
        
        port_JointPosition.setDataSample(jnt_pos_);
        port_JointVelocity.setDataSample(jnt_vel_);
        port_JointTorque.setDataSample(jnt_trq_);
        
        RTT::log(RTT::Info)<<"Done configuring gazebo"<<RTT::endlog();
        last_update_time_ = rtt_rosclock::rtt_now();
        
        boost::shared_ptr<rtt_rosservice::ROSService> rosservice =
        this->getProvider<rtt_rosservice::ROSService>("rosservice");
        if(rosservice)
        {
            std::cout << "Trying to set the ROSService" << std::endl;
            bool ret = rosservice->connect("ready","/"+this->getName()+"/ready","std_srvs/Empty");
            std::cout << "We get "<<ret << RTT::endlog();
        }else{
            std::cerr << "Could not load rosservice" << std::endl;
        }
        
        return true;
    }

    //! Called from Gazebo
    virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model)
    {
        if(model.get() == NULL) {return;}

        // Synchronize with update()
#ifdef XENOMAI
        //gazebo_mutex_.lock();
        if(rtt_done){
            gazebo_done=false;
#else
        RTT::os::MutexTryLock trylock(gazebo_mutex_);
        if(trylock.isSuccessful()) {
#endif
            // Increment simulation step counter (debugging)
            steps_gz_++;

            // Get the RTT and gazebo time for debugging purposes
            rtt_time_ = 1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
            gazebo::common::Time gz_time = model->GetWorld()->GetSimTime();
            gz_time_ = (double)gz_time.sec + ((double)gz_time.nsec)*1E-9;

            // Get the wall time
            gazebo::common::Time gz_wall_time = gazebo::common::Time::GetWallTime();
            wall_time_ = (double)gz_wall_time.sec + ((double)gz_wall_time.nsec)*1E-9;

            // Get state
            //gazebo::physics::JointWrench jw;
            for(unsigned j=0; j<n_joints_; j++) {
                jnt_pos_[j] = gazebo_joints_[j+nb_static_joints]->GetAngle(0).Radian();
                jnt_vel_[j] = gazebo_joints_[j+nb_static_joints]->GetVelocity(0);
                jnt_trq_[j] = gazebo_joints_[j+nb_static_joints]->GetForce(0u);//jnt_trq_cmd_[j];
                //jw = gazebo_joints_[j+1]->GetForceTorque(0u);
                //RTT::log(RTT::Error)<<"j"<<j<<" "<<jw.body1Torque.x<<" "<<jw.body1Torque.y<<" "<<jw.body1Torque.z<<RTT::endlog(); 
                //jnt_trq_[j] = jw.body1Torque.x;
                //jnt_trq_[j] = jnt_trq_cmd_[j];
            }

            /// Set Initial Joint Positions
            if(set_new_pos && data_timestamp == new_pos_timestamp)
            {
                RTT::log(RTT::Warning) << "Setting Joint Position : \n"<<js_cmd<<RTT::endlog();
                if(gazebo_joints_.size())
                    gazebo_joints_[0]->GetAngle(0).Radian();
                for(unsigned j=0; j<n_joints_; j++)
                    gazebo_joints_[j+nb_static_joints]->SetAngle(0,jnt_pos_cmd_[j]);
                set_new_pos = false;
            }else{
                /*if(gazebo_joints_.size())
                    gazebo_joints_[0]->GetAngle(0).Radian();
                for(unsigned int j=0; j < n_joints_; j++)
                    gazebo_joints_[j+1]->SetAngle(0,gazebo_joints_[j+1]->GetAngle(0).Radian());*/
            }
            
            // Simulates breaks
            // NOTE: Gazebo is calling the callback very fast, so we might have false positive
            
            switch(data_fs)
            {
                case RTT::NoData: // Not Connected
                    set_brakes = true;
                    break;
            
                case RTT::OldData: 
                    if(data_timestamp == last_data_timestamp
                        && nb_no_data_++ >= 2) // Connection lost
                        set_brakes = true;
                    break;
            
                case RTT::NewData: //we cool
                    set_brakes = false;
                    if(nb_no_data_-- <= 0)
                        nb_no_data_ = 0;
                    break;
            }
            
            RTT::log(RTT::Debug) << "data_fs : "<<data_fs<<" ts:"<<data_timestamp<<" last ts:"<<last_data_timestamp <<" steps rtt:" <<steps_rtt_<<"last steps:" << last_steps_rtt_<<"brakes:"<<set_brakes<<RTT::endlog();
            
            if(set_brakes)
            {
                
                /*for(gazebo::physics::Link_V::iterator it = model_links_.begin();
                    it != model_links_.end();++it)
                    (*it)->SetGravityMode(false);*/

                for(gazebo::physics::Joint_V::iterator it = gazebo_joints_.begin();it != gazebo_joints_.end();++it)
                    (*it)->SetAngle(0,(*it)->GetAngle(0).Radian());
            }else{
                /*for(gazebo::physics::Link_V::iterator it = model_links_.begin();
                    it != model_links_.end();++it)
                    (*it)->SetGravityMode(true);*/

                // Write command
                if(gazebo_joints_.size())
                    gazebo_joints_[0]->SetForce(0,gazebo_joints_[0]->GetForce(0u));
                for(unsigned j=0; j<n_joints_; j++)
                    gazebo_joints_[j+nb_static_joints]->SetForce(0,jnt_trq_cmd_[j]);
            }
            last_data_timestamp=data_timestamp;
        }else{
            //RTT::log(RTT::Error)<< "gazeboUpdateHook locked" <<RTT::endlog();
        }
#ifdef XENOMAI
     gazebo_done = true;
#endif
    }


    virtual bool configureHook()
    {
        return true;
    }

    virtual void updateHook()
    {
        // Synchronize with gazeboUpdate()
#ifndef XENOMAI
       RTT::os::MutexLock lock(gazebo_mutex_);
#else
       if(true){
           rtt_done=false;
           
#endif
        if(port_JointPositionCommand.connected() || 
            port_JointTorqueCommand.connected() || 
            port_JointVelocityCommand.connected())
            using_ros_topics = false;
        
        static double last_update_time_sim;
        period_sim_ = rtt_time_ - last_update_time_sim;
        last_update_time_sim = rtt_time_;

        // Compute period in wall clock
        static double last_update_time_wall;
        period_wall_ = wall_time_ - last_update_time_wall;
        last_update_time_wall = wall_time_;


        // Increment simulation step counter (debugging)
        steps_rtt_++;

        // Get command from ports

        RTT::os::TimeService::ticks read_start = RTT::os::TimeService::Instance()->getTicks();
        
        if(using_ros_topics)
        {
            data_fs = port_JointStatesCommand.read(js_cmd);
            //if(data_fs == RTT::NoData)
            //    return;
            data_timestamp = js_cmd.header.stamp;
            if(data_fs==RTT::NewData && js_cmd.header.frame_id == "POSITION_CMD")
            {
                RTT::log(RTT::Warning) << "Joint Position Requested : \n"<<js_cmd<<RTT::endlog();
                jnt_pos_cmd_ = js_cmd.position;
                set_new_pos = true;
                new_pos_timestamp = data_timestamp;
            }else
                jnt_trq_cmd_ = js_cmd.effort;
            
        }else{
            jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
            
            if(jnt_pos_fs == RTT::NewData)
                set_new_pos = true;
            
            data_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
            
        }
        
        
        read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

        // Write state to ports
        RTT::os::TimeService::ticks write_start = RTT::os::TimeService::Instance()->getTicks();

        port_JointVelocity.write(jnt_vel_);
        port_JointPosition.write(jnt_pos_);
        port_JointTorque.write(jnt_trq_);
        
        js.header.stamp = rtt_rosclock::host_now(); // Wall time
        js.effort = jnt_trq_;
        js.position = jnt_pos_;
        js.velocity = jnt_vel_;
        
        port_JointStates.write(js);
        
        write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);
#ifdef XENOMAI
        //gazebo_mutex_.unlock();
        rtt_done=true;
       }else{
           //RTT::log(RTT::Error)<< "gazeboUpdateHook not done" <<RTT::endlog();
       }
#endif
    }
protected:

    //! Synchronization
#ifndef XENOMAI
    RTT::os::MutexRecursive gazebo_mutex_;
#endif
    //! The Gazebo Model
    //! The gazebo
    std::vector<gazebo::physics::JointPtr> gazebo_joints_;
    gazebo::physics::Link_V model_links_;
    std::vector<std::string> joint_names_;

    RTT::InputPort<std::vector<double> > port_JointVelocityCommand;
    RTT::InputPort<std::vector<double> > port_JointTorqueCommand;
    RTT::InputPort<std::vector<double> > port_JointPositionCommand;

    RTT::OutputPort<std::vector<double> > port_JointVelocity;
    RTT::OutputPort<std::vector<double> > port_JointTorque;
    RTT::OutputPort<std::vector<double> > port_JointPosition;
    
    RTT::OutputPort<sensor_msgs::JointState> port_JointStates;
    RTT::InputPort<sensor_msgs::JointState> port_JointStatesCommand;
    
    RTT::FlowStatus jnt_pos_fs,
                    data_fs;

    std::vector<double> jnt_vel_,jnt_trq_,jnt_pos_;
    std::vector<double> jnt_vel_cmd_,jnt_trq_cmd_,jnt_pos_cmd_;

    //! RTT time for debugging
    double rtt_time_;
    //! Gazebo time for debugging
    double gz_time_;
    double wall_time_;

    ros::Time last_gz_update_time_,new_pos_timestamp;
    RTT::Seconds gz_period_;
    RTT::Seconds gz_duration_;

    ros::Time last_update_time_;
    RTT::Seconds rtt_period_;
    RTT::Seconds read_duration_;
    RTT::Seconds write_duration_;

    int steps_gz_;
    int steps_rtt_,last_steps_rtt_,nb_no_data_;
    unsigned int n_joints_;
    int cnt_lock_;
    double period_sim_;
    double period_wall_;
    boost::atomic<bool> new_data,set_new_pos;
    boost::atomic<bool> rtt_done,gazebo_done;
    
    sensor_msgs::JointState js,js_cmd;
    bool using_ros_topics;
    ros::Time last_data_timestamp,data_timestamp;
    bool set_brakes;
    int nb_static_joints;
    
    
};

ORO_LIST_COMPONENT_TYPE(LWRGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
