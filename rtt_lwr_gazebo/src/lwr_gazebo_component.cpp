#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

//#include <Eigen/Dense>
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
        rtt_done(true),
        gazebo_done(false),
        new_data(true),
        cnt_lock_(100),
        last_steps_rtt_(0),
        set_new_pos(false),
        nb_no_data_(0),
        using_ros_topics(true),
        set_brakes(false),
        nb_cmd_received_(0),
        new_cmd_sem_(0),
        new_gz_sem_(0),
        sync_with_cmds_(true),
        last_gz_update_time_(0,0)
    {
        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&LWRGazeboComponent::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&LWRGazeboComponent::gazeboUpdateHook,this,RTT::ClientThread);
        this->addOperation("setLinkGravityMode",&LWRGazeboComponent::setLinkGravityMode,this,RTT::ClientThread);
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
        this->addProperty("sync_with_cmds",sync_with_cmds_);

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
    }
    // Let everyone know that rtt_gazebo is loaded
    bool readyService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res)
    {
        return true;
    }

    void setLinkGravityMode(const std::string& link_name,bool gravity_mode)
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

        RTT::log(RTT::Warning)<<"Model has "<<gazebo_joints_.size()<<" joints"<<RTT::endlog();
        RTT::log(RTT::Warning)<<"Model has "<<model_links_.size()<<" links"<<RTT::endlog();

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
            joints_idx.push_back(idx);
            joint_names_.push_back(name);
            RTT::log(RTT::Warning)<<"Adding joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
        }

        if(joints_idx.size() == 0)
        {
            RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
            return false;
        }

        RTT::log(RTT::Warning)<<"Gazebo model found "<<joints_idx.size()<<" joints "<<RTT::endlog();

        for(unsigned j=0; j < joints_idx.size(); j++){
            jnt_pos_cmd_.push_back(0.0);
            jnt_vel_cmd_.push_back(0.0);
            jnt_trq_cmd_.push_back(0.0);
            jnt_pos_.push_back(0.0);
            jnt_vel_.push_back(0.0);
            jnt_trq_.push_back(0.0);
            jnt_pos_brakes_.push_back(0.0);
        }

        js.effort = jnt_trq_;
        js.position = jnt_pos_;
        js.velocity = jnt_vel_;

        js.effort = jnt_trq_cmd_;
        js.position = jnt_pos_cmd_;
        js.velocity = jnt_vel_cmd_;

        js.name = joint_names_;
        js_cmd.name = joint_names_;

        RTT::log(RTT::Warning)<<"Done configuring gazebo"<<RTT::endlog();
        return true;
    }

    //! Called from Gazebo
    virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model)
    {
        if(model.get() == NULL) {return;}

        // Get new data via busy wait
        static ros::Time last_clock,now;

        int n_trials = 1000;
        do{
            updateData();

            if((nb_cmd_received_==0 && data_fs==RTT::NoData) ||  jnt_pos_fs == RTT::NewData)
                break;

            now = rtt_rosclock::host_now();

            if(now != last_clock && data_fs!=RTT::NewData)
                RTT::log(RTT::Debug) <<  getName() << " " <<"Waiting for UpdateHook at "<<rtt_rosclock::host_now()<<" v:"<<nb_cmd_received_<< data_fs<<RTT::endlog();
            last_clock = now;


            // Last chance
            if(n_trials-- <= 0)
            {
                RTT::log(RTT::Error) <<  getName() << " " <<"No cmd received, settings sync_with_cmds to false "<<data_fs<<RTT::endlog();
                sync_with_cmds_ = false;
                break;
            }
            // Sleep to avoid very busy wait
            TIME_SPEC sleep_time;
            sleep_time.tv_sec = 0;
            sleep_time.tv_nsec = 5E5; // 500us;
            rtos_nanosleep(&sleep_time,NULL);

        }while(!(RTT::NewData == data_fs && nb_cmd_received_) && sync_with_cmds_);

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
        for(unsigned j=0; j<joints_idx.size(); j++) {
            jnt_pos_[j] = gazebo_joints_[joints_idx[j]]->GetAngle(0).Radian();
            jnt_vel_[j] = gazebo_joints_[joints_idx[j]]->GetVelocity(0);
            jnt_trq_[j] = gazebo_joints_[joints_idx[j]]->GetForce(0u);
        }

        // Simulates breaks
        // NOTE: Gazebo is calling the callback very fast, so we might have false positive
        // This is only usefull when using ROS interface (not CORBA) + launching gazebo standalone
        // This allows the controller (launched with launch_gazebo:=false) to be restarted online

        switch(data_fs)
        {
            // Not Connected
            case RTT::NoData:
                set_brakes = true;
                break;

            // Connection lost
            case RTT::OldData:
                if(data_timestamp == last_data_timestamp
                    && nb_no_data_++ >= 2)
                    set_brakes = true;
                break;

            // OK
            case RTT::NewData:
                set_brakes = false;
                if(nb_no_data_-- <= 0)
                    nb_no_data_ = 0;
                break;
        }

        // Set Gravity Mode or specified links
        for(std::map<gazebo::physics::LinkPtr,bool>::iterator it = this->gravity_mode_.begin();
            it != this->gravity_mode_.end();++it)
            {
                   it->first->SetGravityMode(it->second);
            }

        //RTT::log(RTT::Debug) << "Gazebo data_fs : "<<data_fs<<" ts:"<<data_timestamp<<" last ts:"<<last_data_timestamp <<" steps rtt:" <<steps_rtt_<<"last steps:" << last_steps_rtt_<<"brakes:"<<set_brakes<<RTT::endlog();

        // Copy Current joint pos in case of brakes
        if(!set_brakes)
            for(unsigned j=0; j<joints_idx.size(); j++)
                jnt_pos_brakes_[j] = jnt_pos_[j];

        // Force Joint Positions in case of a cmd
        if(set_new_pos)
        {
            RTT::log(RTT::Warning) <<  getName() << " " <<jnt_pos_fs<< " Setting Joint Position : \n"<<js_cmd<<RTT::endlog();

            // Update specific joints regarding cmd
            for(unsigned j=0; j<joints_idx.size(); j++)
            {
                gazebo_joints_[joints_idx[j]]->SetAngle(0,jnt_pos_cmd_[j]);
                jnt_pos_brakes_[j] = jnt_pos_cmd_[j];
            }

            // Aknowledge the settings
            set_new_pos = false;

        }else if(set_brakes)
        {
            for(unsigned j=0; j<joints_idx.size(); j++)
                gazebo_joints_[joints_idx[j]]->SetAngle(0,jnt_pos_brakes_[j]);

        }else{

            // Write command
            // Update specific joints regarding cmd
            for(unsigned j=0; j<joints_idx.size(); j++)
                gazebo_joints_[joints_idx[j]]->SetForce(0,jnt_trq_cmd_[j]);
        }
        last_data_timestamp=data_timestamp;
    }

    virtual bool configureHook()
    {
        return true;
    }

    void updateData()
    {
        if(port_JointPositionCommand.connected() ||
           port_JointTorqueCommand.connected() ||
           port_JointVelocityCommand.connected())
        {
            using_ros_topics = false;
        }

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
            data_timestamp = js_cmd.header.stamp;

            // Checking if new joint position is requested
            if(data_fs!= RTT::NoData && js_cmd.header.frame_id == "POSITION_CMD")
            {
                RTT::log(RTT::Warning) << getName() <<" "<< data_fs <<" Joint Position Requested : \n"<<js_cmd<<RTT::endlog();
                jnt_pos_cmd_ = js_cmd.position;
                set_new_pos = true;
                jnt_pos_fs = RTT::NewData;
                new_pos_timestamp = data_timestamp;
            }else{
                jnt_pos_fs = RTT::NoData;
                jnt_trq_cmd_ = js_cmd.effort;
            }

        }else
        {
            data_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
            jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);

            if(jnt_pos_fs == RTT::NewData)
            {
                set_new_pos = true;
                js_cmd.position = jnt_pos_cmd_;
                RTT::log(RTT::Warning) <<getName() <<" "<< jnt_pos_fs <<" Joint Position Requested : \n"<<js_cmd<<RTT::endlog();
            }

            data_timestamp = new_pos_timestamp = rtt_rosclock::host_now();
        }

        read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

        // Write state to ports
        RTT::os::TimeService::ticks write_start = RTT::os::TimeService::Instance()->getTicks();

        if(!using_ros_topics)
        {
            port_JointVelocity.write(jnt_vel_);
            port_JointPosition.write(jnt_pos_);
            port_JointTorque.write(jnt_trq_);
        }

        js.header.stamp = rtt_rosclock::host_now(); // Wall time
        js.effort = jnt_trq_;
        js.position = jnt_pos_;
        js.velocity = jnt_vel_;

        port_JointStates.write(js);
        write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);

        //RTT::log(RTT::Debug) << "updateData() "<<data_fs<<RTT::endlog();
        switch(data_fs){
            case RTT::OldData:
                //RTT::log(RTT::Debug) << data_fs<<" at "<<data_timestamp<<" old "<<last_timestamp<<RTT::endlog();
                break;
            case RTT::NewData:
                RTT::log(RTT::Debug) << getName() << " " <<data_fs<<" at "<<data_timestamp<<RTT::endlog();
                nb_cmd_received_++;
                last_timestamp = data_timestamp;
                break;
            case RTT::NoData:
                nb_cmd_received_= 0;
                break;
        }
    }

    virtual void updateHook()
    {
        RTT::log(RTT::Debug) << getName() << " UpdateHook() "<<rtt_rosclock::host_now()<<RTT::endlog();
        return;
    }

protected:
    std::vector<int> joints_idx;
    std::map<gazebo::physics::LinkPtr,bool> gravity_mode_;
    RTT::os::Semaphore new_cmd_sem_;
    RTT::os::Semaphore new_gz_sem_;
    //! The Gazebo Model
    //! The gazebo
    gazebo::physics::Joint_V gazebo_joints_;
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

    std::vector<double> jnt_vel_,jnt_trq_,jnt_pos_,jnt_pos_brakes_;
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
    int cnt_lock_;
    double period_sim_;
    double period_wall_;
    boost::atomic<bool> new_data,set_new_pos;
    boost::atomic<bool> rtt_done,gazebo_done;

    sensor_msgs::JointState js,js_cmd;
    bool using_ros_topics;
    ros::Time last_data_timestamp,data_timestamp,last_timestamp;
    bool set_brakes;
    int nb_cmd_received_;
    bool sync_with_cmds_;


};

ORO_LIST_COMPONENT_TYPE(LWRGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
