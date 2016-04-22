#include <lwr_utils/ros_helper.h>


using namespace RTT;
using namespace std;


ROSHelper::ROSHelper(RTT::TaskContext *owner) :
RTT::ServiceRequester("ros_helper",owner),
getParam("getParam"),
getThisNodeName("getThisNodeName"),
isSim("isSim"),
getThisNodeNamespace("getThisNodeNamespace"),
waitForROSService("waitForROSService"),
getRobotName("getRobotName"),
getRobotNs("getRobotNs"),
getServiceOwner("getServiceOwner"),
getTaskContext("getTaskContext"),
getOwner("getOwner"),
parseArgv("parseArgv"),
getTfPrefix("getTfPrefix"),
getEnvName("getEnvName"),
addComponent("addComponent"),
rosServiceCall("rosServiceCall"),
printArgv("printArgv")
{
    this->addOperationCaller(getParam);
    this->addOperationCaller(waitForROSService);
    this->addOperationCaller(getThisNodeName);
    this->addOperationCaller(getThisNodeNamespace);
    this->addOperationCaller(isSim);
    this->addOperationCaller(getRobotName);
    this->addOperationCaller(getServiceOwner);
    this->addOperationCaller(getTaskContext);
    this->addOperationCaller(getOwner);
    this->addOperationCaller(parseArgv);
    this->addOperationCaller(getRobotNs);
    this->addOperationCaller(getTfPrefix);
    this->addOperationCaller(getEnvName);
    this->addOperationCaller(addComponent);
    this->addOperationCaller(rosServiceCall);
    this->addOperationCaller(printArgv);
}

class RosHelperService: public RTT::Service
{
public:
    typedef boost::shared_ptr<RosHelperService> shared_ptr;

    RosHelperService(TaskContext* owner) :
    Service("ros_helper", owner),
    owner_(owner),
    has_parsed_arguments_(false)
    {
        this->addOperation("waitForROSService", &RosHelperService::waitForROSService, this);
        this->addOperation("getParam", &RosHelperService::getParam, this);
        this->addOperation("getRobotName", &RosHelperService::getRobotName, this);
        this->addOperation("getRobotNs", &RosHelperService::getRobotNs, this);
        this->addOperation("getTfPrefix", &RosHelperService::getTfPrefix, this);
        this->addOperation("getEnvName", &RosHelperService::getEnvName, this);
        this->addOperation("isSim", &RosHelperService::isSim, this);
        this->addOperation("getThisNodeName", &RosHelperService::getThisNodeName, this);
        this->addOperation("getThisNodeNamespace", &RosHelperService::getThisNodeNamespace, this);
        this->addOperation("getTaskContext", &RosHelperService::getTaskContext, this);
        this->addOperation("getOwner", &RosHelperService::getOwner, this);
        this->addOperation("parseArgv", &RosHelperService::parseArgv, this);
        this->addOperation("addComponent", &RosHelperService::addComponent, this);
        this->addOperation("rosServiceCall", &RosHelperService::rosServiceCall, this);
        this->addOperation("printArgv", &RosHelperService::printArgv, this);

        argv_ = owner->getProvider<OS>("os")->argv();
        this->parseArgv(argv_);
    }
    void printArgv()
    {
        if(argv_.size() == 0)
        {
            RTT::log(Warning) << "No args passed to the deployer" << endlog();
            return ;
        }
        std::cout << "Arguments passed to the deployer : "<<std::endl;
        for(int i=0;i<argv_.size();++i)
            std::cout <<argv_[i]<<std::endl;
    }

    bool parseArgv(const std::vector<std::string>& argv)
    {
        for(int i=0;i<argv.size();++i)
        {
            size_t pos = argv[i].find("=");
            if(pos != std::string::npos && pos != 0)
            {
                const std::string key = argv[i].substr(0,pos);
                const std::string value = argv[i].substr(pos + 1);
                arg_map_[key] = value;
            }
        }
        try
        {
            robot_name_ = arg_map_["robot_name"];
            is_sim_ = (arg_map_["is_sim"] == "true" ? true:false);
            robot_ns_ = arg_map_["robot_ns"];
            tf_prefix_ = arg_map_["tf_prefix"];
            if(robot_name_.empty()
            || robot_ns_.empty()
            || tf_prefix_.empty())
                throw std::out_of_range("Empty arguments");
            has_parsed_arguments_ = true;
        }
        catch(const std::out_of_range& e)
        {
            RTT::log(Warning) << "Wrong argv arguments, please provide arguments "
            "to the deployer (--) : robot_name=$(arg robot_name) "
            "is_sim=$(arg sim) robot_ns=$(arg robot_ns) "
            "tf_prefix=$(arg tf_prefix)\n\n\t "
            "Example : deployer-corba -- robot_name=lwr_sim is_sim=true "
            "robot_ns=/ tf_prefix=/" << RTT::endlog();
            has_parsed_arguments_ = false;
            return false;
        }
        RTT::log(Info) << "Parser returns : \n- robot_name = "<<robot_name_
        <<"\n- sim = "<<is_sim_
        <<"\n- robot_ns = "<<robot_ns_
        <<"\n- tf_prefix = "<<tf_prefix_
        //<<"\n- env_name = "<<env_name_
        <<endlog();
        return true;
    }
    TaskContext * getOwner(){
        return owner_;
    }
    bool addComponent(TaskContext * c)
    {
        if(!owner_) return false;
        return owner_->addPeer(c,c->getName());
    }
    TaskContext * getTaskContext(const std::string& task_name)
    {
        if(!owner_) return NULL;
        if(task_name == owner_->getName())
            return owner_;
        if(owner_->hasPeer(task_name))
            return owner_->getPeer(task_name);
        else{
            log(Error) << task_name <<" does not exists"<<endlog();
            return NULL;
        }
    }
    string getRobotName()
    {
        std::string ns = getRobotNs();

        if(*ns.rbegin() != '/') ns +='/';

        if(ros::param::has(ns+"robot_name"))
            return getParam(ns+"robot_name");

        if(has_parsed_arguments_ )
            return robot_name_;
    }
    string getThisNodeName()
    {
        return ros::this_node::getName();
    }
    bool isSim()
    {
        if(!has_parsed_arguments_)
        {
            std::cout << "Getting Param : is_sim";
            ros::param::get("is_sim",is_sim_);
            std::cout << " -> ["<<is_sim_<<"]"<<std::endl;
        }
        return is_sim_;
    }
    std::string getRobotNs()
    {
        if(has_parsed_arguments_)
            return robot_ns_;
        else
            return getParam("robot_ns");
    }
    std::string getTfPrefix()
    {
        if(has_parsed_arguments_)
            return tf_prefix_;
        else
            return getParam("tf_prefix");
    }
    std::string getThisNodeNamespace()
    {
        std::string ns(ros::this_node::getNamespace());
        size_t pos = ns.find( "//" );
        if ( pos != std::string::npos ) {
           ns.replace( pos, 2, "/" );   // 5 = length( $name )
        }
        return ns;
    }
    std::string getParam(const std::string& res_param_name)
    {
        std::string param;
        std::cout << "Getting Param : "<<res_param_name;
        ros::param::get(res_param_name, param);
        std::cout << " -> ["<<param<<"]"<<std::endl;
        return param;
    }
    bool waitForROSService(std::string service_name, double service_timeout_s)
    {
        return ros::service::waitForService(service_name, service_timeout_s*1E3);
    }
    bool rosServiceCall(const std::string& call_args)
    {
        std::string rosservice_func("rosservice call ");
        if(!system((rosservice_func+call_args).c_str()))
            return true;
        return false;
    }
    std::string getEnvName()
    {
        if(has_parsed_arguments_)
            return env_name_;
        else
            return getParam("env_name");
    }
    bool connectPeerCORBA(const std::string& interface_name,const std::string& peer_name)
    {
        if(getOwner() == NULL) return false;

        if(peer_name.empty() || interface_name.empty()) return false;

        if(this->getOwner()->hasPeer(peer_name) == true) return false;

        if(this->getOwner()->hasPeer(interface_name) == false) return false;

        if(this->getOwner()->getPeer(interface_name)->hasPeer(peer_name) == false) return false;

        return this->getOwner()->connectPeers(this->getOwner()->getPeer(interface_name)->getPeer(peer_name));
    }
    RTT::TaskContext * loadRobot()
    {

    }
    bool hasParsedArgv(){return has_parsed_arguments_;}

    virtual ~RosHelperService(){};
private:
    RTT::TaskContext * owner_;
    std::map<std::string,std::string> arg_map_;
    bool has_parsed_arguments_;
    std::string robot_name_,tf_prefix_,robot_ns_,env_name_;
    bool is_sim_;
    std::vector<std::string> argv_;
    boost::shared_ptr<Scripting> scripting_;

};
ORO_SERVICE_NAMED_PLUGIN(RosHelperService,"ros_helper")
