#ifndef __LWR_HELPER_H__
#define __LWR_HELPER_H__

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace rtt_rosparam {

  class ROSParam : public RTT::ServiceRequester
  {

  public:
    ROSParam(RTT::TaskContext *owner) :
      RTT::ServiceRequester("lwr_helper",owner),
      /*connectHw("connectHw"),
      connectSim("getAllAbsolute"),*/
      connectPeers("connectPeers"),
      connectPeers("waitForROSService")
    {
      /*this->addOperationCaller(connectHw);
      this->addOperationCaller(connectSim);*/
      this->addOperationCaller(connectPeers);
      this->addOperationCaller(waitForROSService);
    }

    /*RTT::OperationCaller<bool(const std::string &)> connectHw;
    RTT::OperationCaller<bool(const std::string &)> connectSim;*/
    RTT::OperationCaller<bool(const std::string &,const std::string &)> connectPeers;
    RTT::OperationCaller<bool(const std::string &,double)> waitForROSService;

  };
}

#endif