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
      connectHw("connectHw"),
      connectSim("getAllAbsolute"),
      connectPeer("connectPeer")
    {
      this->addOperationCaller(connectHw);
      this->addOperationCaller(connectSim);
      this->addOperationCaller(connectPeer);
    }

    RTT::OperationCaller<bool(const std::string &)> connectHw;
    RTT::OperationCaller<bool(const std::string &)> connectSim;
    RTT::OperationCaller<bool(const std::string &,const std::string &)> connectPeer;

  };
}

#endif