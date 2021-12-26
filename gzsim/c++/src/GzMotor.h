#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzMotor : public GzNode{
  static std::string topic_base;
  int chnl;
  public:
    GzMotor(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzMotor();
    const std::string name() { return std::string("GzMotor"+std::to_string(chnl));}
    bool connect();
    void publish();
};