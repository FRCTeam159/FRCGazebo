#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzSwitch : public GzNode{
  static std::string topic_base;
  int chnl;
  double low_limit;
  double high_limit;
  double position;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  public:
    GzSwitch(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzSwitch();
    const std::string name() { return std::string("GzSwitch"+std::to_string(chnl));}
    bool connect();
    void reset();
    void run();
    void stop();
};