#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzGyro : public GzNode{
  static std::string topic_base;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  int chnl;
  public:
    GzGyro(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzGyro();
    const std::string name() { return std::string("GzGyro"+std::to_string(chnl));}
    bool connect();
    void reset();
    void run();
    void stop();
};