#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzCamera : public GzNode{
  static std::string topic_base;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  int chnl;
  public:
    GzCamera(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzCamera();
    const std::string name() { return std::string("GzCamera");}
    bool connect();
    void reset();
    void stop();
    void run();
    void enable();
    void disable();
};