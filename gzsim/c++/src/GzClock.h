#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzClock : public GzNode{
  static std::string topic_base;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  public:
    GzClock(std::shared_ptr<nt::NetworkTable> t);
    ~GzClock();
    const std::string name() { return std::string("GzClock");}
    bool connect();
    void reset();
    void stop();
    void run();
};