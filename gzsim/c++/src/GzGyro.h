#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzGyro : public GzNode{
  static std::string topic_base;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  public:
    GzGyro(std::shared_ptr<nt::NetworkTable> t);
    ~GzGyro();
    const std::string name() { return std::string("GzGyro");}
    bool connect();
    void reset();
    void run();
    void stop();
};