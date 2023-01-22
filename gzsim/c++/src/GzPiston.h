#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzPiston : public GzNode{
  static std::string topic_base;
  int chnl;
  public:
    GzPiston(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzPiston();
    const std::string name() { return std::string("GzPiston"+std::to_string(chnl));}
    bool connect();
    void publish();
};