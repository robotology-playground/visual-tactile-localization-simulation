// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_VIS_TAC_IDL
#define YARP_THRIFT_GENERATOR_VIS_TAC_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class VIS_TAC_IDL;


class VIS_TAC_IDL : public yarp::os::Wire {
public:
  VIS_TAC_IDL();
  virtual bool move_arm_upward(const std::string& armToMove);
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
