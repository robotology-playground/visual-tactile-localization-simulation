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
  virtual std::string start_visual_localization();
  virtual std::string stop_localization();
  virtual std::string reset_filter();
  virtual std::string set_min_allowed_z(const double minZ);
  virtual std::string get_min_allowed_z();
  virtual std::string get_approach_position(const std::string& whereToApproach);
  virtual std::string move_hand_upward(const std::string& armToMove);
  virtual std::string move_arm_rest_pose(const std::string& armToMove);
  virtual std::string home(const std::string& armToPutHome);
  virtual std::string approach(const std::string& armToUse, const std::string& whereToApproach);
  virtual std::string fingers_approach(const std::string& armToUse);
  virtual std::string fingers_restore(const std::string& armToUse);
  virtual std::string enable_contacts_probe(const std::string& armToUse);
  virtual std::string disable_contacts_probe(const std::string& armToUse);
  virtual std::string pull(const std::string& armToUse);
  virtual std::string rotate(const std::string& armToUse);
  virtual std::string stop();
  virtual std::string quit();
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
