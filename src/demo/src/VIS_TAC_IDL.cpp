// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/VIS_TAC_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class VIS_TAC_IDL_init : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_start_visual_localization : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_stop_localization : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_reset_filter : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_set_min_allowed_z : public yarp::os::Portable {
public:
  double minZ;
  std::string _return;
  void init(const double minZ);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_get_min_allowed_z : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_get_approach_position : public yarp::os::Portable {
public:
  std::string whereToApproach;
  std::string _return;
  void init(const std::string& whereToApproach);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_move_hand_upward : public yarp::os::Portable {
public:
  std::string armToMove;
  std::string _return;
  void init(const std::string& armToMove);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_move_arm_rest_pose : public yarp::os::Portable {
public:
  std::string armToMove;
  std::string _return;
  void init(const std::string& armToMove);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_home : public yarp::os::Portable {
public:
  std::string armToPutHome;
  std::string _return;
  void init(const std::string& armToPutHome);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_approach : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string whereToApproach;
  std::string _return;
  void init(const std::string& armToUse, const std::string& whereToApproach);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_fingers_approach : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string _return;
  void init(const std::string& armToUse);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_fingers_restore : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string _return;
  void init(const std::string& armToUse);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_enable_contacts_probe : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string _return;
  void init(const std::string& armToUse);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_disable_contacts_probe : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string _return;
  void init(const std::string& armToUse);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_pull : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string _return;
  void init(const std::string& armToUse);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_rotate : public yarp::os::Portable {
public:
  std::string armToUse;
  std::string _return;
  void init(const std::string& armToUse);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_stop : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class VIS_TAC_IDL_quit : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

bool VIS_TAC_IDL_init::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("init",1,1)) return false;
  return true;
}

bool VIS_TAC_IDL_init::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_init::init() {
  _return = "";
}

bool VIS_TAC_IDL_start_visual_localization::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("start_visual_localization",1,3)) return false;
  return true;
}

bool VIS_TAC_IDL_start_visual_localization::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_start_visual_localization::init() {
  _return = "";
}

bool VIS_TAC_IDL_stop_localization::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("stop_localization",1,2)) return false;
  return true;
}

bool VIS_TAC_IDL_stop_localization::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_stop_localization::init() {
  _return = "";
}

bool VIS_TAC_IDL_reset_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("reset_filter",1,2)) return false;
  return true;
}

bool VIS_TAC_IDL_reset_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_reset_filter::init() {
  _return = "";
}

bool VIS_TAC_IDL_set_min_allowed_z::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("set_min_allowed_z",1,4)) return false;
  if (!writer.writeDouble(minZ)) return false;
  return true;
}

bool VIS_TAC_IDL_set_min_allowed_z::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_set_min_allowed_z::init(const double minZ) {
  _return = "";
  this->minZ = minZ;
}

bool VIS_TAC_IDL_get_min_allowed_z::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("get_min_allowed_z",1,4)) return false;
  return true;
}

bool VIS_TAC_IDL_get_min_allowed_z::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_get_min_allowed_z::init() {
  _return = "";
}

bool VIS_TAC_IDL_get_approach_position::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("get_approach_position",1,3)) return false;
  if (!writer.writeString(whereToApproach)) return false;
  return true;
}

bool VIS_TAC_IDL_get_approach_position::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_get_approach_position::init(const std::string& whereToApproach) {
  _return = "";
  this->whereToApproach = whereToApproach;
}

bool VIS_TAC_IDL_move_hand_upward::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("move_hand_upward",1,3)) return false;
  if (!writer.writeString(armToMove)) return false;
  return true;
}

bool VIS_TAC_IDL_move_hand_upward::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_move_hand_upward::init(const std::string& armToMove) {
  _return = "";
  this->armToMove = armToMove;
}

bool VIS_TAC_IDL_move_arm_rest_pose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("move_arm_rest_pose",1,4)) return false;
  if (!writer.writeString(armToMove)) return false;
  return true;
}

bool VIS_TAC_IDL_move_arm_rest_pose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_move_arm_rest_pose::init(const std::string& armToMove) {
  _return = "";
  this->armToMove = armToMove;
}

bool VIS_TAC_IDL_home::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("home",1,1)) return false;
  if (!writer.writeString(armToPutHome)) return false;
  return true;
}

bool VIS_TAC_IDL_home::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_home::init(const std::string& armToPutHome) {
  _return = "";
  this->armToPutHome = armToPutHome;
}

bool VIS_TAC_IDL_approach::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("approach",1,1)) return false;
  if (!writer.writeString(armToUse)) return false;
  if (!writer.writeString(whereToApproach)) return false;
  return true;
}

bool VIS_TAC_IDL_approach::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_approach::init(const std::string& armToUse, const std::string& whereToApproach) {
  _return = "";
  this->armToUse = armToUse;
  this->whereToApproach = whereToApproach;
}

bool VIS_TAC_IDL_fingers_approach::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("fingers_approach",1,2)) return false;
  if (!writer.writeString(armToUse)) return false;
  return true;
}

bool VIS_TAC_IDL_fingers_approach::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_fingers_approach::init(const std::string& armToUse) {
  _return = "";
  this->armToUse = armToUse;
}

bool VIS_TAC_IDL_fingers_restore::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("fingers_restore",1,2)) return false;
  if (!writer.writeString(armToUse)) return false;
  return true;
}

bool VIS_TAC_IDL_fingers_restore::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_fingers_restore::init(const std::string& armToUse) {
  _return = "";
  this->armToUse = armToUse;
}

bool VIS_TAC_IDL_enable_contacts_probe::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("enable_contacts_probe",1,3)) return false;
  if (!writer.writeString(armToUse)) return false;
  return true;
}

bool VIS_TAC_IDL_enable_contacts_probe::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_enable_contacts_probe::init(const std::string& armToUse) {
  _return = "";
  this->armToUse = armToUse;
}

bool VIS_TAC_IDL_disable_contacts_probe::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("disable_contacts_probe",1,3)) return false;
  if (!writer.writeString(armToUse)) return false;
  return true;
}

bool VIS_TAC_IDL_disable_contacts_probe::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_disable_contacts_probe::init(const std::string& armToUse) {
  _return = "";
  this->armToUse = armToUse;
}

bool VIS_TAC_IDL_pull::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("pull",1,1)) return false;
  if (!writer.writeString(armToUse)) return false;
  return true;
}

bool VIS_TAC_IDL_pull::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_pull::init(const std::string& armToUse) {
  _return = "";
  this->armToUse = armToUse;
}

bool VIS_TAC_IDL_rotate::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("rotate",1,1)) return false;
  if (!writer.writeString(armToUse)) return false;
  return true;
}

bool VIS_TAC_IDL_rotate::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_rotate::init(const std::string& armToUse) {
  _return = "";
  this->armToUse = armToUse;
}

bool VIS_TAC_IDL_stop::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("stop",1,1)) return false;
  return true;
}

bool VIS_TAC_IDL_stop::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_stop::init() {
  _return = "";
}

bool VIS_TAC_IDL_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool VIS_TAC_IDL_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_quit::init() {
  _return = "";
}

VIS_TAC_IDL::VIS_TAC_IDL() {
  yarp().setOwner(*this);
}
std::string VIS_TAC_IDL::init() {
  std::string _return = "";
  VIS_TAC_IDL_init helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::init()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::start_visual_localization() {
  std::string _return = "";
  VIS_TAC_IDL_start_visual_localization helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::start_visual_localization()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::stop_localization() {
  std::string _return = "";
  VIS_TAC_IDL_stop_localization helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::stop_localization()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::reset_filter() {
  std::string _return = "";
  VIS_TAC_IDL_reset_filter helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::reset_filter()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::set_min_allowed_z(const double minZ) {
  std::string _return = "";
  VIS_TAC_IDL_set_min_allowed_z helper;
  helper.init(minZ);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::set_min_allowed_z(const double minZ)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::get_min_allowed_z() {
  std::string _return = "";
  VIS_TAC_IDL_get_min_allowed_z helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::get_min_allowed_z()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::get_approach_position(const std::string& whereToApproach) {
  std::string _return = "";
  VIS_TAC_IDL_get_approach_position helper;
  helper.init(whereToApproach);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::get_approach_position(const std::string& whereToApproach)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::move_hand_upward(const std::string& armToMove) {
  std::string _return = "";
  VIS_TAC_IDL_move_hand_upward helper;
  helper.init(armToMove);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::move_hand_upward(const std::string& armToMove)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::move_arm_rest_pose(const std::string& armToMove) {
  std::string _return = "";
  VIS_TAC_IDL_move_arm_rest_pose helper;
  helper.init(armToMove);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::move_arm_rest_pose(const std::string& armToMove)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::home(const std::string& armToPutHome) {
  std::string _return = "";
  VIS_TAC_IDL_home helper;
  helper.init(armToPutHome);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::home(const std::string& armToPutHome)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::approach(const std::string& armToUse, const std::string& whereToApproach) {
  std::string _return = "";
  VIS_TAC_IDL_approach helper;
  helper.init(armToUse,whereToApproach);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::approach(const std::string& armToUse, const std::string& whereToApproach)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::fingers_approach(const std::string& armToUse) {
  std::string _return = "";
  VIS_TAC_IDL_fingers_approach helper;
  helper.init(armToUse);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::fingers_approach(const std::string& armToUse)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::fingers_restore(const std::string& armToUse) {
  std::string _return = "";
  VIS_TAC_IDL_fingers_restore helper;
  helper.init(armToUse);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::fingers_restore(const std::string& armToUse)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::enable_contacts_probe(const std::string& armToUse) {
  std::string _return = "";
  VIS_TAC_IDL_enable_contacts_probe helper;
  helper.init(armToUse);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::enable_contacts_probe(const std::string& armToUse)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::disable_contacts_probe(const std::string& armToUse) {
  std::string _return = "";
  VIS_TAC_IDL_disable_contacts_probe helper;
  helper.init(armToUse);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::disable_contacts_probe(const std::string& armToUse)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::pull(const std::string& armToUse) {
  std::string _return = "";
  VIS_TAC_IDL_pull helper;
  helper.init(armToUse);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::pull(const std::string& armToUse)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::rotate(const std::string& armToUse) {
  std::string _return = "";
  VIS_TAC_IDL_rotate helper;
  helper.init(armToUse);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::rotate(const std::string& armToUse)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::stop() {
  std::string _return = "";
  VIS_TAC_IDL_stop helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::stop()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string VIS_TAC_IDL::quit() {
  std::string _return = "";
  VIS_TAC_IDL_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string VIS_TAC_IDL::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool VIS_TAC_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "init") {
      std::string _return;
      _return = init();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "start_visual_localization") {
      std::string _return;
      _return = start_visual_localization();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stop_localization") {
      std::string _return;
      _return = stop_localization();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "reset_filter") {
      std::string _return;
      _return = reset_filter();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_min_allowed_z") {
      double minZ;
      if (!reader.readDouble(minZ)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = set_min_allowed_z(minZ);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_min_allowed_z") {
      std::string _return;
      _return = get_min_allowed_z();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_approach_position") {
      std::string whereToApproach;
      if (!reader.readString(whereToApproach)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = get_approach_position(whereToApproach);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "move_hand_upward") {
      std::string armToMove;
      if (!reader.readString(armToMove)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = move_hand_upward(armToMove);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "move_arm_rest_pose") {
      std::string armToMove;
      if (!reader.readString(armToMove)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = move_arm_rest_pose(armToMove);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "home") {
      std::string armToPutHome;
      if (!reader.readString(armToPutHome)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = home(armToPutHome);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "approach") {
      std::string armToUse;
      std::string whereToApproach;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(whereToApproach)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = approach(armToUse,whereToApproach);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "fingers_approach") {
      std::string armToUse;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = fingers_approach(armToUse);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "fingers_restore") {
      std::string armToUse;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = fingers_restore(armToUse);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "enable_contacts_probe") {
      std::string armToUse;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = enable_contacts_probe(armToUse);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "disable_contacts_probe") {
      std::string armToUse;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = disable_contacts_probe(armToUse);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "pull") {
      std::string armToUse;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = pull(armToUse);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "rotate") {
      std::string armToUse;
      if (!reader.readString(armToUse)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = rotate(armToUse);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stop") {
      std::string _return;
      _return = stop();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      std::string _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> VIS_TAC_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("init");
    helpString.push_back("start_visual_localization");
    helpString.push_back("stop_localization");
    helpString.push_back("reset_filter");
    helpString.push_back("set_min_allowed_z");
    helpString.push_back("get_min_allowed_z");
    helpString.push_back("get_approach_position");
    helpString.push_back("move_hand_upward");
    helpString.push_back("move_arm_rest_pose");
    helpString.push_back("home");
    helpString.push_back("approach");
    helpString.push_back("fingers_approach");
    helpString.push_back("fingers_restore");
    helpString.push_back("enable_contacts_probe");
    helpString.push_back("disable_contacts_probe");
    helpString.push_back("pull");
    helpString.push_back("rotate");
    helpString.push_back("stop");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="init") {
      helpString.push_back("std::string init() ");
    }
    if (functionName=="start_visual_localization") {
      helpString.push_back("std::string start_visual_localization() ");
    }
    if (functionName=="stop_localization") {
      helpString.push_back("std::string stop_localization() ");
    }
    if (functionName=="reset_filter") {
      helpString.push_back("std::string reset_filter() ");
    }
    if (functionName=="set_min_allowed_z") {
      helpString.push_back("std::string set_min_allowed_z(const double minZ) ");
    }
    if (functionName=="get_min_allowed_z") {
      helpString.push_back("std::string get_min_allowed_z() ");
    }
    if (functionName=="get_approach_position") {
      helpString.push_back("std::string get_approach_position(const std::string& whereToApproach) ");
    }
    if (functionName=="move_hand_upward") {
      helpString.push_back("std::string move_hand_upward(const std::string& armToMove) ");
    }
    if (functionName=="move_arm_rest_pose") {
      helpString.push_back("std::string move_arm_rest_pose(const std::string& armToMove) ");
    }
    if (functionName=="home") {
      helpString.push_back("std::string home(const std::string& armToPutHome) ");
    }
    if (functionName=="approach") {
      helpString.push_back("std::string approach(const std::string& armToUse, const std::string& whereToApproach) ");
    }
    if (functionName=="fingers_approach") {
      helpString.push_back("std::string fingers_approach(const std::string& armToUse) ");
    }
    if (functionName=="fingers_restore") {
      helpString.push_back("std::string fingers_restore(const std::string& armToUse) ");
    }
    if (functionName=="enable_contacts_probe") {
      helpString.push_back("std::string enable_contacts_probe(const std::string& armToUse) ");
    }
    if (functionName=="disable_contacts_probe") {
      helpString.push_back("std::string disable_contacts_probe(const std::string& armToUse) ");
    }
    if (functionName=="pull") {
      helpString.push_back("std::string pull(const std::string& armToUse) ");
    }
    if (functionName=="rotate") {
      helpString.push_back("std::string rotate(const std::string& armToUse) ");
    }
    if (functionName=="stop") {
      helpString.push_back("std::string stop() ");
    }
    if (functionName=="quit") {
      helpString.push_back("std::string quit() ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


