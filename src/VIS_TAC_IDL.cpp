// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <VIS_TAC_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class VIS_TAC_IDL_move_arm_upward : public yarp::os::Portable {
public:
  std::string armToMove;
  bool _return;
  void init(const std::string& armToMove);
  virtual bool write(yarp::os::ConnectionWriter& connection) override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

bool VIS_TAC_IDL_move_arm_upward::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("move_arm_upward",1,3)) return false;
  if (!writer.writeString(armToMove)) return false;
  return true;
}

bool VIS_TAC_IDL_move_arm_upward::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void VIS_TAC_IDL_move_arm_upward::init(const std::string& armToMove) {
  _return = false;
  this->armToMove = armToMove;
}

VIS_TAC_IDL::VIS_TAC_IDL() {
  yarp().setOwner(*this);
}
bool VIS_TAC_IDL::move_arm_upward(const std::string& armToMove) {
  bool _return = false;
  VIS_TAC_IDL_move_arm_upward helper;
  helper.init(armToMove);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool VIS_TAC_IDL::move_arm_upward(const std::string& armToMove)");
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
    if (tag == "move_arm_upward") {
      std::string armToMove;
      if (!reader.readString(armToMove)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = move_arm_upward(armToMove);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
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
    helpString.push_back("move_arm_upward");
    helpString.push_back("help");
  }
  else {
    if (functionName=="move_arm_upward") {
      helpString.push_back("bool move_arm_upward(const std::string& armToMove) ");
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


