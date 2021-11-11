#ifndef PrimitiveStateRobotConverter_H
#define PrimitiveStateRobotConverter_H

#include <memory>
#include <unordered_map>
#include <mutex>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>

#include <cnoid/Body>
#include <cnoid/BasicSensors>

#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include <primitive_motion_level_tools/PrimitiveState.h>

class PrimitiveStateRobotConverter : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveStateRefIn_("primitiveStateRefIn", m_primitiveStateRef_),
      m_primitiveStateRobotIn_("primitiveStateRobotIn", m_primitiveStateRobot_),
      m_primitiveStateComOut_("primitiveStateComOut", m_primitiveStateCom_)
    {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRefIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRobot_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRobotIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateComOut_;

  };

public:
  PrimitiveStateRobotConverter(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
  std::mutex mutex_;

  unsigned int loop_;

  Ports ports_;

  // portから受け取ったprimitive motion level 指令
  primitive_motion_level_tools::PrimitiveStates primitiveStatesRef_;
  primitive_motion_level_tools::PrimitiveStates primitiveStatesRobot_;

  // params

  // static functions
  static void getPrimitiveStateRef(const std::string& instance_name, PrimitiveStateRobotConverter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void getPrimitiveStateRobot(const std::string& instance_name, PrimitiveStateRobotConverter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void calcOutputPorts(const std::string& instance_name,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStatesRef,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStatesRobot,
                              double dt,
                              PrimitiveStateRobotConverter::Ports& port);
};


extern "C"
{
  void PrimitiveStateRobotConverterInit(RTC::Manager* manager);
};

#endif // PrimitiveStateRobotConverter_H
