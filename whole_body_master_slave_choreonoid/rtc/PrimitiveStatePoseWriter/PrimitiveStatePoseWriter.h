#ifndef PrimitiveStatePoseWriter_H
#define PrimitiveStatePoseWriter_H

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

class PrimitiveStatePoseWriter : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveStateRefIn_("primitiveStateRefIn", m_primitiveStateRef_),
      m_qIn_("qIn", m_q_),
      m_basePoseIn_("basePoseIn", m_basePose_),
      m_primitiveStateComOut_("primitiveStateComOut", m_primitiveStateCom_)
    {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRefIn_;
    RTC::TimedDoubleSeq m_q_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
    RTC::TimedPose3D m_basePose_;
    RTC::InPort<RTC::TimedPose3D> m_basePoseIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateComOut_;

  };

public:
  PrimitiveStatePoseWriter(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
  std::mutex mutex_;

  unsigned int loop_;

  Ports ports_;

  cnoid::BodyPtr robot_;

  // portから受け取ったprimitive motion level 指令
  primitive_motion_level_tools::PrimitiveStates primitiveStates_;

  // params

  // static functions
  static void getPrimitiveState(const std::string& instance_name, PrimitiveStatePoseWriter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void getRobot(const std::string& instance_name, PrimitiveStatePoseWriter::Ports& port, cnoid::BodyPtr& robot);
  static void calcOutputPorts(const std::string& instance_name,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                              const cnoid::BodyPtr& robot,
                              double dt,
                              PrimitiveStatePoseWriter::Ports& port);
};


extern "C"
{
  void PrimitiveStatePoseWriterInit(RTC::Manager* manager);
};

#endif // PrimitiveStatePoseWriter_H
