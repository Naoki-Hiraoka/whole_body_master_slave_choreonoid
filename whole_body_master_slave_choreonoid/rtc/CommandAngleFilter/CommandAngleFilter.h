#ifndef CommandAngleFilter_H
#define CommandAngleFilter_H

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

#include <cpp_filters/TwoPointInterpolator.h>
#include <joint_limit_table/JointLimitTable.h>

#include "CommandAngleFilterService_impl.h"

class CommandAngleFilter : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_qIn_("qIn", m_q_),
      m_qOut_("qOut", m_qCom_),
      m_basePoseIn_("basePoseIn", m_basePose_),
      m_basePoseOut_("basePoseOut", m_basePoseCom_),

      m_CommandAngleFilterServicePort_("CommandAngleFilterService") {
    }

    RTC::TimedDoubleSeq m_q_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
    RTC::TimedPose3D m_basePose_;
    RTC::InPort<RTC::TimedPose3D> m_basePoseIn_;
    RTC::TimedDoubleSeq m_qCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
    RTC::TimedPose3D m_basePoseCom_;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseOut_;

    CommandAngleFilterService_impl m_service0_;
    RTC::CorbaPort m_CommandAngleFilterServicePort_;
  };

  class ControlMode{
  public:
    enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_CONTROL, MODE_CONTROL, MODE_SYNC_TO_IDLE};
  private:
    mode_enum current, previous, next;
  public:
    ControlMode(){ current = previous = next = MODE_IDLE;}
    ~ControlMode(){}
    bool setNextMode(const mode_enum _request){
      switch(_request){
      case MODE_SYNC_TO_CONTROL:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_CONTROL; return true; }else{ return false; }
      case MODE_CONTROL:
        if(current == MODE_SYNC_TO_CONTROL){ next = MODE_CONTROL; return true; }else{ return false; }
      case MODE_SYNC_TO_IDLE:
        if(current == MODE_CONTROL){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      case MODE_IDLE:
        if(current == MODE_SYNC_TO_IDLE ){ next = MODE_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(){ previous = current; current = next; }
    mode_enum now(){ return current; }
    mode_enum pre(){ return previous; }
    bool isRunning(){ return (current==MODE_SYNC_TO_CONTROL) || (current==MODE_CONTROL) || (current==MODE_SYNC_TO_IDLE) ;}
    bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_CONTROL) ;}
  };


public:
  CommandAngleFilter(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool setParams(const whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam& i_param);
  bool getParams(whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam& i_param);
  bool startControl();
  bool stopControl();

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  std::map<std::string, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > joint_limit_tables_;
  cnoid::BodyPtr robot_ref_;
  cnoid::BodyPtr robot_com_;

  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::VectorX> > qInterpolator_;
  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > rootpInterpolator_;
  std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> rootRInterpolator_; // world系

  // stop時に使う
  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::VectorX> > qOffsetInterpolator_;
  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > rootpOffsetInterpolator_;
  std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> rootROffsetInterpolator_; // world系

  // param
  double minGoalTime_;

};


extern "C"
{
  void CommandAngleFilterInit(RTC::Manager* manager);
};

#endif // CommandAngleFilter_H
