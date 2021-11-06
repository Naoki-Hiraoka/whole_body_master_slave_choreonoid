#ifndef EEFFrameConverter_H
#define EEFFrameConverter_H

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <time.h>
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

#include <cpp_filters/TwoPointInterpolator.h>
#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include "EEFFrameConverterService_impl.h"

class EEFFrameConverter : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveStateRefIn_("primitiveStateRefIn", m_primitiveStateRef_),

      m_qIn_("qIn", m_q_),
      m_basePoseIn_("basePoseIn", m_basePose_),

      m_primitiveStateComOut_("primitiveStateComOut", m_primitiveStateCom_),

      m_EEFFrameConverterServicePort_("EEFFrameConverterService") {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRefIn_;

    RTC::TimedDoubleSeq m_q_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
    RTC::TimedPose3D m_basePose_;
    RTC::InPort<RTC::TimedPose3D> m_basePoseIn_;

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateComOut_;

    EEFFrameConverterService_impl m_service0_;
    RTC::CorbaPort m_EEFFrameConverterServicePort_;
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

  class PositionFilter {
  public:
    PositionFilter(const cnoid::Position& init_T, const cnoid::Vector6& init_v, const cnoid::Vector6& init_a, cpp_filters::interpolation_mode imode=cpp_filters::HOFFARBIB) :
      p_(init_T.translation(), init_v.head<3>(), init_a.head<3>(), imode),
      R_(init_T.linear(), init_v.tail<3>(), init_a.tail<3>(), imode) {}
    void get(cnoid::Position& T, double dt=0.0) {
      cnoid::Vector3 p; cnoid::Matrix3 R;
      p_.get(p, dt); T.translation() = p;
      R_.get(R,dt);T.linear() = R;
    }
    void Reset(const cnoid::Position& T) {
      p_.reset(T.translation());
      R_.reset(T.linear());
    }
    bool isEmpty() {
      return p_.isEmpty() && R_.isEmpty();
    }
    void setGoal(const cnoid::Position& T, double t){
      p_.setGoal(T.translation(),t);
      R_.setGoal(T.linear(),t);
    }
  protected:
    cpp_filters::TwoPointInterpolator<cnoid::Vector3> p_;
    cpp_filters::TwoPointInterpolatorSO3 R_;
  };

public:
  EEFFrameConverter(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& i_param);
  bool getParams(whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& i_param);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  cnoid::BodyPtr robot_;

  // portから受け取ったprimitive motion level 指令
  primitive_motion_level_tools::PrimitiveStates primitiveStatesRef_;

  // robotの座標系の原点->Refの座標系の原点のtransform
  cnoid::Position transForm_;
  // 新しく増えたEEFに適用するオフセット
  std::unordered_map<std::string, std::shared_ptr<PositionFilter> > positionOffsetInterpolatorMap_;
  std::unordered_set<std::string> prevEEFs_;

  // params

  // static functions
  static void getRobot(const std::string& instance_name, EEFFrameConverter::Ports& port, cnoid::BodyPtr& robot);
  static void getPrimitiveState(const std::string& instance_name, EEFFrameConverter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void processModeTransition(const std::string& instance_name, EEFFrameConverter::ControlMode& mode, std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap, std::unordered_set<std::string>& prevEEFs);
  static void calcFrameConversion(const std::string& instance_name, const cnoid::BodyPtr& robot, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, cnoid::Position& transForm);
  static void applyOffset(const std::string& instance_name, const cnoid::BodyPtr& robot, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, cnoid::Position& transForm, double dt, std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap, std::unordered_set<std::string>& prevEEFs);
  static void calcOutputPorts(const std::string& instance_name,
                              EEFFrameConverter::Ports& port,
                              bool isRunning,
                              double dt,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                              const cnoid::Position& transForm,
                              const std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap);
};


extern "C"
{
  void EEFFrameConverterInit(RTC::Manager* manager);
};

#endif // EEFFrameConverter_H
