#ifndef SupportEEFFixer_H
#define SupportEEFFixer_H

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
#include <cpp_filters/FirstOrderLowpassFilter.h>
#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include "SupportEEFFixerService_impl.h"
#include "InternalWrenchController.h"
#include "TiltController.h"

class SupportEEFFixer : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveStateRefIn_("primitiveStateRefIn", m_primitiveStateRef_),
      m_qComIn_("qComIn", m_qCom_),
      m_basePoseComIn_("basePoseComIn", m_basePoseCom_),
      m_qActIn_("qActIn", m_qAct_),
      m_tauActIn_("tauActIn", m_tauAct_),
      m_imuActIn_("imuActIn", m_imuAct_),

      m_primitiveStateComOut_("primitiveStateComOut", m_primitiveStateCom_),

      m_SupportEEFFixerServicePort_("SupportEEFFixerService") {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRefIn_;
    RTC::TimedDoubleSeq m_qCom_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qComIn_;
    RTC::TimedPose3D m_basePoseCom_;
    RTC::InPort<RTC::TimedPose3D> m_basePoseComIn_;

    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_tauAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_;
    RTC::TimedOrientation3D m_imuAct_;
    RTC::InPort<RTC::TimedOrientation3D> m_imuActIn_;

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateComOut_;

    SupportEEFFixerService_impl m_service0_;
    RTC::CorbaPort m_SupportEEFFixerServicePort_;
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
  SupportEEFFixer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param);
  bool getParams(whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param);
  void applyWrenchDistributionControl(double transitionTime);
  void applyTiltControl(double transitionTime);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  cnoid::BodyPtr robot_act_;
  cnoid::BodyPtr robot_com_; // 下位より
  std::vector<std::shared_ptr<cpp_filters::FirstOrderLowPassFilter<double> > > tauActFilter_;
  std::vector<double> pgain_;

  // portから受け取ったprimitive motion level 指令
  primitive_motion_level_tools::PrimitiveStates primitiveStatesRef_;
  std::unordered_map<std::string, std::shared_ptr<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> > > wrenchFilterMap_;

  // SupportEEFのtargetPoseをこの値で上書きする
  std::unordered_map<std::string, cnoid::Position> fixedPoseMap_;
  std::unordered_map<std::string, std::pair<std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >, std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> > > outputOffsetInterpolator_; // stopControlしたときに使う

  whole_body_master_slave_choreonoid::InternalWrenchController internalWrenchController_;
  whole_body_master_slave_choreonoid::TiltController tiltController_;

  // params
  std::vector<cnoid::LinkPtr> useJoints_;

  // static functions
  static void getCommandRobot(const std::string& instance_name, SupportEEFFixer::Ports& port, cnoid::BodyPtr& robot);
  static void getPrimitiveState(const std::string& instance_name, SupportEEFFixer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates, std::unordered_map<std::string, std::shared_ptr<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> > >& wrenchFilterMap);
  static void calcActualRobot(const std::string& instance_name, SupportEEFFixer::Ports& port, cnoid::BodyPtr& robot, std::vector<std::shared_ptr<cpp_filters::FirstOrderLowPassFilter<double> >>& tauActFilter, double dt);
  static void processModeTransition(const std::string& instance_name, SupportEEFFixer::ControlMode& mode, std::unordered_map<std::string, cnoid::Position>& fixedPoseMap, std::unordered_map<std::string, std::pair<std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >, std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> > >& outputOffsetInterpolator, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, whole_body_master_slave_choreonoid::InternalWrenchController& internalWrenchController, whole_body_master_slave_choreonoid::TiltController& tiltController);
  static void addOrRemoveFixedEEFMap(const std::string& instance_name, const cnoid::BodyPtr& robot_com, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, std::unordered_map<std::string, cnoid::Position>& fixedPoseMap, std::unordered_set<std::string>& newFixedEEF);
  static void calcOutputPorts(const std::string& instance_name,
                              SupportEEFFixer::Ports& port,
                              bool isRunning,
                              double dt,
                              const std::unordered_map<std::string, cnoid::Position>& fixedPoseMap,
                              std::unordered_map<std::string, std::pair<std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >, std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> > >& outputOffsetInterpolator,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                              const std::unordered_set<std::string>& newFixedEEF);
};


extern "C"
{
  void SupportEEFFixerInit(RTC::Manager* manager);
};

#endif // SupportEEFFixer_H
