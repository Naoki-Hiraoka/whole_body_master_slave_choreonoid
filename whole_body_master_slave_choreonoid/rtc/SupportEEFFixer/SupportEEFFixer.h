#ifndef SupportEEFFixer_H
#define SupportEEFFixer_H

#include <memory>
#include <map>
#include <time.h>

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
#include "SupportEEFFixerService_impl.h"

class SupportEEFFixer : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveStateRefIn_("primitiveStateRefIn", m_primitiveStateRef_),
      m_previewPrimitiveStateRefIn_("previewPrimitiveStateRefIn", m_previewPrimitiveStateRef_),
      m_qComIn_("qComIn", m_qCom_),
      m_basePoseComIn_("basePoseComIn", m_basePoseCom_),

      m_primitiveStateComOut_("primitiveStateComOut", m_primitiveStateCom_),
      m_verticesOut_("verticesOut", m_vertices_),
      m_previewVerticesOut_("previewVerticesOut", m_previewVertices_),

      m_SupportEEFFixerServicePort_("SupportEEFFixerService") {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRefIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq m_previewPrimitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq> m_previewPrimitiveStateRefIn_;
    RTC::TimedDoubleSeq m_qCom_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qComIn_;
    RTC::TimedPose3D m_basePoseCom_;
    RTC::InPort<RTC::TimedPose3D> m_basePoseComIn_;

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateComOut_;
    RTC::TimedDoubleSeq m_vertices_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_verticesOut_;
    RTC::TimedDoubleSeq m_previewVertices_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_previewVerticesOut_;

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

protected:

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  cnoid::BodyPtr robot_com_; // 下位より
  cnoid::Vector3 prevCOMCom_; // 前回の出力

  // portから受け取ったprimitive motion level 指令
  primitive_motion_level_tools::PrimitiveStates primitiveStates_;// 現在
  primitive_motion_level_tools::PrimitiveStatesSequence previewPrimitiveStates_;//将来

  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > outputCOMOffsetInterpolator_;

  // params
  double regionMargin_;

  // static functions
  static void getPrimitiveState(const std::string& instance_name, SupportEEFFixer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void getPreviewPrimitiveState(const std::string& instance_name, SupportEEFFixer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStatesSequence& previewPrimitiveStates);
  static void getCommandRobot(const std::string& instance_name, SupportEEFFixer::Ports& port, cnoid::BodyPtr& robot);
  static void processModeTransition(const std::string& instance_name, SupportEEFFixer::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& outputCOMOffsetInterpolator, const cnoid::Vector3& prevCOMCom, const primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void passThrough(const std::string& instance_name, const cnoid::BodyPtr& robot_com, std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& outputCOMOffsetInterpolator, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, double dt, cnoid::Vector3& prevCOMCom);
  static void preProcessForControl(const std::string& instance_name);
  static void calcOutputPorts(const std::string& instance_name,
                              SupportEEFFixer::Ports& port,
                              const cnoid::Vector3 prevCOMCom,
                              const Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                              const Eigen::VectorXd& l,
                              const Eigen::VectorXd& u,
                              bool isRunning,
                              std::vector<Eigen::Vector2d>& vertices,
                              std::vector<Eigen::Vector2d>& previewVertices,
                              double dt,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStates);
};


extern "C"
{
  void SupportEEFFixerInit(RTC::Manager* manager);
};

#endif // SupportEEFFixer_H
