#ifndef COMController_H
#define COMController_H

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

#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include "COMControllerService_impl.h"

class COMController : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveStateRefIn_("primitiveStateRefIn", m_primitiveStateRef_),
      m_previewPrimitiveStateRefIn_("previewPrimitiveStateRefIn", m_previewPrimitiveStateRef_),

      m_primitiveStateComOut_("primitiveStateComOut", m_primitiveStateCom_),
      m_verticesOut_("verticesOut", m_vertices_),

      m_COMControllerServicePort_("COMControllerService") {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateRefIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_previewPrimitiveStateRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_previewPrimitiveStateRefIn_;

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveStateCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveStateComOut_;
    RTC::TimedDoubleSeq m_vertices_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_verticesOut_;

    COMControllerService_impl m_service0_;
    RTC::CorbaPort m_COMControllerServicePort_;
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
  COMController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam& i_param);
  bool getParams(whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam& i_param);

protected:

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  cnoid::BodyPtr robot_;

  // 1. portから受け取ったprimitive motion level 指令など
  std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> > primitiveStateMap_;

  // params
  double regionMargin_;

  // static functions
  static void readPorts(const std::string& instance_name, COMController::Ports& port);
  static void getPrimitiveState(const std::string& instance_name, const COMController::Ports& port, double dt, std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveStateMap);
  static void processModeTransition(const std::string& instance_name, COMController::ControlMode& mode);
  static void preProcessForControl(const std::string& instance_name);
  static void calcOutputPorts(const std::string& instance_name,
                              COMController::Ports& port,
                              std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveStateMap,
                              double dt,
                              const Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                              const Eigen::VectorXd& l,
                              const Eigen::VectorXd& u,
                              const std::vector<Eigen::Vector2d>& vertices);
};


extern "C"
{
  void COMControllerInit(RTC::Manager* manager);
};

#endif // COMController_H
