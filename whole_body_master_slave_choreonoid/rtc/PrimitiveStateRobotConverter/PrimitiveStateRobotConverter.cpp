#include "PrimitiveStateRobotConverter.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>
#include <cnoid/ValueTree>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* PrimitiveStateRobotConverter_spec[] = {
  "implementation_id", "PrimitiveStateRobotConverter",
  "type_name",         "PrimitiveStateRobotConverter",
  "description",       "wholebodymasterslave component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

PrimitiveStateRobotConverter::PrimitiveStateRobotConverter(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_()
{
}

RTC::ReturnCode_t PrimitiveStateRobotConverter::onInitialize(){

  addInPort("primitiveStateRefIn", this->ports_.m_primitiveStateRefIn_);
  addInPort("primitiveStateRobotIn", this->ports_.m_primitiveStateRobotIn_);
  addOutPort("primitiveStateComOut", this->ports_.m_primitiveStateComOut_);


  this->loop_ = 0;

  return RTC::RTC_OK;
}

void PrimitiveStateRobotConverter::getPrimitiveStateRef(const std::string& instance_name, PrimitiveStateRobotConverter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  if(port.m_primitiveStateRefIn_.isNew()) {
    port.m_primitiveStateRefIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRef_);
  }

  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void PrimitiveStateRobotConverter::getPrimitiveStateRobot(const std::string& instance_name, PrimitiveStateRobotConverter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  if(port.m_primitiveStateRobotIn_.isNew()) {
    port.m_primitiveStateRobotIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRobot_);
  }

  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void PrimitiveStateRobotConverter::calcOutputPorts(const std::string& instance_name,
                                               const primitive_motion_level_tools::PrimitiveStates& primitiveStatesRef,
                                                   const primitive_motion_level_tools::PrimitiveStates& primitiveStatesRobot,
                                               double dt,
                                               PrimitiveStateRobotConverter::Ports& port){
  std::unordered_map<std::string,int> primitiveStatesRobotIdxMap;
  for(int i=0;i<port.m_primitiveStateRobot_.data.length();i++){
    primitiveStatesRobotIdxMap[std::string(port.m_primitiveStateRobot_.data[i].name)] = i;
  }

  // primitiveState
  port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
  for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
    std::string name = std::string(port.m_primitiveStateCom_.data[i].name);
    if(primitiveStatesRobotIdxMap.find(name) != primitiveStatesRobotIdxMap.end()){
      // overwrite
      port.m_primitiveStateCom_.data[i] = port.m_primitiveStateRobot_.data[primitiveStatesRobotIdxMap[name]];
    }

    std::shared_ptr<primitive_motion_level_tools::PrimitiveState> primitiveState = primitiveStatesRef.primitiveState().find(name)->second;
    if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
    const cnoid::Vector6& targetWrench = primitiveState->targetWrench();
    for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrench[j];
    cnoid::Position targetPose = primitiveState->targetPose();
    port.m_primitiveStateCom_.data[i].pose.position.x = targetPose.translation()[0];
    port.m_primitiveStateCom_.data[i].pose.position.y = targetPose.translation()[1];
    port.m_primitiveStateCom_.data[i].pose.position.z = targetPose.translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPose.linear());
    port.m_primitiveStateCom_.data[i].pose.orientation.r = rpy[0];
    port.m_primitiveStateCom_.data[i].pose.orientation.p = rpy[1];
    port.m_primitiveStateCom_.data[i].pose.orientation.y = rpy[2];

    for(int j=0;j<6;j++) port.m_primitiveStateCom_.data[i].poseFollowGain[j] = port.m_primitiveStateRef_.data[i].poseFollowGain[j];
    for(int j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrenchFollowGain[j] = port.m_primitiveStateRef_.data[i].wrenchFollowGain[j];

    port.m_primitiveStateCom_.data[i].supportCOM = port.m_primitiveStateRef_.data[i].supportCOM;
  }
  port.m_primitiveStateComOut_.write();
}

RTC::ReturnCode_t PrimitiveStateRobotConverter::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();
  // read ports
  PrimitiveStateRobotConverter::getPrimitiveStateRef(instance_name, this->ports_, dt, this->primitiveStatesRef_);
  PrimitiveStateRobotConverter::getPrimitiveStateRobot(instance_name, this->ports_, dt, this->primitiveStatesRobot_);

  // write outport
  PrimitiveStateRobotConverter::calcOutputPorts(instance_name, this->primitiveStatesRef_, this->primitiveStatesRobot_, dt, this->ports_);

  this->loop_++;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PrimitiveStateRobotConverter::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveStateRobotConverter::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveStateRobotConverter::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void PrimitiveStateRobotConverterInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveStateRobotConverter_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveStateRobotConverter>, RTC::Delete<PrimitiveStateRobotConverter>);
    }
};
