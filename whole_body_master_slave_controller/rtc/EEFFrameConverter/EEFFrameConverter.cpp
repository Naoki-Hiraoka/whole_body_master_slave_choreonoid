#include "EEFFrameConverter.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>

static const char* EEFFrameConverter_spec[] = {
  "implementation_id", "EEFFrameConverter",
  "type_name",         "EEFFrameConverter",
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

EEFFrameConverter::EEFFrameConverter(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t EEFFrameConverter::onInitialize(){

  addInPort("primitiveStateRefIn", this->ports_.m_primitiveStateRefIn_);
  addInPort("qIn", this->ports_.m_qIn_);
  addInPort("basePoseIn", this->ports_.m_basePoseIn_);
  addOutPort("primitiveStateComOut", this->ports_.m_primitiveStateComOut_);
  this->ports_.m_EEFFrameConverterServicePort_.registerProvider("service0", "EEFFrameConverterService", this->ports_.m_service0_);
  addPort(this->ports_.m_EEFFrameConverterServicePort_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->loop_ = 0;

  this->mode_.setNextMode(ControlMode::MODE_IDLE);

  return RTC::RTC_OK;
}

void EEFFrameConverter::getRobot(const std::string& instance_name, EEFFrameConverter::Ports& port, cnoid::BodyPtr& robot) {
  bool updated = false;
  if(port.m_qIn_.isNew()){
    port.m_qIn_.read();
    if(port.m_q_.data.length() == robot->numJoints()){
      for ( int i = 0; i < robot->numJoints(); i++ ){
        robot->joint(i)->q() = port.m_q_.data[i];
      }
    }
    updated = true;
  }
  if(port.m_basePoseIn_.isNew()){
    port.m_basePoseIn_.read();
    robot->rootLink()->p()[0] = port.m_basePose_.data.position.x;
    robot->rootLink()->p()[1] = port.m_basePose_.data.position.y;
    robot->rootLink()->p()[2] = port.m_basePose_.data.position.z;
    robot->rootLink()->R() = cnoid::rotFromRpy(port.m_basePose_.data.orientation.r, port.m_basePose_.data.orientation.p, port.m_basePose_.data.orientation.y);
    updated = true;
  }
  if(updated){
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();
  }
}


void EEFFrameConverter::getPrimitiveState(const std::string& instance_name, EEFFrameConverter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  // primitivestates
  if(port.m_primitiveStateRefIn_.isNew()) {
    port.m_primitiveStateRefIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRef_);
  }
  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void EEFFrameConverter::processModeTransition(const std::string& instance_name, EEFFrameConverter::ControlMode& mode, std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap, std::unordered_set<std::string>& prevEEFs){
  switch(mode.now()){
  case EEFFrameConverter::ControlMode::MODE_SYNC_TO_CONTROL:
    if(mode.pre() == EEFFrameConverter::ControlMode::MODE_IDLE){
    }
    mode.setNextMode(EEFFrameConverter::ControlMode::MODE_CONTROL);
    break;
  case EEFFrameConverter::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == EEFFrameConverter::ControlMode::MODE_CONTROL){
      prevEEFs.clear();
      positionOffsetInterpolatorMap.clear();
    }
    mode.setNextMode(EEFFrameConverter::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void EEFFrameConverter::applyOffset(const std::string& instance_name, const cnoid::BodyPtr& robot, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, cnoid::Position& transForm, double dt, std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap, std::unordered_set<std::string>& prevEEFs) {
  // 消滅したEEFを削除
  for(std::unordered_map<std::string, std::shared_ptr<PositionFilter> >::iterator it = positionOffsetInterpolatorMap.begin(); it != positionOffsetInterpolatorMap.end(); ) {
    if (primitiveStates.primitiveState().find(it->first) == primitiveStates.primitiveState().end())
      it = positionOffsetInterpolatorMap.erase(it);
    else ++it;
  }

  // 増加したEEFの反映
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end(); it++) {
    if(prevEEFs.find(it->first)==prevEEFs.end()){
      cnoid::LinkPtr link = robot->link(it->second->parentLinkName());
      if(link) {
        cnoid::Position currentT = link->T() * it->second->localPose();
        cnoid::Position targetT = transForm * it->second->targetPose();
        std::shared_ptr<PositionFilter> offsetFilter = std::make_shared<PositionFilter>(currentT * targetT.inverse(), cnoid::Vector6::Zero(), cnoid::Vector6::Zero());
        offsetFilter->setGoal(cnoid::Position::Identity(), 3.0);
        positionOffsetInterpolatorMap[it->first] = offsetFilter;
        prevEEFs.insert(it->first);
      }
    }
  }

  // 補間を1ステップすすめる
  for(std::unordered_map<std::string, std::shared_ptr<PositionFilter> >::iterator it = positionOffsetInterpolatorMap.begin(); it!=positionOffsetInterpolatorMap.end();){
    cnoid::Position tmp;
    it->second->get(tmp, dt);
    if(it->second->isEmpty()) it = positionOffsetInterpolatorMap.erase(it);
    else it++;
  }
}

void EEFFrameConverter::calcOutputPorts(const std::string& instance_name,
                                        EEFFrameConverter::Ports& port,
                                        bool isRunning,
                                        double dt,
                                        const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                        const cnoid::Position& transForm,
                                        const std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap){
  if(isRunning){
    // primitiveState
    port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
    for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
      std::string name = std::string(port.m_primitiveStateCom_.data[i].name);
      const std::shared_ptr<primitive_motion_level_tools::PrimitiveState>& primitiveState = primitiveStates.primitiveState().find(name)->second;
      if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
      const cnoid::Vector6& targetWrench = primitiveState->targetWrench();
      cnoid::Vector6 targetWrenchTransformed;
      targetWrenchTransformed.head<3>() = transForm.linear() * targetWrench.head<3>();
      targetWrenchTransformed.tail<3>() = transForm.linear() * targetWrench.tail<3>();
      for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrenchTransformed[j];

      const cnoid::Position& targetPose = primitiveState->targetPose();
      cnoid::Position targetPoseTransformed = transForm * targetPose;
      if(positionOffsetInterpolatorMap.find(name) != positionOffsetInterpolatorMap.end()){
        cnoid::Position offset;
        positionOffsetInterpolatorMap.find(name)->second->get(offset);
        targetPoseTransformed = offset * targetPoseTransformed;
      }
      port.m_primitiveStateCom_.data[i].pose.position.x = targetPoseTransformed.translation()[0];
      port.m_primitiveStateCom_.data[i].pose.position.y = targetPoseTransformed.translation()[1];
      port.m_primitiveStateCom_.data[i].pose.position.z = targetPoseTransformed.translation()[2];
      cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPoseTransformed.linear());
      port.m_primitiveStateCom_.data[i].pose.orientation.r = rpy[0];
      port.m_primitiveStateCom_.data[i].pose.orientation.p = rpy[1];
      port.m_primitiveStateCom_.data[i].pose.orientation.y = rpy[2];

      if(primitiveState->isPoseCGlobal()){
        // TODO !!!
        if(primitiveState->poseC().rows() != 0) {
          std::cerr << "\x1b[31m[" << instance_name << "] " << "isPoseCGlobal is not implemented yet" << "\x1b[39m" << std::endl;
          return;
        }
      }
      if(primitiveState->isWrenchCGlobal()){
        // TODO !!!
        if(primitiveState->wrenchC().rows() != 0) {
          std::cerr << "\x1b[31m[" << instance_name << "] " << "iswrenchCGlobal is not implemented yet" << "\x1b[39m" << std::endl;
          return;
        }
      }
    }
    port.m_primitiveStateComOut_.write();
  }
}

RTC::ReturnCode_t EEFFrameConverter::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  EEFFrameConverter::getPrimitiveState(instance_name, this->ports_, dt, this->primitiveStatesRef_);
  EEFFrameConverter::getRobot(instance_name, this->ports_, this->robot_);

  // mode遷移を実行
  EEFFrameConverter::processModeTransition(instance_name, this->mode_, this->positionOffsetInterpolatorMap_, this->prevEEFs_);

  if(this->mode_.isRunning()) {
    //calcFrameConversion
    EEFFrameConverter::applyOffset(instance_name, this->robot_, this->primitiveStatesRef_, this->transForm_, dt, this->positionOffsetInterpolatorMap_, this->prevEEFs_);
  }else{

  }

  // write outport
  EEFFrameConverter::calcOutputPorts(instance_name, this->ports_, this->mode_.isRunning(), dt, this->primitiveStatesRef_, this->transForm_, this->positionOffsetInterpolatorMap_);

  this->loop_++;
  return RTC::RTC_OK;
}


bool EEFFrameConverter::startControl(){
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool EEFFrameConverter::stopControl(){
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool EEFFrameConverter::setParams(const whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;

  return true;
}


bool EEFFrameConverter::getParams(whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  return true;
}

RTC::ReturnCode_t EEFFrameConverter::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t EEFFrameConverter::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t EEFFrameConverter::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void EEFFrameConverterInit(RTC::Manager* manager) {
        RTC::Properties profile(EEFFrameConverter_spec);
        manager->registerFactory(profile, RTC::Create<EEFFrameConverter>, RTC::Delete<EEFFrameConverter>);
    }
};
