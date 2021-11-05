#include "SupportEEFFixer.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* SupportEEFFixer_spec[] = {
  "implementation_id", "SupportEEFFixer",
  "type_name",         "SupportEEFFixer",
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

SupportEEFFixer::SupportEEFFixer(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t SupportEEFFixer::onInitialize(){

  addInPort("primitiveStateRefIn", this->ports_.m_primitiveStateRefIn_);
  addInPort("qComIn", this->ports_.m_qComIn_);
  addInPort("basePoseComIn", this->ports_.m_basePoseComIn_);
  addOutPort("primitiveStateComOut", this->ports_.m_primitiveStateComOut_);
  this->ports_.m_SupportEEFFixerServicePort_.registerProvider("service0", "SupportEEFFixerService", this->ports_.m_service0_);
  addPort(this->ports_.m_SupportEEFFixerServicePort_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_com_ = bodyLoader.load(fileName);
  if(!this->robot_com_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->loop_ = 0;

  this->mode_.setNextMode(ControlMode::MODE_IDLE);

  return RTC::RTC_OK;
}

void SupportEEFFixer::getCommandRobot(const std::string& instance_name, SupportEEFFixer::Ports& port, cnoid::BodyPtr& robot) {
  bool updated = false;
  if(port.m_qComIn_.isNew()){
    port.m_qComIn_.read();
    if(port.m_qCom_.data.length() == robot->numJoints()){
      for ( int i = 0; i < robot->numJoints(); i++ ){
        robot->joint(i)->q() = port.m_qCom_.data[i];
      }
    }
    updated = true;
  }
  if(port.m_basePoseComIn_.isNew()){
    port.m_basePoseComIn_.read();
    robot->rootLink()->p()[0] = port.m_basePoseCom_.data.position.x;
    robot->rootLink()->p()[1] = port.m_basePoseCom_.data.position.y;
    robot->rootLink()->p()[2] = port.m_basePoseCom_.data.position.z;
    robot->rootLink()->R() = cnoid::rotFromRpy(port.m_basePoseCom_.data.orientation.r, port.m_basePoseCom_.data.orientation.p, port.m_basePoseCom_.data.orientation.y);
    updated = true;
  }
  if(updated){
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();
  }
}


void SupportEEFFixer::getPrimitiveState(const std::string& instance_name, SupportEEFFixer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  if(port.m_primitiveStateRefIn_.isNew()) {
    port.m_primitiveStateRefIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRef_);
  }

  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void SupportEEFFixer::processModeTransition(const std::string& instance_name, SupportEEFFixer::ControlMode& mode, std::unordered_map<std::string, cnoid::Position>& fixedPoseMap, std::unordered_map<std::string, std::pair<std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >, std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> > >& outputOffsetInterpolator, const primitive_motion_level_tools::PrimitiveStates& primitiveStates){
  switch(mode.now()){
  case SupportEEFFixer::ControlMode::MODE_SYNC_TO_CONTROL:
    mode.setNextMode(SupportEEFFixer::ControlMode::MODE_CONTROL);
    break;
  case SupportEEFFixer::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == SupportEEFFixer::ControlMode::MODE_CONTROL){
      outputOffsetInterpolator.clear();
      for(std::unordered_map<std::string, cnoid::Position>::iterator it = fixedPoseMap.begin(); it != fixedPoseMap.end() ;it++){
        if(primitiveStates.primitiveState().find(it->first) == primitiveStates.primitiveState().end()) continue;
        const cnoid::Position& poseRef = primitiveStates.primitiveState().find(it->first)->second->targetPose();
        std::pair<std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >, std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> > interpolator;
        interpolator.first = std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(it->second.translation() - poseRef.translation(), cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
        interpolator.first->setGoal(cnoid::Vector3::Zero(), 3.0);
        interpolator.second = std::make_shared<cpp_filters::TwoPointInterpolatorSO3>(it->second.linear() * poseRef.linear().transpose(), cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
        interpolator.second->setGoal(cnoid::Matrix3::Identity(), 3.0);
        outputOffsetInterpolator[it->first] = interpolator;
      }
      fixedPoseMap.clear();
    }
    mode.setNextMode(SupportEEFFixer::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void SupportEEFFixer::addOrRemoveFixedEEFMap(const std::string& instance_name, const cnoid::BodyPtr& robot_com, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, std::unordered_map<std::string, cnoid::Position>& fixedPoseMap, std::unordered_set<std::string>& newFixedEEF) {
  // 消滅したSupportEEFを削除
  for(std::unordered_map<std::string, cnoid::Position >::iterator it = fixedPoseMap.begin(); it != fixedPoseMap.end(); ) {
    if (primitiveStates.primitiveState().find(it->first) == primitiveStates.primitiveState().end() ||
        !primitiveStates.primitiveState().find(it->first)->second->supportCOM())
      it = fixedPoseMap.erase(it);
    else ++it;
  }

  // 増加したSupportEEFの反映
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end(); it++) {
    if(it->second->supportCOM() && fixedPoseMap.find(it->first)==fixedPoseMap.end()){
      cnoid::LinkPtr link = robot_com->link(it->second->parentLinkName());
      if(link) {
        fixedPoseMap[it->first] = link->T() * it->second->localPose();
        newFixedEEF.insert(it->first);
      }
    }
  }
}

void SupportEEFFixer::calcOutputPorts(const std::string& instance_name,
                                      SupportEEFFixer::Ports& port,
                                      bool isRunning,
                                      double dt,
                                      const std::unordered_map<std::string, cnoid::Position>& fixedPoseMap,
                                      std::unordered_map<std::string, std::pair<std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >, std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> > >& outputOffsetInterpolator,
                                      const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                      const std::unordered_set<std::string>& newFixedEEF){
  // primitiveState
  port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
  for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
    std::string name = std::string(port.m_primitiveStateCom_.data[i].name);
    if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
    const cnoid::Vector6& targetWrench = primitiveStates.primitiveState().find(name)->second->targetWrench();
    for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrench[j];

    cnoid::Position targetPose;
    if(fixedPoseMap.find(name) != fixedPoseMap.end()) {
      targetPose = fixedPoseMap.find(name)->second;
      if(newFixedEEF.find(name) != newFixedEEF.end()) port.m_primitiveStateCom_.data[i].time = 0.0;//ワープ
    } else {
      // reference の値を補間してそのまま流す
      targetPose = primitiveStates.primitiveState().find(name)->second->targetPose();
      // stopControl直後はoffsetを適用
      if (outputOffsetInterpolator.find(name) != outputOffsetInterpolator.end()){
        cnoid::Vector3 offsetp = cnoid::Vector3::Zero();
        if (!outputOffsetInterpolator[name].first->isEmpty()) outputOffsetInterpolator[name].first->get(offsetp, dt);
        targetPose.translation() = targetPose.translation() + offsetp;
        cnoid::Matrix3 offsetR = cnoid::Matrix3d::Identity();
        if (!outputOffsetInterpolator[name].second->isEmpty()) outputOffsetInterpolator[name].second->get(offsetR, dt);
        targetPose.linear() = offsetR * targetPose.linear();
        if(outputOffsetInterpolator[name].first->isEmpty() && outputOffsetInterpolator[name].second->isEmpty()) outputOffsetInterpolator.erase(name);
      }
    }
    port.m_primitiveStateCom_.data[i].pose.position.x = targetPose.translation()[0];
    port.m_primitiveStateCom_.data[i].pose.position.y = targetPose.translation()[1];
    port.m_primitiveStateCom_.data[i].pose.position.z = targetPose.translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPose.linear());
    port.m_primitiveStateCom_.data[i].pose.orientation.r = rpy[0];
    port.m_primitiveStateCom_.data[i].pose.orientation.p = rpy[1];
    port.m_primitiveStateCom_.data[i].pose.orientation.y = rpy[2];
  }
  port.m_primitiveStateComOut_.write();
}

RTC::ReturnCode_t SupportEEFFixer::onExecute(RTC::UniqueId ec_id){

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  SupportEEFFixer::getPrimitiveState(instance_name, this->ports_, dt, this->primitiveStatesRef_);
  SupportEEFFixer::getCommandRobot(instance_name, this->ports_, this->robot_com_);

  // mode遷移を実行
  SupportEEFFixer::processModeTransition(instance_name, this->mode_, this->fixedPoseMap_, this->outputOffsetInterpolator_, this->primitiveStatesRef_);

  std::unordered_set<std::string> newFixedEEF;
  if(this->mode_.isRunning()) {
    SupportEEFFixer::addOrRemoveFixedEEFMap(instance_name, this->robot_com_, this->primitiveStatesRef_, this->fixedPoseMap_, newFixedEEF);

    // this->scfrController_.control(this->primitiveStates_, this->previewPrimitiveStates_, this->robot_com_->mass(), dt, this->regionMargin_, this->prevCOMCom_, M, l, u, vertices, previewVertices, this->debugLevel_);
  }else{

  }

  // write outport
  SupportEEFFixer::calcOutputPorts(instance_name, this->ports_, this->mode_.isRunning(), dt, this->fixedPoseMap_, this->outputOffsetInterpolator_, this->primitiveStatesRef_, newFixedEEF);

  this->loop_++;
  return RTC::RTC_OK;
}


bool SupportEEFFixer::startControl(){
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool SupportEEFFixer::stopControl(){
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool SupportEEFFixer::setParams(const whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;
  return true;
}


bool SupportEEFFixer::getParams(whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  return true;
}

RTC::ReturnCode_t SupportEEFFixer::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t SupportEEFFixer::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t SupportEEFFixer::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void SupportEEFFixerInit(RTC::Manager* manager) {
        RTC::Properties profile(SupportEEFFixer_spec);
        manager->registerFactory(profile, RTC::Create<SupportEEFFixer>, RTC::Delete<SupportEEFFixer>);
    }
};
