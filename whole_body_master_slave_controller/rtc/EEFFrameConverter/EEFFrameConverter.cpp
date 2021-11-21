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
  addOutPort("refFramePoseOut", this->ports_.m_refFramePoseOut_);
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

  this->transForm_ = cnoid::Position::Identity();

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

void EEFFrameConverter::processModeTransition(const std::string& instance_name, EEFFrameConverter::ControlMode& mode, std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionInterpolatorMap, std::unordered_map<std::string, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > >& frameConversionWeightInterpolatorMap){
  switch(mode.now()){
  case EEFFrameConverter::ControlMode::MODE_SYNC_TO_CONTROL:
    if(mode.pre() == EEFFrameConverter::ControlMode::MODE_IDLE){
    }
    mode.setNextMode(EEFFrameConverter::ControlMode::MODE_CONTROL);
    break;
  case EEFFrameConverter::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == EEFFrameConverter::ControlMode::MODE_CONTROL){
      positionInterpolatorMap.clear();
      frameConversionWeightInterpolatorMap.clear();
    }
    mode.setNextMode(EEFFrameConverter::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void EEFFrameConverter::calcFrameConversion(const std::string& instance_name, const cnoid::BodyPtr& robot, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, cnoid::Position& transForm, std::unordered_map<std::string, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > >& frameConversionWeightInterpolatorMap, double dt) {

  // 1. weightを計算. posefollowgainが0でないeefのみ変換の基準とする
  //  消滅したEEFを削除
  for(std::unordered_map<std::string, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > >::iterator it=frameConversionWeightInterpolatorMap.begin();it!=frameConversionWeightInterpolatorMap.end();){
    if (primitiveStates.primitiveState().find(it->first) == primitiveStates.primitiveState().end() || primitiveStates.primitiveState().find(it->first)->second->poseFollowGain().norm()==0.0)
      it = frameConversionWeightInterpolatorMap.erase(it);
    else ++it;
  }
  //  新しく増えたEEFを追加 0 -> 1
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it=primitiveStates.primitiveState().begin();it!=primitiveStates.primitiveState().end();it++){
    if(primitiveStates.primitiveState().find(it->first)->second->poseFollowGain().norm()!=0.0 && frameConversionWeightInterpolatorMap.find(it->first)==frameConversionWeightInterpolatorMap.end()){
      std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > frameConversionWeightInterpolator = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0, 0.0, 0.0, cpp_filters::LINEAR); // 0はよくないが、後ろの処理でdt進むので0ではなくなる
      frameConversionWeightInterpolator->setGoal(1.0, 1.0);
      frameConversionWeightInterpolatorMap[it->first] = frameConversionWeightInterpolator;
    }
  }

  // 2. 座標を取得
  std::vector<cnoid::Position> refTs;
  std::vector<cnoid::Position> actTs;
  std::vector<double> weights;
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it=primitiveStates.primitiveState().begin();it!=primitiveStates.primitiveState().end();it++){
    if(it->second->poseFollowGain().norm()==0.0) continue;
    const cnoid::LinkPtr& link = robot->link(it->second->parentLinkName());
    if(!link) continue;
    refTs.push_back(it->second->targetPose());
    actTs.push_back(link->T() * it->second->localPose());
    double w, v, a;
    frameConversionWeightInterpolatorMap[it->first]->get(w, v, a, dt);
    weights.push_back(w);
  }
  if(refTs.size() == 0) return;

  // 3. 変換(transForm)を計算 // ref座標系で表現したrobot座標系の原点
  for(size_t loop=0;loop<3;loop++){
    // calc error, jacobian
    cnoid::VectorX error = cnoid::VectorX::Zero(refTs.size()*4);
    Eigen::SparseMatrix<double,Eigen::RowMajor> J(refTs.size()*4,4);
    Eigen::SparseMatrix<double,Eigen::RowMajor> W(refTs.size()*4,refTs.size()*4);
    for(size_t i=0;i<refTs.size();i++){
      const cnoid::Position& refT = refTs[i];
      const cnoid::Position& actT = transForm * actTs[i]; // ref座標系に揃える

      error.segment<3>(4*i) = refT.translation() - actT.translation();
      cnoid::Matrix3 diffR = refT.linear() * actT.linear().transpose();
      cnoid::Matrix3 horizontal_diffR;
      {
        cnoid::Vector3 localZ = diffR * cnoid::Vector3::UnitZ();
        cnoid::Vector3 cross = localZ.cross(cnoid::Vector3::UnitZ());
        double cos = std::min(1.0,std::max(-1.0,localZ.dot(cnoid::Vector3::UnitZ()))); // acosは定義域外のときnanを返す
        double angle = std::acos(cos); // 0~pi
        cnoid::Vector3 axis = (cross.norm()!=0.0) ? cross.normalized() : cnoid::Vector3::UnitX();// include sign
        horizontal_diffR = Eigen::AngleAxisd(angle,axis).toRotationMatrix() * diffR;
      }
      error[4*i+3] = cnoid::rpyFromRot(horizontal_diffR)[2];

      cnoid::Vector3 dp = cnoid::Vector3::UnitZ().cross(actT.translation() - transForm.translation());
      J.coeffRef(4*i+0,0) = 1.0; J.coeffRef(4*i+0,3) = dp[0];
      J.coeffRef(4*i+1,1) = 1.0; J.coeffRef(4*i+1,3) = dp[1];
      J.coeffRef(4*i+2,2) = 1.0; J.coeffRef(4*i+2,3) = dp[2];
      J.coeffRef(4*i+3,3) = 1.0;
    }

    // solve
    for(size_t i=0;i<refTs.size();i++){
      for(int j=0;j<3;j++) W.coeffRef(i*4+j,i*4+j) = weights[i];
      W.coeffRef(i*4+3,i*4+3) = weights[i] * 0.01;//yawの寄与を小さく
    }
    // d_origin = (Jt W J)^-1 Jt W error
    // <=>
    // (Jt W J) d_origin = Jt W error
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    solver.compute(J.transpose() * W * J);
    if(solver.info()!=Eigen::Success) {
      // decomposition failed. ありえないか
      return;
    }
    cnoid::Vector4 d_origin = solver.solve(J.transpose() * W * error);
    if(solver.info()!=Eigen::Success) {
      // solving failed ありえないか
      return;
    }

    // apply result
    transForm.translation() += d_origin.head<3>();
    transForm.linear() = cnoid::rotFromRpy(0,0,cnoid::rpyFromRot(transForm.linear())[2] + d_origin[3]);
  }

}

void EEFFrameConverter::applyPositionFilter(const std::string& instance_name, const cnoid::BodyPtr& robot, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, cnoid::Position& transForm, double dt, std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionInterpolatorMap) {
  // 消滅したEEFを削除
  for(std::unordered_map<std::string, std::shared_ptr<PositionFilter> >::iterator it = positionInterpolatorMap.begin(); it != positionInterpolatorMap.end(); ) {
    if (primitiveStates.primitiveState().find(it->first) == primitiveStates.primitiveState().end())
      it = positionInterpolatorMap.erase(it);
    else ++it;
  }
  // 増加したEEFの反映
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end(); it++) {
    if(positionInterpolatorMap.find(it->first)==positionInterpolatorMap.end()){
      std::shared_ptr<PositionFilter> filter = std::make_shared<PositionFilter>(cnoid::Position::Identity(), cnoid::Vector6::Zero(), cnoid::Vector6::Zero());
      cnoid::LinkPtr link = robot->link(it->second->parentLinkName());
      if(link) filter->Reset(link->T() * it->second->localPose());
      else filter->Reset(transForm.inverse() * it->second->targetPose());
      positionInterpolatorMap[it->first] = filter;
    }
  }

  // 補間を1ステップすすめる
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end(); it++) {
    positionInterpolatorMap[it->first]->setGoal(transForm.inverse() * it->second->targetPose(), 1.0);
  }
}

void EEFFrameConverter::calcOutputPorts(const std::string& instance_name,
                                        EEFFrameConverter::Ports& port,
                                        bool isRunning,
                                        double dt,
                                        const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                        const cnoid::Position& transForm,
                                        const std::unordered_map<std::string, std::shared_ptr<PositionFilter> >& positionOffsetInterpolatorMap){
  cnoid::Position transFormInv = transForm.inverse();
  if(isRunning){
    // primitiveState
    port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
    for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
      std::string name = std::string(port.m_primitiveStateCom_.data[i].name);
      const std::shared_ptr<primitive_motion_level_tools::PrimitiveState>& primitiveState = primitiveStates.primitiveState().find(name)->second;
      if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
      const cnoid::Vector6& targetWrench = primitiveState->targetWrench();
      cnoid::Vector6 targetWrenchTransformed;
      targetWrenchTransformed.head<3>() = transFormInv.linear() * targetWrench.head<3>();
      targetWrenchTransformed.tail<3>() = transFormInv.linear() * targetWrench.tail<3>();
      for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrenchTransformed[j];

      cnoid::Position targetPoseTransformed;
      positionOffsetInterpolatorMap.find(name)->second->get(targetPoseTransformed, dt);
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

  // refFramePose robot座標系で表現したref座標系の原点
  port.m_refFramePose_.data.position.x = transFormInv.translation()[0];
  port.m_refFramePose_.data.position.y = transFormInv.translation()[1];
  port.m_refFramePose_.data.position.z = transFormInv.translation()[2];
  cnoid::Vector3 rpy = cnoid::rpyFromRot(transFormInv.linear());
  port.m_refFramePose_.data.orientation.r = rpy[0];
  port.m_refFramePose_.data.orientation.p = rpy[1];
  port.m_refFramePose_.data.orientation.y = rpy[2];
  port.m_refFramePose_.tm = port.m_q_.tm;
  port.m_refFramePoseOut_.write();

}

RTC::ReturnCode_t EEFFrameConverter::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  EEFFrameConverter::getPrimitiveState(instance_name, this->ports_, dt, this->primitiveStatesRef_);
  EEFFrameConverter::getRobot(instance_name, this->ports_, this->robot_);

  // mode遷移を実行
  EEFFrameConverter::processModeTransition(instance_name, this->mode_, this->positionInterpolatorMap_, this->frameConversionWeightInterpolatorMap_);

  if(this->mode_.isRunning()) {
    EEFFrameConverter::calcFrameConversion(instance_name, this->robot_, this->primitiveStatesRef_, this->transForm_, this->frameConversionWeightInterpolatorMap_, dt);

    EEFFrameConverter::applyPositionFilter(instance_name, this->robot_, this->primitiveStatesRef_, this->transForm_, dt, this->positionInterpolatorMap_);

  }else{

  }

  // write outport
  EEFFrameConverter::calcOutputPorts(instance_name, this->ports_, this->mode_.isRunning(), dt, this->primitiveStatesRef_, this->transForm_, this->positionInterpolatorMap_);

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
