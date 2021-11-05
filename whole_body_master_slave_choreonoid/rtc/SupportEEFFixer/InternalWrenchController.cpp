#include "InternalWrenchController.h"

namespace whole_body_master_slave_choreonoid {
  void InternalWrenchController::control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                         const cnoid::BodyPtr& robot_act,
                                         const std::vector<double>& pgain,
                                         double dt,
                                         std::unordered_map<std::string, cnoid::Position>& fixedPoseMap //input & output
                                         ) {
    if(this->startFlag_) {
      // 修正量を計算
      this->startFlag_ = false;
      if(this->calcEEModification(primitiveStates, robot_act, pgain)) this->remainTime_ = this->transitionTime_;
    }

    // 補間が終了
    if(this->remainTime_ == 0.0) {
      transitionVelocityMap_.clear();
      return;
    }

    // calcEEModification時からsupportEEFが変わっていたら止まる
    bool changed = false;
    for(std::unordered_map<std::string, cnoid::Position>::iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      if(this->transitionVelocityMap_.find(it->first) == this->transitionVelocityMap_.end()) changed = true;
    }
    for(std::unordered_map<std::string, cnoid::Vector6>::iterator it=this->transitionVelocityMap_.begin(); it != this->transitionVelocityMap_.end(); it++) {
      if(fixedPoseMap.find(it->first) == fixedPoseMap.end()) changed = true;
    }
    if(changed) {
      this->remainTime_ = 0.0;
      transitionVelocityMap_.clear();
      return;
    }

    // fixedPoseMapを相対的に動かす
    for(std::unordered_map<std::string, cnoid::Position>::iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      cnoid::Vector6 delta = this->transitionVelocityMap_[it->first] * dt; //local
      it->second.translation() += it->second.linear() * delta.head<3>();
      if(delta.tail<3>().norm() != 0.0)
        it->second.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(it->second.linear()) * Eigen::AngleAxisd(delta.tail<3>().norm(),delta.tail<3>().normalized()));
    }
    this->remainTime_ = std::max(0.0, this->remainTime_ - dt);

    return;
  }

  bool InternalWrenchController::calcEEModification(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                                    const cnoid::BodyPtr& robot_act,
                                                    const std::vector<double>& pgain) {

  }

}
