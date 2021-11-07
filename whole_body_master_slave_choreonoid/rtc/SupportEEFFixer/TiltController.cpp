#include "TiltController.h"
#include <cnoid/EigenUtil>
#include <iostream>

namespace whole_body_master_slave_choreonoid {
  void TiltController::control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                               const cnoid::BodyPtr& robot_act,
                               const cnoid::BodyPtr& robot_com,
                               double dt,
                               int debugLevel,
                               std::unordered_map<std::string, cnoid::Position>& fixedPoseMap //input & output
                               ) {
    if(this->startFlag_) {
      // 修正量を計算
      this->startFlag_ = false;
      if(this->calcTiltModification(primitiveStates, fixedPoseMap, robot_act, robot_com, debugLevel)) this->remainTime_ = this->transitionTime_;
    }

    // 補間が終了
    if(this->remainTime_ == 0.0) {
      transitionVelocity_ = cnoid::Vector3::Zero();
      return;
    }

    // fixedPoseMapを, 中点を中心に回転させる
    if(fixedPoseMap.size() != 0 && this->transitionVelocity_.norm() > 0.0) {
      cnoid::Vector3 midp = cnoid::Vector3::Zero();
      for(std::unordered_map<std::string, cnoid::Position>::iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
        midp += it->second.translation();
      }
      midp /= fixedPoseMap.size();

      cnoid::AngleAxis rotR(this->transitionVelocity_.norm()*dt,this->transitionVelocity_.normalized());
      for(std::unordered_map<std::string, cnoid::Position>::iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
        it->second.translation() = (midp + rotR * (it->second.translation() - midp)).eval();
        it->second.linear() = (rotR * it->second.linear()).eval();
      }
    }

    this->remainTime_ = std::max(0.0, this->remainTime_ - dt);

    return;
  }

  bool TiltController::calcTiltModification(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                            const std::unordered_map<std::string, cnoid::Position>& fixedPoseMap,
                                            const cnoid::BodyPtr& robot_act,
                                            const cnoid::BodyPtr& robot_com,
                                            int debugLevel) {
    /*
      robot_comのfixedPose系で表現した鉛直軸と、robot_actのfixedPose系で表現した鉛直軸を一致させる.

      rootLinkやgsensorの座標を用いて一致させようとしても、位置P制御ぶんの関節角度誤差のためfixedPoseの傾きが正しくならない
    */
    cnoid::Vector3 vel = cnoid::Vector3::Zero();
    for(std::unordered_map<std::string, cnoid::Position>::const_iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      cnoid::Vector3 Zaxis_in_com_frame = it->second.linear().transpose() * cnoid::Vector3::UnitZ();
      cnoid::Vector3 Zaxis_in_act_frame = (robot_act->link(primitiveStates.primitiveState().find(it->first)->second->parentLinkName())->R() * primitiveStates.primitiveState().find(it->first)->second->localPose().linear()).transpose() * cnoid::Vector3::UnitZ();

      cnoid::Vector3 cross = Zaxis_in_com_frame.cross(Zaxis_in_act_frame);
      if(cross.norm()==0){
        this->transitionVelocity_ = cnoid::Vector3::Zero();
      }else{
        double angle = std::acos(Zaxis_in_com_frame.dot(Zaxis_in_act_frame)); // 0~pi
        cnoid::Vector3 axis = cross.normalized(); // include sign
        vel -= axis * angle / this->transitionTime_;
      }

      if(debugLevel > 0){
        std::cerr << "[TiltController::calcTiltModification] Zaxis_in_com_frame: " << Zaxis_in_com_frame.transpose() << ", Zaxis_in_act_frame: " << Zaxis_in_act_frame.transpose() << std::endl;
      }
    }

    if(fixedPoseMap.size()>0) this->transitionVelocity_ = vel / fixedPoseMap.size();
    else this->transitionVelocity_ = cnoid::Vector3::Zero();

    if(debugLevel > 0){
      std::cerr << "[TiltController::calcTiltModification] velocity: " << this->transitionVelocity_.transpose() << std::endl;
    }


    return true;
  }

}
