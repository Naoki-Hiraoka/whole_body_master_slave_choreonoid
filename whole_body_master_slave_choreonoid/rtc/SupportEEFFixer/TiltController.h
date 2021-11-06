#ifndef WHOLEBODYMASTERSLAVECHOREONOID_SUPPORTEEFFIXER_TILTCONTROLLER_H
#define WHOLEBODYMASTERSLAVECHOREONOID_SUPPORTEEFFIXER_TILTCONTROLLER_H

#include <cnoid/Body>
#include <vector>
#include <string>
#include <unordered_map>
#include <primitive_motion_level_tools/PrimitiveState.h>

namespace whole_body_master_slave_choreonoid {
  class TiltController {
  public:
    void control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                 const cnoid::BodyPtr& robot_act,
                 const cnoid::BodyPtr& robot_com,
                 double dt,
                 int debugLevel,
                 std::unordered_map<std::string, cnoid::Position>& fixedPoseMap //input & output
                 );

    void start(double transitionTime) { startFlag_ = true; transitionTime_ = transitionTime; }
    void reset() {startFlag_ = false; remainTime_ = 0.0;}
  protected:
    bool calcTiltModification(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                              const std::unordered_map<std::string, cnoid::Position>& fixedPoseMap,
                              const cnoid::BodyPtr& robot_act,
                              const cnoid::BodyPtr& robot_com,
                              int debugLevel);

  protected:
    bool startFlag_ = false;
    double transitionTime_ = 1.0;

    double remainTime_ = 0.0;
    cnoid::Vector3 transitionVelocity_; // Command world frame
  };
}

#endif
