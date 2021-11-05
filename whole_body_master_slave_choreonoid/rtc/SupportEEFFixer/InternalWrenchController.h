#ifndef WHOLEBODYMASTERSLAVECHOREONOID_SUPPORTEEFFIXER_INTERNALWRENCHCONTROLLER_H
#define WHOLEBODYMASTERSLAVECHOREONOID_SUPPORTEEFFIXER_INTERNALWRENCHCONTROLLER_H

#include <cnoid/Body>
#include <vector>
#include <string>
#include <unordered_map>

#include <primitive_motion_level_tools/PrimitiveState.h>

namespace whole_body_master_slave_choreonoid {
  class InternalWrenchController {
  public:
    void control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                 const cnoid::BodyPtr& robot_act,
                 const std::vector<double>& pgain,
                 double dt,
                 std::unordered_map<std::string, cnoid::Position>& fixedPoseMap //input & output
                 );

    void start(double transitionTime) { startFlag_ = true; transitionTime_ = transitionTime; }
    void reset() {startFlag_ = false; remainTime_ = 0.0; transitionVelocityMap_.clear(); }
  protected:
    bool calcEEModification(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                            const cnoid::BodyPtr& robot_act,
                            const std::vector<double>& pgain);

  protected:
    bool startFlag_ = false;
    double transitionTime_ = 1.0;

    double remainTime_ = 0.0;
    std::unordered_map<std::string, cnoid::Vector6> transitionVelocityMap_; // local frame
  };
}

#endif
