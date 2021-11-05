#ifndef WHOLEBODYMASTERSLAVECHOREONOID_SUPPORTEEFFIXER_INTERNALWRENCHCONTROLLER_H
#define WHOLEBODYMASTERSLAVECHOREONOID_SUPPORTEEFFIXER_INTERNALWRENCHCONTROLLER_H

#include <cnoid/Body>
#include <vector>
#include <string>
#include <unordered_map>

#include <primitive_motion_level_tools/PrimitiveState.h>
#include <prioritized_qp_base/PrioritizedQPBaseSolver.h>
#include <prioritized_qp/PrioritizedQPSolver.h>
#include <ik_constraint/PositionConstraint.h>

namespace whole_body_master_slave_choreonoid {
  class InternalWrenchController {
  public:
    void control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                 const cnoid::BodyPtr& robot_act,
                 const std::vector<double>& pgain,
                 const std::vector<cnoid::LinkPtr>& useJoints,
                 double dt,
                 int debugLevel,
                 std::unordered_map<std::string, cnoid::Position>& fixedPoseMap //input & output
                 );

    void start(double transitionTime) { startFlag_ = true; transitionTime_ = transitionTime; }
    void reset() {startFlag_ = false; remainTime_ = 0.0; transitionVelocityMap_.clear(); positionConstraintMap_.clear();}
  protected:
    bool calcEEModification(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                            const cnoid::BodyPtr& robot_act,
                            const std::vector<double>& pgain,
                            const std::vector<cnoid::LinkPtr>& useJoints,
                            const std::unordered_map<std::string, cnoid::Position>& fixedPoseMap,
                            int debugLevel);

    static bool appendRow(const std::vector<cnoid::VectorX>& vs, cnoid::VectorX& vout);
    static bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout);
    static bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout);
  protected:
    bool startFlag_ = false;
    double transitionTime_ = 1.0;

    double remainTime_ = 0.0;
    std::unordered_map<std::string, cnoid::Vector6> transitionVelocityMap_; // local frame

    std::unordered_map<std::string, std::shared_ptr<IK::PositionConstraint> > positionConstraintMap_;
    std::shared_ptr<prioritized_qp::Task> rootBalanceTask_;
    std::shared_ptr<prioritized_qp::Task> tauLimitTask_;
    std::shared_ptr<prioritized_qp::Task> wrenchLimitTask_;
    std::shared_ptr<prioritized_qp::Task> wrenchTargetTask_;
    std::shared_ptr<prioritized_qp::Task> tauTargetTask_;
  };
}

#endif
