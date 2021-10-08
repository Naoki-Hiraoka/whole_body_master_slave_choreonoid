#ifndef PrimitiveMotionLevelController_PositionControl_H
#define PrimitiveMotionLevelController_PositionControl_H

#include <unordered_map>
#include <cnoid/Body>
#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include "PrimitiveCommand.h"

namespace PrimitiveMotionLevel {

  class PositionController {
  public:
    void reset();
    void control(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap, // primitive motion level target
                 const cnoid::BodyPtr& robot_ref, // command level target
                 cnoid::BodyPtr& robot_com, //output
                 double dt
                 );

    class PositionTask {
    public:
      PositionTask(const std::string& name);
      void updateFromPrimitiveCommand(const std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand>& primitiveCommand) {primitiveCommand_ = primitiveCommand;}
      void calcImpedanceControl(double dt);
      void getIKConstraintsforSupportEEF(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double weight=1.0);
      void getIKConstraintsforInteractEEF(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double weight=1.0);
      void getIKConstraintsforCOM(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double weight=1.0);
      const std::string& name() const { return name_;}
      const std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand>& primitiveCommand() const {return primitiveCommand_;}
    protected:
      static void calcPositionConstraint(const cnoid::BodyPtr& robot_com,
                                         const std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand>& primitiveCommand,
                                         const cnoid::Position& offset,
                                         double weight,
                                         std::shared_ptr<IK::PositionConstraint>& positionConstraint);
      static void calcCOMConstraint(const cnoid::BodyPtr& robot_com,
                                    const std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand>& primitiveCommand,
                                    double weight,
                                    std::shared_ptr<IK::COMConstraint>& comConstraint);

      std::string name_;
      std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand> primitiveCommand_;

      cnoid::Position offset_; // world系
      cnoid::Vector6 dOffsetPrev_; // world系

      std::shared_ptr<IK::PositionConstraint> positionConstraint_;
      std::shared_ptr<IK::COMConstraint> comConstraint_;
    };

    class FullbodyIKSolver {
    public:
      void solveFullbodyIK(cnoid::BodyPtr& robot_com,
                           const cnoid::BodyPtr& robot_ref,
                           const std::map<std::string, std::shared_ptr<PositionTask> >& positionTaskMap_);
    protected:
      cnoid::VectorX jlim_avoid_weight_old_;

      std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> > jointAngleConstraint_;
      std::shared_ptr<IK::PositionConstraint> rootLinkConstraint_;
    };
  protected:
    std::map<std::string, std::shared_ptr<PositionTask> > positionTaskMap_;

    FullbodyIKSolver fullbodyIKSolver_;

    // static functions
    static void getPrimitiveCommand(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<PositionController::PositionTask> >& positionTaskMap);
    static void getCommandLevelIKConstraints(const cnoid::BodyPtr& robot_ref, std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> > jointAngleConstraint, std::shared_ptr<IK::PositionConstraint>& rootLinkConstraint, std::vector<std::shared_ptr<IK::IKConstraint> >& commandLevelIKConstraints, const cnoid::BodyPtr& robot_com, double weight = 1.0);
  };
}

#endif
