#include <ros/ros.h>
#include <primitive_motion_level_msgs/PrimitiveStateArray.h>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>
#include <cnoid/EigenUtil>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <unordered_map>
#include <cpp_filters/TwoPointInterpolator.h>

class MasterController {
public:
  MasterController(){
    ros::NodeHandle nh, pnh("~");

    cnoid::BodyLoader bodyLoader;
    std::string fileName;
    pnh.getParam("master_model",fileName);
    this->master_robot_vrml_ = bodyLoader.load(fileName);
    if(!this->master_robot_vrml_){
      ROS_FATAL_STREAM("failed to load model[" << fileName << "]");
      exit(1);
    }
    pnh.getParam("slave_model",fileName);
    this->slave_robot_vrml_ = bodyLoader.load(fileName);
    if(!this->slave_robot_vrml_){
      ROS_FATAL_STREAM("failed to load model[" << fileName << "]");
      exit(1);
    }

    std::string robot_description;
    pnh.param("master_robot_description",robot_description,std::string("robot_description"));
    this->master_robot_urdf_ = std::make_shared<urdf::Model>();
    this->master_robot_urdf_->initParam(robot_description);
    pnh.getParam("slave_robot_description",robot_description);
    this->slave_robot_urdf_ = std::make_shared<urdf::Model>();
    this->slave_robot_urdf_->initParam(robot_description);

    // localpose, parentlinknameをみる
    masterConfigSub_ = pnh.subscribe("master_config", 1, &MasterController::masterConfigCb, this);
    // pose, wrench以外をみる
    slaveConfigSub_ = pnh.subscribe("slave_config", 1, &MasterController::slaveConfigCb, this);

    masterJointStateSub_ = pnh.subscribe("master_joint_states", 1, &MasterController::masterJointStateCb, this);
    masterOdometrySub_ = pnh.subscribe("master_odom", 1, &MasterController::masterOdometryCb, this);
    slaveJointStateSub_ = pnh.subscribe("slave_joint_states", 1, &MasterController::slaveJointStateCb, this);
    slaveOdometrySub_ = pnh.subscribe("slave_odom", 1, &MasterController::slaveOdometryCb, this);

    slaveCommandPub_ = pnh.advertise<primitive_motion_level_msgs::PrimitiveStateArray>("slave_command",10);

    masterToSlaveFramepInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(), cpp_filters::HOFFARBIB);
    masterToSlaveFrameRInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolatorSO3>(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(), cpp_filters::HOFFARBIB);

    timer_ = nh.createTimer(ros::Duration(1.0 / 500),
                            boost::bind(&MasterController::periodicTimerCallback, this, _1));
  }

  void slaveConfigCb(primitive_motion_level_msgs::PrimitiveStateArray::ConstPtr msg){
    slaveConfigMsg_ = *msg;
    slaveConfig_.updateFromMsg(*msg);
    slaveConfigMsgIdxMap_.clear();
    for(int i=0;i<msg->primitive_state.size();i++) slaveConfigMsgIdxMap_[msg->primitive_state[i].name] = i;
  }

  void masterConfigCb(primitive_motion_level_msgs::PrimitiveStateArray::ConstPtr msg){
    masterConfigMsg_ = *msg;
    masterConfig_.updateFromMsg(*msg);
    masterConfigMsgIdxMap_.clear();
    for(int i=0;i<msg->primitive_state.size();i++) masterConfigMsgIdxMap_[msg->primitive_state[i].name] = i;
  }

  void masterJointStateCb(sensor_msgs::JointState::ConstPtr msg){
    if(msg->name.size() != msg->position.size()) return;
    for(int i=0;i<msg->name.size();i++){
      const cnoid::LinkPtr& link = this->master_robot_vrml_->link(msg->name[i]);
      if(!link) continue;
      link->q() = msg->position[i];
    }
    this->master_robot_vrml_->calcForwardKinematics();
  }

  void masterOdometryCb(nav_msgs::Odometry::ConstPtr msg){
    this->master_robot_vrml_->rootLink()->p()[0] = msg->pose.pose.position.x;
    this->master_robot_vrml_->rootLink()->p()[1] = msg->pose.pose.position.y;
    this->master_robot_vrml_->rootLink()->p()[2] = msg->pose.pose.position.z;
    cnoid::Quaternion q(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    this->master_robot_vrml_->rootLink()->R() = cnoid::Matrix3(q);
    this->master_robot_vrml_->calcForwardKinematics();
  }

  void slaveJointStateCb(sensor_msgs::JointState::ConstPtr msg){
    if(msg->name.size() != msg->position.size()) return;
    for(int i=0;i<msg->name.size();i++){
      const cnoid::LinkPtr& link = this->slave_robot_vrml_->link(msg->name[i]);
      if(!link) continue;
      link->q() = msg->position[i];
    }
    this->slave_robot_vrml_->calcForwardKinematics();
  }

  void slaveOdometryCb(nav_msgs::Odometry::ConstPtr msg){
    this->slave_robot_vrml_->rootLink()->p()[0] = msg->pose.pose.position.x;
    this->slave_robot_vrml_->rootLink()->p()[1] = msg->pose.pose.position.y;
    this->slave_robot_vrml_->rootLink()->p()[2] = msg->pose.pose.position.z;
    cnoid::Quaternion q(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    this->slave_robot_vrml_->rootLink()->R() = cnoid::Matrix3(q);
    this->slave_robot_vrml_->calcForwardKinematics();
  }

  void periodicTimerCallback(const ros::TimerEvent& event){
    // supportCOM=trueのEEFに着目してmasterの座標系とslaveの座標系の対応をとる
    std::vector<cnoid::Position> masterTs;
    std::vector<cnoid::Position> slaveTs;
    for(std::map<std::string,std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it=this->slaveConfig_.primitiveState().begin();it!=this->slaveConfig_.primitiveState().end();it++){
      if(this->masterConfigMsgIdxMap_.find(it->first) == this->masterConfigMsgIdxMap_.end()) continue;
      if(!it->second->supportCOM()) continue;
      cnoid::LinkPtr slaveLink = this->URDFToVRMLslave(it->second->parentLinkName());
      if(!slaveLink) continue;
      cnoid::Position slaveT = slaveLink->T() * it->second->localPose();
      cnoid::LinkPtr masterLink = this->URDFToVRMLmaster(this->masterConfig_.primitiveState()[it->first]->parentLinkName());
      if(!masterLink) continue;
      cnoid::Position masterT = masterLink->T() * this->masterConfig_.primitiveState()[it->first]->localPose();
      masterTs.push_back(masterT);
      slaveTs.push_back(slaveT);
    }
    if(masterTs.size() != 0){
      cnoid::Vector3 p;
      masterToSlaveFramepInterpolator_->get(p);
      cnoid::Matrix3 R;
      masterToSlaveFrameRInterpolator_->get(R);
      cnoid::Position masterToSlaveFrame;
      masterToSlaveFrame.translation() = p;
      masterToSlaveFrame.linear() = R;
      for(size_t loop=0;loop<3;loop++){
        // calc error, jacobian
        cnoid::VectorX error = cnoid::VectorX::Zero(masterTs.size()*4);
        Eigen::SparseMatrix<double,Eigen::RowMajor> J(masterTs.size()*4,4);
        Eigen::SparseMatrix<double,Eigen::RowMajor> W(masterTs.size()*4,masterTs.size()*4);
        for(size_t i=0;i<masterTs.size();i++){
          const cnoid::Position& slaveT = slaveTs[i];
          const cnoid::Position& masterT = masterToSlaveFrame * masterTs[i]; // slave座標系に揃える

          error.segment<3>(4*i) = slaveT.translation() - masterT.translation();
          cnoid::Matrix3 diffR = slaveT.linear() * masterT.linear().transpose();
          cnoid::Matrix3 horizontal_diffR;
          {
            cnoid::Vector3 localZ = diffR * cnoid::Vector3::UnitZ();
            cnoid::Vector3 cross = localZ.cross(cnoid::Vector3::UnitZ());
            double angle = std::acos(localZ.dot(cnoid::Vector3::UnitZ())); // 0~pi
            cnoid::Vector3 axis = (cross.norm()!=0.0) ? cross.normalized() : cnoid::Vector3::UnitX();// include sign
            horizontal_diffR = Eigen::AngleAxisd(angle,axis).toRotationMatrix() * diffR;
          }
          error[4*i+3] = cnoid::rpyFromRot(horizontal_diffR)[2];

          cnoid::Vector3 dp = cnoid::Vector3::UnitZ().cross(masterT.translation() - masterToSlaveFrame.translation());
          J.coeffRef(4*i+0,0) = 1.0; J.coeffRef(4*i+0,3) = dp[0];
          J.coeffRef(4*i+1,1) = 1.0; J.coeffRef(4*i+1,3) = dp[1];
          J.coeffRef(4*i+2,2) = 1.0; J.coeffRef(4*i+2,3) = dp[2];
          J.coeffRef(4*i+3,3) = 1.0;
        }

        // solve
        for(size_t i=0;i<masterTs.size();i++){
          for(int j=0;j<3;j++) W.coeffRef(i*4+j,i*4+j) = 1.0;
          W.coeffRef(i*4+3,i*4+3) = 0.01;//yawの寄与を小さく
        }
        // d_origin = (Jt W J)^-1 Jt W error
        // <=>
        // (Jt W J) d_origin = Jt W error
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
        solver.compute(J.transpose() * W * J);
        if(solver.info()==Eigen::Success) {
          cnoid::Vector4 d_origin = solver.solve(J.transpose() * W * error);
          if(solver.info()==Eigen::Success) {
            // apply result
            masterToSlaveFrame.translation() += d_origin.head<3>();
            masterToSlaveFrame.linear() = cnoid::rotFromRpy(0,0,cnoid::rpyFromRot(masterToSlaveFrame.linear())[2] + d_origin[3]);
          }
        }
      }
      masterToSlaveFramepInterpolator_->setGoal(masterToSlaveFrame.translation(), 1.0);
      masterToSlaveFrameRInterpolator_->setGoal(masterToSlaveFrame.linear(), 1.0);
    }

    // 実際の変換を求める
    double dt = (event.current_real - event.last_real).toSec();
    cnoid::Vector3 p;
    masterToSlaveFramepInterpolator_->get(p, dt);
    cnoid::Matrix3 R;
    masterToSlaveFrameRInterpolator_->get(R, dt);
    cnoid::Position masterToSlaveFrame;
    masterToSlaveFrame.translation() = p;
    masterToSlaveFrame.linear() = R;

    // publish
    primitive_motion_level_msgs::PrimitiveStateArray msg;
    for(std::map<std::string,std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it=this->slaveConfig_.primitiveState().begin();it!=this->slaveConfig_.primitiveState().end();it++){
      if(this->masterConfigMsgIdxMap_.find(it->first) == this->masterConfigMsgIdxMap_.end()) continue;
      cnoid::LinkPtr masterLink = this->URDFToVRMLmaster(this->masterConfig_.primitiveState()[it->first]->parentLinkName());
      if(!masterLink) continue;
      cnoid::Position masterT = masterLink->T() * this->masterConfig_.primitiveState()[it->first]->localPose();
      cnoid::Position targetT = masterToSlaveFrame * masterT;

      primitive_motion_level_msgs::PrimitiveState out = this->slaveConfigMsg_.primitive_state[this->slaveConfigMsgIdxMap_[it->first]];
      out.pose.position.x = targetT.translation()[0];
      out.pose.position.y = targetT.translation()[1];
      out.pose.position.z = targetT.translation()[2];
      cnoid::Quaternion q(targetT.linear());
      out.pose.orientation.w = q.w();
      out.pose.orientation.x = q.x();
      out.pose.orientation.y = q.y();
      out.pose.orientation.z = q.z();

      out.time = dt * 5; // なんとなく
      msg.primitive_state.push_back(out);
    }
    slaveCommandPub_.publish(msg);
  }

  cnoid::LinkPtr URDFToVRMLmaster(const std::string& URDFLinkName){
    std::shared_ptr<const urdf::Link> link = this->master_robot_urdf_->getLink(URDFLinkName);
    if(link){
      if(link->parent_joint){
        return this->master_robot_vrml_->link(link->parent_joint->name);
      }else if (link == this->master_robot_urdf_->getRoot()){
        return this->master_robot_vrml_->rootLink();
      }
    }
    ROS_ERROR_STREAM("failed to find link [" << URDFLinkName << "]");
    return nullptr;
  };
  cnoid::LinkPtr URDFToVRMLslave(const std::string& URDFLinkName){
    std::shared_ptr<const urdf::Link> link = this->slave_robot_urdf_->getLink(URDFLinkName);
    if(link){
      if(link->parent_joint){
        return this->slave_robot_vrml_->link(link->parent_joint->name);
      }else if (link == this->slave_robot_urdf_->getRoot()){
        return this->slave_robot_vrml_->rootLink();
      }
    }
    ROS_ERROR_STREAM("failed to find link [" << URDFLinkName << "]");
    return nullptr;
  };

protected:
  ros::Subscriber slaveConfigSub_;
  ros::Subscriber masterConfigSub_;

  primitive_motion_level_msgs::PrimitiveStateArray slaveConfigMsg_;
  std::unordered_map<std::string, int> slaveConfigMsgIdxMap_;
  primitive_motion_level_tools::PrimitiveStates slaveConfig_;
  primitive_motion_level_msgs::PrimitiveStateArray masterConfigMsg_;
  std::unordered_map<std::string, int> masterConfigMsgIdxMap_;
  primitive_motion_level_tools::PrimitiveStates masterConfig_;
  std::unordered_map<std::string,cnoid::ForceSensorPtr> pairedSensorMap_;

  ros::Subscriber slaveJointStateSub_;
  ros::Subscriber slaveOdometrySub_;
  ros::Subscriber masterJointStateSub_;
  ros::Subscriber masterOdometrySub_;

  ros::Publisher slaveCommandPub_;
  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > masterToSlaveFramepInterpolator_;
  std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3 > masterToSlaveFrameRInterpolator_;

  std::shared_ptr<urdf::Model> slave_robot_urdf_;
  cnoid::BodyPtr slave_robot_vrml_;
  std::shared_ptr<urdf::Model> master_robot_urdf_;
  cnoid::BodyPtr master_robot_vrml_;

  ros::Timer timer_;
};

int main (int argc, char** argv){
  ros::init(argc, argv, "master_controller");
  ros::NodeHandle nh;
  MasterController mc;
  ros::spin();
  return 0;
}
