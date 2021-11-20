#include <ros/ros.h>
#include <primitive_motion_level_msgs/PrimitiveStateArray.h>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <unordered_map>

class MasterController {
public:
  MasterController(){
    ros::NodeHandle nh, pnh("~");

    cnoid::BodyLoader bodyLoader;
    std::string fileName;
    pnh.getParam("slave_model",fileName);
    this->robot_vrml_ = bodyLoader.load(fileName);
    if(!this->robot_vrml_){
      ROS_FATAL_STREAM("failed to load model[" << fileName << "]");
      exit(1);
    }

    std::string slave_robot_description;
    pnh.getParam("slave_robot_description",slave_robot_description);

    this->robot_urdf_ = std::make_shared<urdf::Model>();
    this->robot_urdf_->initParam(slave_robot_description);

    // localpose, parentlinkname, supportCOMをみる
    slaveConfigSub_ = pnh.subscribe("slave_config", 1, &MasterController::slaveConfigCb, this);
    // pose, wrench以外をみる
    masterConfigSub_ = pnh.subscribe("master_config", 1, &MasterController::masterConfigCb, this);

    slaveJointStateSub_ = pnh.subscribe("slave_joint_states", 1, &MasterController::slaveJointStateCb, this);
    slaveOdometrySub_ = pnh.subscribe("slave_odom", 1, &MasterController::slaveOdometryCb, this);

    {
      cnoid::DeviceList<cnoid::ForceSensor> fsensors = this->robot_vrml_->devices<cnoid::ForceSensor>();
      for(int i=0;i<fsensors.size();i++){
        this->slaveForceSensorSubs_.push_back(pnh.subscribe(fsensors[i]->name(),1,&MasterController::slaveForceSensorCb, this));
      }
    }

    masterCommandPub_ = pnh.advertise<primitive_motion_level_msgs::PrimitiveStateArray>("master_command",10);
    masterCommandAllPub_ = pnh.advertise<primitive_motion_level_msgs::PrimitiveStateArray>("master_command_all",10);

    std::map<std::string,std::string> pairedSensorStrMap;
    pnh.getParam("paired_sensor",pairedSensorStrMap);
    for(std::map<std::string,std::string>::iterator it=pairedSensorStrMap.begin();it!=pairedSensorStrMap.end();it++){
      cnoid::ForceSensorPtr fsensor = this->robot_vrml_->findDevice<cnoid::ForceSensor>(it->second);
      if(fsensor) this->pairedSensorMap_[it->first] = fsensor;
    }

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

  void slaveJointStateCb(sensor_msgs::JointState::ConstPtr msg){
    if(msg->name.size() != msg->position.size()) return;
    for(int i=0;i<msg->name.size();i++){
      const cnoid::LinkPtr& link = this->robot_vrml_->link(msg->name[i]);
      if(!link) continue;
      link->q() = msg->position[i];
    }
    this->robot_vrml_->calcForwardKinematics();
  }

  void slaveOdometryCb(nav_msgs::Odometry::ConstPtr msg){
    this->robot_vrml_->rootLink()->p()[0] = msg->pose.pose.position.x;
    this->robot_vrml_->rootLink()->p()[1] = msg->pose.pose.position.y;
    this->robot_vrml_->rootLink()->p()[2] = msg->pose.pose.position.z;
    cnoid::Quaternion q(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    this->robot_vrml_->rootLink()->R() = cnoid::Matrix3(q);
    this->robot_vrml_->calcForwardKinematics();
  }

  void slaveForceSensorCb(geometry_msgs::WrenchStamped::ConstPtr msg){
    cnoid::ForceSensorPtr sensor = this->robot_vrml_->findDevice<cnoid::ForceSensor>(msg->header.frame_id);
    if(!sensor) return;
    sensor->F()[0] = msg->wrench.force.x;
    sensor->F()[1] = msg->wrench.force.y;
    sensor->F()[2] = msg->wrench.force.z;
    sensor->F()[3] = msg->wrench.torque.x;
    sensor->F()[4] = msg->wrench.torque.y;
    sensor->F()[5] = msg->wrench.torque.z;
  }

  void periodicTimerCallback(const ros::TimerEvent& event){
    primitive_motion_level_msgs::PrimitiveStateArray msg_support;
    primitive_motion_level_msgs::PrimitiveStateArray msg_all;
    double dt = (event.current_real - event.last_real).toSec();
    for(std::map<std::string,std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it=this->slaveConfig_.primitiveState().begin();it!=this->slaveConfig_.primitiveState().end();it++){
      if(this->masterConfigMsgIdxMap_.find(it->first) == this->masterConfigMsgIdxMap_.end()) continue;
      cnoid::LinkPtr link = this->URDFToVRML(it->second->parentLinkName());
      if(!link) continue;
      cnoid::Position pose = link->T() * it->second->localPose();
      cnoid::Vector6 wrench = cnoid::Vector6::Zero();
      if(this->pairedSensorMap_.find(it->first) != this->pairedSensorMap_.end()){
        cnoid::ForceSensorPtr sensor = this->pairedSensorMap_[it->first];
        cnoid::Position sensorT = sensor->link()->T() * sensor->T_local();
        cnoid::Position eef2SensorT = pose.inverse() * sensorT;
        cnoid::Vector6 wrenchlocal;
        wrenchlocal.head<3>() = eef2SensorT.linear() * -sensor->F().head<3>(); // sensorはロボットが受ける力. masterに反映する際には符号反転
        wrenchlocal.tail<3>() = eef2SensorT.linear() * -sensor->F().tail<3>();
        wrenchlocal.tail<3>() += eef2SensorT.translation().cross(wrenchlocal.head<3>());
        wrench.head<3>() = pose.linear() * wrenchlocal.head<3>();
        wrench.tail<3>() = pose.linear() * wrenchlocal.tail<3>();
      }

      primitive_motion_level_msgs::PrimitiveState out = this->masterConfigMsg_.primitive_state[this->masterConfigMsgIdxMap_[it->first]];
      out.pose.position.x = pose.translation()[0];
      out.pose.position.y = pose.translation()[1];
      out.pose.position.z = pose.translation()[2];
      cnoid::Quaternion q(pose.linear());
      out.pose.orientation.w = q.w();
      out.pose.orientation.x = q.x();
      out.pose.orientation.y = q.y();
      out.pose.orientation.z = q.z();
      out.wrench.resize(6);
      for(int i=0;i<6;i++) out.wrench[i] = wrench[i];

      out.pose_follow_gain.resize(6);
      out.wrench_follow_gain.resize(6);
      if(it->second->supportCOM()){
        for(int i=0;i<6;i++) out.pose_follow_gain[i] *= 1.0;
        for(int i=0;i<6;i++) out.wrench_follow_gain[i] *= 0.0;
      }else{
        for(int i=0;i<6;i++) out.pose_follow_gain[i] *= 0.0;
        for(int i=0;i<6;i++) out.wrench_follow_gain[i] *= 1.0;
      }

      out.time = dt * 3; // なんとなく

      msg_support.primitive_state.push_back(out);

      out.pose_follow_gain = this->masterConfigMsg_.primitive_state[this->masterConfigMsgIdxMap_[it->first]].pose_follow_gain;
      for(int i=0;i<6;i++) out.wrench_follow_gain[i] *= 0.0;
      msg_all.primitive_state.push_back(out);

    }
    masterCommandPub_.publish(msg_support);
    masterCommandAllPub_.publish(msg_all);
  }

  cnoid::LinkPtr URDFToVRML(const std::string& URDFLinkName){
    urdf::LinkConstSharedPtr link = this->robot_urdf_->getLink(URDFLinkName);
    if(link){
      if(link->parent_joint){
        return this->robot_vrml_->link(link->parent_joint->name);
      }else if (link == this->robot_urdf_->getRoot()){
        return this->robot_vrml_->rootLink();
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
  std::vector<ros::Subscriber> slaveForceSensorSubs_;

  ros::Publisher masterCommandPub_;
  ros::Publisher masterCommandAllPub_;

  std::shared_ptr<urdf::Model> robot_urdf_;
  cnoid::BodyPtr robot_vrml_;

  ros::Timer timer_;
};

int main (int argc, char** argv){
  ros::init(argc, argv, "master_controller");
  ros::NodeHandle nh;
  MasterController mc;
  ros::spin();
  return 0;
}
