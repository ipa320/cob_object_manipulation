/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware.h>

#include <planning_environment/models/model_utils.h>
#include <sensor_msgs/JointState.h>
#include <kinematics_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace arm_kinematics_constraint_aware {

static const std::string IK_WITH_COLLISION_SERVICE = "get_constraint_aware_ik";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";
static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const double IK_DEFAULT_TIMEOUT = 10.0;

ArmKinematicsConstraintAware::ArmKinematicsConstraintAware(): kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),node_handle_("~")
{
  std::string group_name, kinematics_solver_name;
  node_handle_.param<bool>("visualize_solution",visualize_solution_,true);
  node_handle_.param<std::string>("group", group_, std::string());
  node_handle_.param<std::string>("kinematics_solver",kinematics_solver_name," ");
  ROS_INFO("Using kinematics solver name: %s",kinematics_solver_name.c_str());
  if (group_.empty())
  {
    ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to monitor, node cannot compute collision aware inverse kinematics");
    active_ = false;
    return;
  }

  kinematics_solver_ = NULL;
  try
  {
    kinematics_solver_ = kinematics_loader_.createClassInstance(kinematics_solver_name);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
    active_ = false;
    return;
  }
  if(kinematics_solver_->initialize(group_))
    active_ = true;
  else
  {
    active_ = false;
    return;
  }
  root_name_ = kinematics_solver_->getBaseFrame();
  if(!getChainInfo(group_,chain_info_))
  {
    ROS_ERROR("Could not construct chain info for group %s",group_.c_str());
    return;
  }
  collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");
  if(group_.empty()) {
    ROS_WARN("Must specify planning group in configuration");
  }
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(group_);
  if(joint_model_group == NULL) {
    ROS_WARN_STREAM("No joint group " << group_);
  }
  arm_links_ = joint_model_group->getGroupLinkNames();

  const planning_models::KinematicModel::LinkModel* end_effector_link = collision_models_interface_->getKinematicModel()->getLinkModel(chain_info_.link_names.back());
  end_effector_collision_links_ = collision_models_interface_->getKinematicModel()->getChildLinkModelNames(end_effector_link);

  advertiseBaseKinematicsServices();
  advertiseConstraintIKService();
}

void ArmKinematicsConstraintAware::advertiseBaseKinematicsServices()
{
  ik_service_ = node_handle_.advertiseService(IK_SERVICE,&ArmKinematicsConstraintAware::getPositionIK,this);
  fk_service_ = node_handle_.advertiseService(FK_SERVICE,&ArmKinematicsConstraintAware::getPositionFK,this);
  ik_solver_info_service_ = node_handle_.advertiseService(IK_INFO_SERVICE,&ArmKinematicsConstraintAware::getIKSolverInfo,this);
  fk_solver_info_service_ = node_handle_.advertiseService(FK_INFO_SERVICE,&ArmKinematicsConstraintAware::getFKSolverInfo,this);
}

void ArmKinematicsConstraintAware::advertiseConstraintIKService()
{
  ik_collision_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_SERVICE,&ArmKinematicsConstraintAware::getConstraintAwarePositionIK,this);
  display_trajectory_publisher_ = root_handle_.advertise<arm_navigation_msgs::DisplayTrajectory>("ik_solution_display", 1);
}

bool ArmKinematicsConstraintAware::isReady(arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  if(!active_)
  {
    ROS_ERROR("IK service is not ready");
    return false;
  }
  if(!collision_models_interface_->isPlanningSceneSet()) {
    ROS_WARN("Planning scene not set");
    error_code.val = error_code.COLLISION_CHECKING_UNAVAILABLE;
    return false;
  } 
  error_code.val = error_code.SUCCESS;
  return true;
}

bool ArmKinematicsConstraintAware::getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request_in,
                                                                kinematics_msgs::GetConstraintAwarePositionIK::Response &response)
{
  if(!isReady(response.error_code))
    return true;

  if(!checkConstraintAwareIKService(request_in,response,chain_info_))
  {
    ROS_ERROR("IK service request is malformed");
    return true;
  }

  collision_models_interface_->disableCollisionsForNonUpdatedLinks(group_);

  ros::Time start_time = ros::Time::now();
  ROS_DEBUG("Received IK request is in the frame: %s",request_in.ik_request.pose_stamped.header.frame_id.c_str());

  ik_request_ = request_in.ik_request;
  constraints_ = request_in.constraints;

  geometry_msgs::PoseStamped pose_msg_in = ik_request_.pose_stamped;
  geometry_msgs::PoseStamped pose_msg_out;
  planning_environment::setRobotStateAndComputeTransforms(request_in.ik_request.robot_state, *collision_models_interface_->getPlanningSceneState());
  
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  root_name_,
                                                                  pose_msg_in.header,
                                                                  pose_msg_in.pose,
                                                                  pose_msg_out)) {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }  
  ik_request_.pose_stamped = pose_msg_out;
  ROS_DEBUG_STREAM("Pose is " << pose_msg_out.pose.position.x << " " << pose_msg_out.pose.position.y << " " << pose_msg_out.pose.position.z);
  ROS_DEBUG("Transformed IK request is in the frame: %s",ik_request_.pose_stamped.header.frame_id.c_str());
  arm_kinematics_constraint_aware::reorderJointState(ik_request_.ik_seed_state.joint_state,chain_info_);

  ros::Time ik_solver_time = ros::Time::now();
  int kinematics_error_code;
  bool ik_valid = kinematics_solver_->searchPositionIK(ik_request_.pose_stamped.pose,
                                                       ik_request_.ik_seed_state.joint_state.position,
                                                       request_in.timeout.toSec(),
                                                       response.solution.joint_state.position,
                                                       boost::bind(&ArmKinematicsConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                       boost::bind(&ArmKinematicsConstraintAware::collisionCheck, this, _1, _2, _3),kinematics_error_code);
  ROS_DEBUG("IK solver time: %f",(ros::Time::now()-ik_solver_time).toSec());

  if(ik_valid)
  {
    response.solution.joint_state.name = chain_info_.joint_names;
    /*
      if(visualize_solution_)
      {
      arm_navigation_msgs::DisplayTrajectory display_trajectory;
      display_trajectory.trajectory.joint_trajectory.points.resize(1);
      display_trajectory.trajectory.joint_trajectory.points[0].positions = response.solution.joint_state.position;
      display_trajectory.trajectory.joint_trajectory.joint_names = response.solution.joint_state.name;
      planning_monitor_->convertKinematicStateToRobotState(*kinematic_state_,display_trajectory.robot_state);
      display_trajectory_publisher_.publish(display_trajectory);
      }
    */
    kinematics_msgs::GetPositionFK::Request req; 
    kinematics_msgs::GetPositionFK::Response res;
    req.header = pose_msg_out.header;
    req.robot_state.joint_state = response.solution.joint_state;
    req.fk_link_names.push_back(request_in.ik_request.ik_link_name);
    getPositionFK(req,res);
    ROS_DEBUG_STREAM("Fk says " << res.pose_stamped[0].pose.position.x << " " << res.pose_stamped[0].pose.position.y << " " << res.pose_stamped[0].pose.position.z);

    response.error_code.val = response.error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("A collision aware ik solution could not be found");
    response.error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);
    if(response.error_code.val != response.error_code.IK_LINK_IN_COLLISION) 
    {
      sendEndEffectorPose(collision_models_interface_->getPlanningSceneState(),true);
    }
    return true;
  }
}

void ArmKinematicsConstraintAware::collisionCheck(const geometry_msgs::Pose &ik_pose,
                                                  const std::vector<double> &ik_solution,
                                                  int &error_code)
{
  std::map<std::string, double> joint_values;
  for(unsigned int i=0; i < chain_info_.joint_names.size(); i++)
    joint_values[chain_info_.joint_names[i]] = ik_solution[i];

  collision_models_interface_->getPlanningSceneState()->setKinematicState(joint_values);
  if(collision_models_interface_->getPlanningSceneState() == NULL) {
    ROS_INFO_STREAM("Messed up");
  }
  if(collision_models_interface_->isKinematicStateInCollision(*(collision_models_interface_->getPlanningSceneState()))) {
    //TODO - broadcast collisions
    error_code = kinematics::STATE_IN_COLLISION;
  } else {
    error_code = kinematics::SUCCESS;
  }

  if(!planning_environment::doesKinematicStateObeyConstraints(*(collision_models_interface_->getPlanningSceneState()), 
                                                              constraints_)) {
    error_code = kinematics::GOAL_CONSTRAINTS_VIOLATED;
  }
}

void ArmKinematicsConstraintAware::initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_solution,
                                                    int &error_code)
{
  std::string kinematic_frame_id = kinematics_solver_->getBaseFrame();
  std::string planning_frame_id = collision_models_interface_->getWorldFrameId();
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = ik_pose;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = kinematic_frame_id;
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  planning_frame_id,
                                                                  pose_stamped.header,
                                                                  pose_stamped.pose,
                                                                  pose_stamped)) {
    ROS_ERROR_STREAM("Cannot transform from " << pose_stamped.header.frame_id << " to " << planning_frame_id);
  }
  //disabling all collision for arm links
  collision_space::EnvironmentModel::AllowedCollisionMatrix save_acm = collision_models_interface_->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = save_acm;
  for(unsigned int i = 0; i < arm_links_.size(); i++) {
    acm.changeEntry(arm_links_[i], true);
  }
  collision_models_interface_->setAlteredAllowedCollisionMatrix(acm);

  btTransform transform;
  tf::poseMsgToTF(pose_stamped.pose,transform);
  if(!collision_models_interface_->getPlanningSceneState()->hasLinkState(ik_request_.ik_link_name)) {
    ROS_ERROR("Could not find end effector root_link %s", ik_request_.ik_link_name.c_str());
    error_code = kinematics::INVALID_LINK_NAME;
    return;
  }
  collision_models_interface_->getPlanningSceneState()->updateKinematicStateWithLinkAt(ik_request_.ik_link_name, transform);
  if(collision_models_interface_->isKinematicStateInCollision(*(collision_models_interface_->getPlanningSceneState()))) {
    error_code = kinematics::IK_LINK_IN_COLLISION;
    ROS_DEBUG_STREAM("Initial pose check failing");
    sendEndEffectorPose(collision_models_interface_->getPlanningSceneState(), false);
  }
  else
    error_code = kinematics::SUCCESS;
    
  collision_models_interface_->setAlteredAllowedCollisionMatrix(save_acm);
}

void ArmKinematicsConstraintAware::sendEndEffectorPose(const planning_models::KinematicState* state, bool valid) {
 boost::shared_ptr<urdf::Model> robot_model = collision_models_interface_->getParsedDescription();
  visualization_msgs::MarkerArray hand_array;
  unsigned int id = 0;
  for(unsigned int i = 0; i < end_effector_collision_links_.size(); i++) {
    boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(end_effector_collision_links_[i]);
    if(urdf_link == NULL) {
      ROS_DEBUG_STREAM("No entry in urdf for link " << end_effector_collision_links_[i]);
      continue;
    }
    if(!urdf_link->collision) {
      continue;
    }
    const urdf::Geometry *geom = urdf_link->collision->geometry.get();
    if(!geom) {
      ROS_DEBUG_STREAM("No collision geometry for link " << end_effector_collision_links_[i]);
      continue;
    }
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
    if(mesh) {
      if (!mesh->filename.empty()) {
        planning_models::KinematicState::LinkState* ls = state->getLinkState(end_effector_collision_links_[i]);
        visualization_msgs::Marker mark;
        mark.header.frame_id = collision_models_interface_->getWorldFrameId();
        mark.header.stamp = ros::Time::now();
        mark.id = id++;
        if(!valid) {
          mark.ns = "initial_pose_collision";
        } else {
          mark.ns = "initial_pose_ok";
        }
        mark.type = mark.MESH_RESOURCE;
        mark.scale.x = 1.0;
        mark.scale.y = 1.0;
        mark.scale.z = 1.0;
        if(!valid) {
          mark.color.r = 1.0;
        } else {
          mark.color.g = 1.0;
        }
        mark.color.a = .8;
        mark.pose.position.x = ls->getGlobalCollisionBodyTransform().getOrigin().x();
        mark.pose.position.y = ls->getGlobalCollisionBodyTransform().getOrigin().y();
        mark.pose.position.z = ls->getGlobalCollisionBodyTransform().getOrigin().z();
        mark.pose.orientation.x = ls->getGlobalCollisionBodyTransform().getRotation().x();
        mark.pose.orientation.y = ls->getGlobalCollisionBodyTransform().getRotation().y();
        mark.pose.orientation.z = ls->getGlobalCollisionBodyTransform().getRotation().z();
        mark.pose.orientation.w = ls->getGlobalCollisionBodyTransform().getRotation().w();
        mark.mesh_resource = mesh->filename;
        hand_array.markers.push_back(mark);
      }
    }
  }
  //vis_marker_array_publisher_.publish(hand_array);
}

void ArmKinematicsConstraintAware::printStringVec(const std::string &prefix, const std::vector<std::string> &string_vector)
{
  ROS_DEBUG("%s",prefix.c_str());
  for(unsigned int i=0; i < string_vector.size(); i++)
  {
    ROS_DEBUG("%s",string_vector[i].c_str());
  }
}

bool ArmKinematicsConstraintAware::getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                                                 kinematics_msgs::GetPositionIK::Response &response)
{
  bool scene_set = collision_models_interface_->isPlanningSceneSet();
  if(!isReady(response.error_code)) {
    if(request.ik_request.pose_stamped.header.frame_id != root_name_) {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }
  }

  if(!checkIKService(request,response,chain_info_))
    return true;

  geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
  geometry_msgs::PoseStamped pose_msg_out;
  if(scene_set) {
    planning_environment::setRobotStateAndComputeTransforms(request.ik_request.robot_state, *collision_models_interface_->getPlanningSceneState());
    
    if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                    root_name_,
                                                                    pose_msg_in.header,
                                                                    pose_msg_in.pose,
                                                                    pose_msg_out)) {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }  
  } else {
    pose_msg_out = pose_msg_in;
  }
  request.ik_request.pose_stamped = pose_msg_out;
  ROS_DEBUG_STREAM("Pose is " << pose_msg_out.pose.position.x << " " << pose_msg_out.pose.position.y << " " << pose_msg_out.pose.position.z);

  arm_kinematics_constraint_aware::reorderJointState(request.ik_request.ik_seed_state.joint_state,chain_info_);

  int kinematics_error_code;
  bool ik_valid = kinematics_solver_->searchPositionIK(pose_msg_out.pose,
                                                      request.ik_request.ik_seed_state.joint_state.position,
                                                      request.timeout.toSec(),
                                                      response.solution.joint_state.position,
                                                      kinematics_error_code);

  response.error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);

  if(ik_valid)
  {
    response.solution.joint_state.name = chain_info_.joint_names;
    response.error_code.val = response.error_code.SUCCESS;
    kinematics_msgs::GetPositionFK::Request req; 
    kinematics_msgs::GetPositionFK::Response res;
    req.header = request.ik_request.pose_stamped.header;
    req.robot_state.joint_state = response.solution.joint_state;
    req.fk_link_names.push_back(request.ik_request.ik_link_name);
    getPositionFK(req,res);
    ROS_DEBUG_STREAM("Fk says " << res.pose_stamped[0].pose.position.x << " " << res.pose_stamped[0].pose.position.y << " " << res.pose_stamped[0].pose.position.z);
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    return true;
  }
}

bool ArmKinematicsConstraintAware::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK node not active");
    return true;
  }
  response.kinematic_solver_info = chain_info_;
  return true;
}

bool ArmKinematicsConstraintAware::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK node not active");
    return true;
  }
  response.kinematic_solver_info = chain_info_;
  return true;
}

bool ArmKinematicsConstraintAware::getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                                                   kinematics_msgs::GetPositionFK::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("FK service not active");
    return true;
  }

  if(!checkFKService(request,response,chain_info_))
    return true;

  arm_kinematics_constraint_aware::reorderJointState(request.robot_state.joint_state,chain_info_);

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());

  bool valid = true;
  std::vector<geometry_msgs::Pose> solutions;
  solutions.resize(request.fk_link_names.size());
  if(kinematics_solver_->getPositionFK(request.fk_link_names,request.robot_state.joint_state.position,solutions) >=0)
  {    
    for(unsigned int i=0; i < solutions.size(); i++)
    {      
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose = solutions[i];
      pose_stamped.header.frame_id = root_name_;
      pose_stamped.header.stamp = ros::Time();
      
      if(!collision_models_interface_->isPlanningSceneSet()) {
        if(request.header.frame_id != root_name_) {
          response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
          return true;
        }
      } else if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                      request.header.frame_id,
                                                                      pose_stamped.header,
                                                                      solutions[i],
                                                                      pose_stamped)) {
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        return true;
      }
      response.pose_stamped[i] = pose_stamped;
      response.fk_link_names[i] = request.fk_link_names[i];
      response.error_code.val = response.error_code.SUCCESS;
    }
  }
  else
  {
    ROS_ERROR("Could not compute FK");
    response.error_code.val = response.error_code.NO_FK_SOLUTION;
    valid = false;
  }
  return valid;
}

} // namespace

