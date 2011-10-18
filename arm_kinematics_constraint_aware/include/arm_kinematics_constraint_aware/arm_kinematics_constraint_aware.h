/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef ARM_KINEMATICS_CONSTRAINT_AWARE_H
#define ARM_KINEMATICS_CONSTRAINT_AWARE_H

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/DisplayTrajectory.h>
#include <arm_navigation_msgs/LinkPadding.h>

// MISC
#include <planning_environment/models/collision_models_interface.h>
#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware_utils.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <urdf/model.h>

// plugin
#include <pluginlib/class_loader.h>
#include <kinematics_base/kinematics_base.h>


namespace arm_kinematics_constraint_aware
{
class ArmKinematicsConstraintAware
{
public:

  /** @class
   *  @brief ROS/KDL based interface for the inverse kinematics of the PR2 arm
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   *
   *  This class provides a ROS/KDL interface to the inverse kinematics of the PR2 arm. 
   *  It will compute a collision free solution to the inverse kinematics of the PR2 arm. 
   *  The collision environment needs to be active for this method to work. This requires the presence of a node 
   *  that is publishing collision maps. 
   *  To use this node, you must have a roscore running with a robot description available from the ROS param server. 
   */
  ArmKinematicsConstraintAware();

  virtual ~ArmKinematicsConstraintAware()
  {
    if (collision_models_interface_)
      delete collision_models_interface_;
  };

  /**
   * @brief This method searches for and returns the closest solution to the initial guess in the first set of solutions it finds. 
   *
   * @return < 0 if no solution is found
   * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
   * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
   * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
   * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
   * @param timeout The amount of time (in seconds) to spend looking for a solution.
   */
  bool getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request, 
                                    kinematics_msgs::GetConstraintAwarePositionIK::Response &response);

  bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                     kinematics_msgs::GetPositionIK::Response &response);

  bool getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                       kinematics_msgs::GetKinematicSolverInfo::Response &response);

  bool getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                       kinematics_msgs::GetKinematicSolverInfo::Response &response);

  bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                     kinematics_msgs::GetPositionFK::Response &response);

  bool isActive(){ return active_;}
private:

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
  kinematics::KinematicsBase* kinematics_solver_;
  bool active_;

  ros::NodeHandle node_handle_,root_handle_;
  ros::ServiceServer ik_collision_service_, ik_service_, fk_service_, ik_solver_info_service_, fk_solver_info_service_;
  planning_environment::CollisionModelsInterface *collision_models_interface_;
  std::string group_,root_name_;
  bool use_collision_map_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  std::vector<std::string> end_effector_collision_links_;
  std::vector<std::string> arm_links_;
  void collisionCheck(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_solution,
                      int &error_code);
  void initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_solution,
                        int &error_code);
  void printStringVec(const std::string &prefix, const std::vector<std::string> &string_vector);
  ros::Publisher display_trajectory_publisher_;
  bool visualize_solution_;
  kinematics_msgs::PositionIKRequest ik_request_;
  arm_navigation_msgs::Constraints constraints_;

  void advertiseBaseKinematicsServices();
  void advertiseConstraintIKService();

  bool isReady(arm_navigation_msgs::ArmNavigationErrorCodes &error_code);
  void sendEndEffectorPose(const planning_models::KinematicState* state, bool valid);

  kinematics_msgs::KinematicSolverInfo chain_info_;

};
}
#endif
