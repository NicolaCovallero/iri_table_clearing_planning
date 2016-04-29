// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _table_clearing_grasp_execution_alg_node_h_
#define _table_clearing_grasp_execution_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "table_clearing_grasp_execution_alg.h"

// [publisher subscriber headers]
#include <geometry_msgs/PoseStamped.h>

// [service client headers]
#include <iri_wam_common_msgs/QueryWamInverseKinematicsFromPose.h>

// [action server client headers]
#include <iri_common_drivers_msgs/tool_closeAction.h>
#include <iri_common_drivers_msgs/tool_openAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iri_action_server/iri_action_server.h>
#include <iri_table_clearing_common_msgs/GraspObjectAction.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class TableClearingGraspExecutionAlgNode : public algorithm_base::IriBaseAlgorithm<TableClearingGraspExecutionAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher goal_pose_publisher_;
    geometry_msgs::PoseStamped goal_pose_PoseStamped_msg_;


    // [subscriber attributes]

    // [service attributes]

    // [client attributes]
    ros::ServiceClient gripper_ik_from_pose_client_;
    iri_wam_common_msgs::QueryWamInverseKinematicsFromPose gripper_ik_from_pose_srv_;


    // [action server attributes]
    IriActionServer<iri_table_clearing_common_msgs::GraspObjectAction> grasp_action_aserver_;
    void grasp_actionStartCallback(const iri_table_clearing_common_msgs::GraspObjectGoalConstPtr& goal);
    void grasp_actionStopCallback(void);
    bool grasp_actionIsFinishedCallback(void);
    bool grasp_actionHasSucceededCallback(void);
    void grasp_actionGetResultCallback(iri_table_clearing_common_msgs::GraspObjectResultPtr& result);
    void grasp_actionGetFeedbackCallback(iri_table_clearing_common_msgs::GraspObjectFeedbackPtr& feedback);
    bool grasp_action_active;
    bool grasp_action_succeeded;
    bool grasp_action_finished;



    // [action client attributes]
    actionlib::SimpleActionClient<iri_common_drivers_msgs::tool_closeAction> close_tool_client_;
    iri_common_drivers_msgs::tool_closeGoal close_tool_goal_;
    bool close_toolMakeActionRequest();
    void close_toolDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_closeResultConstPtr& result);
    void close_toolActive();
    void close_toolFeedback(const iri_common_drivers_msgs::tool_closeFeedbackConstPtr& feedback);

    actionlib::SimpleActionClient<iri_common_drivers_msgs::tool_openAction> open_tool_client_;
    iri_common_drivers_msgs::tool_openGoal open_tool_goal_;
    bool open_toolMakeActionRequest();
    void open_toolDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_openResultConstPtr& result);
    void open_toolActive();
    void open_toolFeedback(const iri_common_drivers_msgs::tool_openFeedbackConstPtr& feedback);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_client_;
    control_msgs::FollowJointTrajectoryGoal traj_goal_;
    bool trajMakeActionRequest();
    void trajDone(const actionlib::SimpleClientGoalState& state,  const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void trajActive();
    void trajFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);


   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:

    geometry_msgs::PoseStamped grasping_pose;
    geometry_msgs::PoseStamped approaching_pose;
    geometry_msgs::PoseStamped pre_dropping_pose;
    geometry_msgs::PoseStamped dropping_pose;

    /**
     * Joint state
     */
    sensor_msgs::JointState joints_grasping_pose;
    sensor_msgs::JointState joints_approaching_pose;
    sensor_msgs::JointState joints_pre_dropping_pose;
    sensor_msgs::JointState joints_dropping_pose;

    sensor_msgs::JointState home_joint_state;


    /**
     * 
     */
    bool feasible;

    /**
     * We have 11 steps:
     * 1) check feasibility of the path
     * 2) go to pre grasping pose
     * 3) open gripper
     * 4) go to grasping pose
     * 5) close gripper
     * 6) go to pre grasp pose
     * 7) go to pre dropping pose
     * 8) go to dropping pose
     * 9) open gripper
     * 10) close gripper
     * 11) go home
     */
    uint step;

    std::string info;


   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    TableClearingGraspExecutionAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~TableClearingGraspExecutionAlgNode(void);

    void setJointPoseGoal(sensor_msgs::JointState joint_pose);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
