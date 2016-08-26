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

#ifndef _table_clearing_execute_alg_node_h_
#define _table_clearing_execute_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "table_clearing_execute_alg.h"
#include <ctime>

// [publisher subscriber headers]
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

// [service client headers]
#include <iri_table_clearing_gazebo/DeleteObject.h>
#include <iri_common_drivers_msgs/QueryInverseKinematics.h>
#include <iri_wam_common_msgs/QueryWamInverseKinematicsFromPose.h>
#include <iri_common_drivers_msgs/QueryJointsMovement.h>
#include <iri_table_clearing_execute/ExecuteGrasping.h>
#include <iri_table_clearing_execute/ExecutePushing.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>

#include <iri_common_drivers_msgs/tool_closeAction.h>
#include <iri_common_drivers_msgs/tool_openAction.h>

// [action server client headers]
#include <actionlib/client/terminal_state.h>





/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class TableClearingExecuteAlgNode : public algorithm_base::IriBaseAlgorithm<TableClearingExecuteAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher current_joint_state_publisher_;

    ros::Publisher action_pose_publisher_;
    geometry_msgs::PoseStamped action_pose_PoseStamped_msg_;


    // [subscriber attributes]
    ros::Subscriber current_joint_state_subscriber_;
    void current_joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    pthread_mutex_t current_joint_state_mutex_;
    void current_joint_state_mutex_enter(void);
    void current_joint_state_mutex_exit(void);


    // [service attributes]
    ros::ServiceServer execute_grasping_server_;
    bool execute_graspingCallback(iri_table_clearing_execute::ExecuteGrasping::Request &req, iri_table_clearing_execute::ExecuteGrasping::Response &res);
    pthread_mutex_t execute_grasping_mutex_;
    void execute_grasping_mutex_enter(void);
    void execute_grasping_mutex_exit(void);

    ros::ServiceServer execute_pushing_server_;
    bool execute_pushingCallback(iri_table_clearing_execute::ExecutePushing::Request &req, iri_table_clearing_execute::ExecutePushing::Response &res);
    pthread_mutex_t execute_pushing_mutex_;
    void execute_pushing_mutex_enter(void);
    void execute_pushing_mutex_exit(void);


    // [client attributes]
    ros::ServiceClient remove_object_client_;
    iri_table_clearing_gazebo::DeleteObject remove_object_srv_;

    ros::ServiceClient move_joints_client_;
    iri_common_drivers_msgs::QueryJointsMovement move_joints_srv_;

    ros::ServiceClient estirabot_gripper_ik_client_;
    iri_common_drivers_msgs::QueryInverseKinematics estirabot_gripper_ik_srv_;

    ros::ServiceClient estirabot_gripper_ik_from_pose_client_;
    iri_common_drivers_msgs::QueryInverseKinematics estirabot_gripper_ik_from_pose_srv_;
    TrajClient* traj_client_; 


    // [action server attributes]

    // [action client attributes]
    actionlib::SimpleActionClient<iri_common_drivers_msgs::tool_closeAction> close_gripper_client_;
    iri_common_drivers_msgs::tool_closeGoal close_gripper_goal_;
    bool close_gripperMakeActionRequest();
    void close_gripperDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_closeResultConstPtr& result);
    void close_gripperActive();
    void close_gripperFeedback(const iri_common_drivers_msgs::tool_closeFeedbackConstPtr& feedback);

    actionlib::SimpleActionClient<iri_common_drivers_msgs::tool_openAction> open_gripper_client_;
    iri_common_drivers_msgs::tool_openGoal open_gripper_goal_;
    bool open_gripperMakeActionRequest();
    void open_gripperDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_openResultConstPtr& result);
    void open_gripperActive();
    void open_gripperFeedback(const iri_common_drivers_msgs::tool_openFeedbackConstPtr& feedback);


    geometry_msgs::PoseStamped first_pose;

    sensor_msgs::JointState joints_dropping_pose;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:

    bool volatile gripper_open;
    bool real_robot;
    bool use_moveit;

   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    TableClearingExecuteAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~TableClearingExecuteAlgNode(void);

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
    void testTrajectory();
    void testIK();

    trajectory_msgs::JointTrajectoryPoint setTrajectoryPoint(sensor_msgs::JointState joint_state, double secs_ = 3.0);

    bool askForUserInput(std::string text);

    bool move2JointsPose(sensor_msgs::JointState joint_state,double velocity , double acceleration);

    bool atHomePosition();

};

#endif
