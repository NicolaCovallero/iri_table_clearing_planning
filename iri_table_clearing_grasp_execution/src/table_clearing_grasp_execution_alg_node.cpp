#include "table_clearing_grasp_execution_alg_node.h"

const std::string IK_SERVICE = "/estirabot/estirabot_tcp_ik/get_wam_ik";
const std::string FROM_POSE_IK_SERVICE = "/estirabot/estirabot_tcp_ik/get_wam_ik_from_pose";

TableClearingGraspExecutionAlgNode::TableClearingGraspExecutionAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingGraspExecutionAlgorithm>(),
  close_tool_client_("close_tool", true),
  open_tool_client_("open_tool", true),
  traj_client_("traj", true),
  grasp_action_aserver_(public_node_handle_, "grasp_action")
{
  //init class attributes if necessary
  this->loop_rate_ = 10;//in [Hz]

  std::string ik_service, from_pose_ik_service;
  this->public_node_handle_.param("ik_service", ik_service,IK_SERVICE);
  this->public_node_handle_.param("from_pose_ik_service", from_pose_ik_service, FROM_POSE_IK_SERVICE);

  // initialize the homeState variable
  this->home_joint_state.position.resize(7);
  this->home_joint_state.position[0] = 0.0f;
  this->home_joint_state.position[1] = 0.0f;
  this->home_joint_state.position[2] = 0.0f;
  this->home_joint_state.position[3] = 0.0f;
  this->home_joint_state.position[4] = 0.0f;
  this->home_joint_state.position[5] = 0.0f;
  this->home_joint_state.position[6] = 0.0f;
  this->home_joint_state.velocity.resize(7);
  this->home_joint_state.velocity[0] = 0.0f;
  this->home_joint_state.velocity[1] = 0.0f;
  this->home_joint_state.velocity[2] = 0.0f;
  this->home_joint_state.velocity[3] = 0.0f;
  this->home_joint_state.velocity[4] = 0.0f;
  this->home_joint_state.velocity[5] = 0.0f;
  this->home_joint_state.velocity[6] = 0.0f;
  this->home_joint_state.effort.resize(7);
  this->home_joint_state.effort[0] = 0.0f;
  this->home_joint_state.effort[1] = 0.0f;
  this->home_joint_state.effort[2] = 0.0f;
  this->home_joint_state.effort[3] = 0.0f;
  this->home_joint_state.effort[4] = 0.0f;
  this->home_joint_state.effort[5] = 0.0f;
  this->home_joint_state.effort[6] = 0.0f;
  this->home_joint_state.name.resize(7);
  this->home_joint_state.name[0] = "estirabot_joint_1";
  this->home_joint_state.name[1] = "estirabot_joint_2";
  this->home_joint_state.name[2] = "estirabot_joint_3";
  this->home_joint_state.name[3] = "estirabot_joint_4";
  this->home_joint_state.name[4] = "estirabot_joint_5";
  this->home_joint_state.name[5] = "estirabot_joint_6";
  this->home_joint_state.name[6] = "estirabot_joint_7";  

  // [init publishers]
  this->goal_pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  gripper_ik_from_pose_client_ = this->public_node_handle_.serviceClient<iri_wam_common_msgs::QueryWamInverseKinematicsFromPose>("from_pose_ik_service");
  
  // [init action servers]
  grasp_action_aserver_.registerStartCallback(boost::bind(&TableClearingGraspExecutionAlgNode::grasp_actionStartCallback, this, _1));
  grasp_action_aserver_.registerStopCallback(boost::bind(&TableClearingGraspExecutionAlgNode::grasp_actionStopCallback, this));
  grasp_action_aserver_.registerIsFinishedCallback(boost::bind(&TableClearingGraspExecutionAlgNode::grasp_actionIsFinishedCallback, this));
  grasp_action_aserver_.registerHasSucceedCallback(boost::bind(&TableClearingGraspExecutionAlgNode::grasp_actionHasSucceededCallback, this));
  grasp_action_aserver_.registerGetResultCallback(boost::bind(&TableClearingGraspExecutionAlgNode::grasp_actionGetResultCallback, this, _1));
  grasp_action_aserver_.registerGetFeedbackCallback(boost::bind(&TableClearingGraspExecutionAlgNode::grasp_actionGetFeedbackCallback, this, _1));
  grasp_action_aserver_.start();
  this->grasp_action_active=false;
  this->grasp_action_succeeded=false;
  this->grasp_action_finished=false;

  
  // [init action clients]
}

TableClearingGraspExecutionAlgNode::~TableClearingGraspExecutionAlgNode(void)
{
  // [free dynamic memory]
}

void TableClearingGraspExecutionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // Initialize the topic message structure
  //this->goal_pose_PoseStamped_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  //gripper_ik_from_pose_srv_.request.data = my_var;
  //ROS_INFO("TableClearingGraspExecutionAlgNode:: Sending New Request!");
  //if (gripper_ik_from_pose_client_.call(gripper_ik_from_pose_srv_))
  //{
    //ROS_INFO("TableClearingGraspExecutionAlgNode:: Response: %s", gripper_ik_from_pose_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingGraspExecutionAlgNode:: Failed to Call Server on topic gripper_ik_from_pose ");
  //}


  
  // [fill action structure and make request to the action server]
  // variable to hold the state of the current goal on the server
  //actionlib::SimpleClientGoalState close_tool_state(actionlib::SimpleClientGoalState::PENDING);
  // to get the state of the current goal
  //alg_.unlock();
  //close_tool_state=close_tool_client_.getState();
  // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
  //alg_.lock();
  //if(close_tool_state==actionlib::SimpleClientGoalState::ABORTED)
  //{
  //  do something
  //}
  //else if(close_tool_state==actionlib::SimpleClientGoalState::SUCCEEDED)
  //{
  //  do something else
  //}
  //close_toolMakeActionRequest();

  // variable to hold the state of the current goal on the server
  //actionlib::SimpleClientGoalState open_tool_state(actionlib::SimpleClientGoalState::PENDING);
  // to get the state of the current goal
  //alg_.unlock();
  //open_tool_state=open_tool_client_.getState();
  // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
  //alg_.lock();
  //if(open_tool_state==actionlib::SimpleClientGoalState::ABORTED)
  //{
  //  do something
  //}
  //else if(open_tool_state==actionlib::SimpleClientGoalState::SUCCEEDED)
  //{
  //  do something else
  //}
  //open_toolMakeActionRequest();

  // variable to hold the state of the current goal on the server
  //actionlib::SimpleClientGoalState traj_state(actionlib::SimpleClientGoalState::PENDING);
  // to get the state of the current goal
  //alg_.unlock();
  //traj_state=traj_client_.getState();
  // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
  //alg_.lock();
  //if(traj_state==actionlib::SimpleClientGoalState::ABORTED)
  //{
  //  do something
  //}
  //else if(traj_state==actionlib::SimpleClientGoalState::SUCCEEDED)
  //{
  //  do something else
  //}
  //trajMakeActionRequest();

  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).

  // To finish the action server with success
  //this->grasp_action_succeeded=true;
  //this->grasp_action_finished=true;
  // To finish the action server with failure
  //this->grasp_action_succeeded=false;
  //this->grasp_action_finished=true;

  // IMPORTANT: it is better to use the boolean variables to control the
  // behavior of the action server instead of direclty calling the action server
  // class functions.

  switch(step)
  {
    case 0: // check feasibility of the poses
      goal_pose_publisher_.publish(grasping_pose);
      
      gripper_ik_from_pose_srv_.request.desired_pose.header.stamp = ros::Time::now();
      feasible = true;
      for (uint i = 0; i < 4; ++i)
      {
        switch(i)
        {
          case 0: 
            gripper_ik_from_pose_srv_.request.current_joints = this->home_joint_state;
            gripper_ik_from_pose_srv_.request.desired_pose = approaching_pose;
            if (gripper_ik_from_pose_client_.call(gripper_ik_from_pose_srv_))
              joints_approaching_pose = gripper_ik_from_pose_srv_.response.desired_joints;
            else
            {
              ROS_ERROR("Impossible calling %s service or solution not found for approaching pose",gripper_ik_from_pose_client_.getService().c_str());
              feasible = false;
              grasp_action_finished = true;
              grasp_action_succeeded = true;
              return;
            }
            break;
          case 1: 
            gripper_ik_from_pose_srv_.request.current_joints = joints_approaching_pose;
            gripper_ik_from_pose_srv_.request.desired_pose = grasping_pose;
            if (gripper_ik_from_pose_client_.call(gripper_ik_from_pose_srv_))
              joints_grasping_pose = gripper_ik_from_pose_srv_.response.desired_joints;
            else
            {
              ROS_ERROR("Impossible calling %s service or solution not found for grasping pose",gripper_ik_from_pose_client_.getService().c_str());
              feasible = false;
              grasp_action_finished = true;
              grasp_action_succeeded = true;
              return;
            }
            break;
          case 2:
            gripper_ik_from_pose_srv_.request.current_joints = joints_approaching_pose;
            gripper_ik_from_pose_srv_.request.desired_pose = pre_dropping_pose;
            gripper_ik_from_pose_srv_.request.desired_pose.header.frame_id =  "/estirabot_link_footprint";
            if (gripper_ik_from_pose_client_.call(gripper_ik_from_pose_srv_))
              joints_pre_dropping_pose = gripper_ik_from_pose_srv_.response.desired_joints;
            else
            {
              ROS_ERROR("Impossible calling %s service or solution not found for pre dropping pose",gripper_ik_from_pose_client_.getService().c_str());
              feasible = false;
              grasp_action_finished = true;
              grasp_action_succeeded = true;
              return;
            }
            break;
          // case 3:
          //   gripper_ik_from_pose_srv_.request.current_joints = joints_pre_dropping_pose;
          //   gripper_ik_from_pose_srv_.request.desired_pose = dropping_pose;
          //   gripper_ik_from_pose_srv_.request.desired_pose.header.frame_id =  "/estirabot_link_footprint";
          //   if (gripper_ik_from_pose_client_.call(gripper_ik_from_pose_srv_))
          //     joints_dropping_pose = gripper_ik_from_pose_srv_.response.desired_joints;
          //   else
          //   {
          //     ROS_ERROR("Impossible calling %s service or solution not found for dropping pose",gripper_ik_from_pose_client_.getService().c_str());
          //     feasible = false;
          //     grasp_action_finished = true;
          //     grasp_action_succeeded = true;
          //     return;
          //   }
            break;
          default: break;
        }  
      }
      info = "IK computed : Feasible";
      ROS_INFO("%s",info.c_str());
      step ++;
      break;
    case 1: // go to pre grasping pose
      info = "Going to pre grasping pose";
      ROS_INFO("%s",info.c_str());
      setJointPoseGoal(joints_approaching_pose);
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error going to pre grasping pose");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 2: // opening gripper
      info = "Opening gripper";
      ROS_INFO("%s",info.c_str());
      if(!open_toolMakeActionRequest())
      {
        ROS_ERROR("Error opening gripper");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
    case 3: // going to grasping pose
      info = "Going to grasping pose";
      ROS_INFO("%s",info.c_str());
      setJointPoseGoal(joints_grasping_pose);
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error going to grasping pose");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 4: // closing gripper
      info = "Closin gripper";
      ROS_INFO("%s",info.c_str());
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error closin gripper");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 5: // going to pre grasping pose again
      info = "Going to pre grasping pose again";
      ROS_INFO("%s",info.c_str());
      setJointPoseGoal(joints_approaching_pose);
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error going to pre grasping pose again");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 6: // going to pre dropping pose
      info = "Going to pre dropping pose";
      ROS_INFO("%s",info.c_str());
      setJointPoseGoal(joints_pre_dropping_pose);
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error going to pre dropping pose");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 7: // going to dropping pose
      info = "Going to dropping pose";
      ROS_INFO("%s",info.c_str());
      setJointPoseGoal(joints_dropping_pose);
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error going to dropping pose");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 8: // opening the gripper 
      info = "Opening gripper";
      ROS_INFO("%s",info.c_str());
      if(!open_toolMakeActionRequest())
      {
        ROS_ERROR("Error opening gripper");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }

      // wait in order to let the object fall down
      ros::Duration(1).sleep(); // sleep for a second

      step ++;
      break;
    case 9:
      info = "Closin gripper";
      ROS_INFO("%s",info.c_str());
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error closin gripper");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      step ++;
      break;
    case 10: // go home
      info = "Going to home";
      ROS_INFO("%s",info.c_str());
      setJointPoseGoal(home_joint_state);
      if(!trajMakeActionRequest())
      {
        ROS_ERROR("Error going home");
        grasp_action_finished = true;
        grasp_action_succeeded = false;
      }
      grasp_action_finished = true;
      grasp_action_succeeded = true;
      step ++;
      break;
  }


  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->goal_pose_publisher_.publish(this->goal_pose_PoseStamped_msg_);

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void TableClearingGraspExecutionAlgNode::close_toolDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_closeResultConstPtr& result)
{
  alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
    ROS_INFO("TableClearingGraspExecutionAlgNode::close_toolDone: Goal Achieved!");
  else
    ROS_INFO("TableClearingGraspExecutionAlgNode::close_toolDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::close_toolActive()
{
  alg_.lock();
  //ROS_INFO("TableClearingGraspExecutionAlgNode::close_toolActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::close_toolFeedback(const iri_common_drivers_msgs::tool_closeFeedbackConstPtr& feedback)
{
  alg_.lock();
  //ROS_INFO("TableClearingGraspExecutionAlgNode::close_toolFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    close_tool_client_.cancelGoal();
    //ROS_INFO("TableClearingGraspExecutionAlgNode::close_toolFeedback: Cancelling Action!");
  }
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::open_toolDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_openResultConstPtr& result)
{
  alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
    ROS_INFO("TableClearingGraspExecutionAlgNode::open_toolDone: Goal Achieved!");
  else
    ROS_INFO("TableClearingGraspExecutionAlgNode::open_toolDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::open_toolActive()
{
  alg_.lock();
  //ROS_INFO("TableClearingGraspExecutionAlgNode::open_toolActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::open_toolFeedback(const iri_common_drivers_msgs::tool_openFeedbackConstPtr& feedback)
{
  alg_.lock();
  //ROS_INFO("TableClearingGraspExecutionAlgNode::open_toolFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    open_tool_client_.cancelGoal();
    //ROS_INFO("TableClearingGraspExecutionAlgNode::open_toolFeedback: Cancelling Action!");
  }
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::trajDone(const actionlib::SimpleClientGoalState& state,  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
    ROS_INFO("TableClearingGraspExecutionAlgNode::trajDone: Goal Achieved!");
  else
    ROS_INFO("TableClearingGraspExecutionAlgNode::trajDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::trajActive()
{
  alg_.lock();
  //ROS_INFO("TableClearingGraspExecutionAlgNode::trajActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::trajFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  alg_.lock();
  //ROS_INFO("TableClearingGraspExecutionAlgNode::trajFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    traj_client_.cancelGoal();
    //ROS_INFO("TableClearingGraspExecutionAlgNode::trajFeedback: Cancelling Action!");
  }
  alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::grasp_actionStartCallback(const iri_table_clearing_common_msgs::GraspObjectGoalConstPtr& goal)
{
  this->alg_.lock();
  //check goal
  this->grasp_action_active=true;
  this->grasp_action_succeeded=false;
  this->grasp_action_finished=false;

  step = 0;

  grasping_pose = goal->grasping_pose;
  approaching_pose = goal->approaching_pose;
  pre_dropping_pose = goal->pre_dropping_pose;
  dropping_pose = goal->dropping_pose;

  //execute goal
  this->alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::grasp_actionStopCallback(void)
{
  this->alg_.lock();
  //stop action
  this->grasp_action_active=false;
  this->alg_.unlock();
}

bool TableClearingGraspExecutionAlgNode::grasp_actionIsFinishedCallback(void)
{
  return feasible;

  // bool ret = false;

  // this->alg_.lock();
  // //if action has finish for any reason
  // ret = this->grasp_action_finished;
  // this->alg_.unlock();

  // return ret;
}

bool TableClearingGraspExecutionAlgNode::grasp_actionHasSucceededCallback(void)
{
  bool ret = false;

  this->alg_.lock();
  //if goal was accomplished
  ret = this->grasp_action_succeeded;
  this->grasp_action_active=false;
  this->alg_.unlock();

  return ret;
}

void TableClearingGraspExecutionAlgNode::grasp_actionGetResultCallback(iri_table_clearing_common_msgs::GraspObjectResultPtr& result)
{
  this->alg_.lock();
  //update result data to be sent to client
  //result->data = data;
  this->alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::grasp_actionGetFeedbackCallback(iri_table_clearing_common_msgs::GraspObjectFeedbackPtr& feedback)
{
  this->alg_.lock();
  //update feedback data to be sent to client
  //ROS_INFO("feedback: %s", feedback->data.c_str());
  feedback->info = info;
  this->alg_.unlock();
}


/*  [action requests] */
bool TableClearingGraspExecutionAlgNode::close_toolMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  // this->alg_.unlock();
  if(close_tool_client_.isServerConnected())
  {
    //ROS_DEBUG("TableClearingGraspExecutionAlgNode::close_toolMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //close_tool_goal_.data = my_desired_goal;
    close_tool_client_.sendGoal(close_tool_goal_,
                boost::bind(&TableClearingGraspExecutionAlgNode::close_toolDone,     this, _1, _2),
                boost::bind(&TableClearingGraspExecutionAlgNode::close_toolActive,   this),
                boost::bind(&TableClearingGraspExecutionAlgNode::close_toolFeedback, this, _1));
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingGraspExecutionAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingGraspExecutionAlgNode::close_toolMakeActionRequest: HRI server is not connected");
    return false;
  }
}

bool TableClearingGraspExecutionAlgNode::open_toolMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  // this->alg_.unlock();
  if(open_tool_client_.isServerConnected())
  {
    //ROS_DEBUG("TableClearingGraspExecutionAlgNode::open_toolMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //open_tool_goal_.data = my_desired_goal;
    open_tool_client_.sendGoal(open_tool_goal_,
                boost::bind(&TableClearingGraspExecutionAlgNode::open_toolDone,     this, _1, _2),
                boost::bind(&TableClearingGraspExecutionAlgNode::open_toolActive,   this),
                boost::bind(&TableClearingGraspExecutionAlgNode::open_toolFeedback, this, _1));
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingGraspExecutionAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingGraspExecutionAlgNode::open_toolMakeActionRequest: HRI server is not connected");
    return false;
  }
}

bool TableClearingGraspExecutionAlgNode::trajMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  // this->alg_.unlock();
  if(traj_client_.isServerConnected())
  {
    //ROS_DEBUG("TableClearingGraspExecutionAlgNode::trajMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //traj_goal_.data = my_desired_goal;
    traj_client_.sendGoal(traj_goal_,
                boost::bind(&TableClearingGraspExecutionAlgNode::trajDone,     this, _1, _2),
                boost::bind(&TableClearingGraspExecutionAlgNode::trajActive,   this),
                boost::bind(&TableClearingGraspExecutionAlgNode::trajFeedback, this, _1));
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingGraspExecutionAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingGraspExecutionAlgNode::trajMakeActionRequest: HRI server is not connected");
    return false;
  }
}


void TableClearingGraspExecutionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TableClearingGraspExecutionAlgNode::setJointPoseGoal(sensor_msgs::JointState joint_pose)
{
  
  traj_goal_.trajectory.joint_names.resize(7);
  traj_goal_.trajectory.joint_names[0] = "estirabot_joint_1";
  traj_goal_.trajectory.joint_names[1] = "estirabot_joint_2";
  traj_goal_.trajectory.joint_names[2] = "estirabot_joint_3";
  traj_goal_.trajectory.joint_names[3] = "estirabot_joint_4";
  traj_goal_.trajectory.joint_names[4] = "estirabot_joint_5";
  traj_goal_.trajectory.joint_names[5] = "estirabot_joint_6";
  traj_goal_.trajectory.joint_names[6] = "estirabot_joint_7";  

  traj_goal_.trajectory.points.resize(1);
  traj_goal_.trajectory.points[0].positions.resize(7);
  traj_goal_.trajectory.points[0].positions[0] = joint_pose.position[0];
  traj_goal_.trajectory.points[0].positions[1] = joint_pose.position[1];
  traj_goal_.trajectory.points[0].positions[2] = joint_pose.position[2];
  traj_goal_.trajectory.points[0].positions[3] = joint_pose.position[3];
  traj_goal_.trajectory.points[0].positions[4] = joint_pose.position[4];
  traj_goal_.trajectory.points[0].positions[5] = joint_pose.position[5];
  traj_goal_.trajectory.points[0].positions[6] = joint_pose.position[6];

  traj_goal_.trajectory.points[0].velocities.resize(7);
  traj_goal_.trajectory.points[0].velocities[0] = 0.02f;
  traj_goal_.trajectory.points[0].velocities[1] = 0.02f;
  traj_goal_.trajectory.points[0].velocities[2] = 0.02f;
  traj_goal_.trajectory.points[0].velocities[3] = 0.02f;
  traj_goal_.trajectory.points[0].velocities[5] = 0.02f;
  traj_goal_.trajectory.points[0].velocities[4] = 0.02f;
  traj_goal_.trajectory.points[0].velocities[6] = 0.02f;
  traj_goal_.trajectory.points[0].accelerations.resize(7);
  traj_goal_.trajectory.points[0].accelerations[0] = 0.0f;
  traj_goal_.trajectory.points[0].accelerations[1] = 0.0f;
  traj_goal_.trajectory.points[0].accelerations[2] = 0.0f;
  traj_goal_.trajectory.points[0].accelerations[3] = 0.0f;
  traj_goal_.trajectory.points[0].accelerations[4] = 0.0f;
  traj_goal_.trajectory.points[0].accelerations[5] = 0.0f;
  traj_goal_.trajectory.points[0].accelerations[6] = 0.0f;
  traj_goal_.trajectory.points[0].time_from_start = ros::Duration(3); 

}

void TableClearingGraspExecutionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TableClearingGraspExecutionAlgNode>(argc, argv, "table_clearing_grasp_execution_alg_node");
}
