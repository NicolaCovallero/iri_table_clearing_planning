#include "table_clearing_execute_alg_node.h"

const std::string IK_SERVICE = "/estirabot/estirabot_tcp_ik/get_wam_ik";
const std::string FROM_POSE_IK_SERVICE = "/estirabot/estirabot_tcp_ik/get_wam_ik_from_pose";

TableClearingExecuteAlgNode::TableClearingExecuteAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingExecuteAlgorithm>(),
  close_gripper_client_("close_gripper", true),
  open_gripper_client_("open_gripper", true)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  std::string ik_service, from_pose_ik_service;
  this->public_node_handle_.param("ik_service", ik_service,IK_SERVICE);
  this->public_node_handle_.param("from_pose_ik_service", from_pose_ik_service, FROM_POSE_IK_SERVICE);
  this->public_node_handle_.param("automatic", this->alg_.automatic, false);
  this->public_node_handle_.param("real_robot", this->real_robot, false);

  std::string execution_str;
  if(this->alg_.automatic)
    execution_str = "automatic";
  else
    execution_str = "manual";

  std::cout << "ik_service: "  << ik_service << std::endl
            << "from_pose_ik_service: " << from_pose_ik_service << std::endl
            << "execution: " << execution_str << std::endl
            << "real_robot: " << real_robot << std::endl;

  // [init publishers]
  this->action_pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("action_pose", 1);
  
  // [init subscribers]
  this->current_joint_state_subscriber_ = this->public_node_handle_.subscribe("/estirabot/joint_states", 1, &TableClearingExecuteAlgNode::current_joint_state_callback, this);
  pthread_mutex_init(&this->current_joint_state_mutex_,NULL);

  
  // [init services]
  this->execute_grasping_server_ = this->public_node_handle_.advertiseService("execute_grasping", &TableClearingExecuteAlgNode::execute_graspingCallback, this);
  pthread_mutex_init(&this->execute_grasping_mutex_,NULL);

  this->execute_pushing_server_ = this->public_node_handle_.advertiseService("execute_pushing", &TableClearingExecuteAlgNode::execute_pushingCallback, this);
  pthread_mutex_init(&this->execute_pushing_mutex_,NULL);

  
  // [init clients]
  remove_object_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_gazebo::DeleteObject>("/estirabot/delete_model");

  move_joints_client_ = this->public_node_handle_.serviceClient<iri_common_drivers_msgs::QueryJointsMovement>("move_joints");

  estirabot_gripper_ik_client_ = this->public_node_handle_.serviceClient<iri_common_drivers_msgs::QueryInverseKinematics>(ik_service);

  estirabot_gripper_ik_from_pose_client_ = this->public_node_handle_.serviceClient<iri_wam_common_msgs::QueryWamInverseKinematicsFromPose>(from_pose_ik_service);

  

  // [init action servers]
  
  // [init action clients]
  // traj_client_ = new TrajClient("follow_joint_trajectory", true);


  // // initialize the homeState variable
  this->alg_.home_joint_state.position.resize(7);
  this->alg_.home_joint_state.position[0] = 0.0f;
  this->alg_.home_joint_state.position[1] = 0.0f;
  this->alg_.home_joint_state.position[2] = 0.0f;
  this->alg_.home_joint_state.position[3] = 0.0f;
  this->alg_.home_joint_state.position[4] = 0.0f;
  this->alg_.home_joint_state.position[5] = 0.0f;
  this->alg_.home_joint_state.position[6] = 0.0f;
  this->alg_.home_joint_state.velocity.resize(7);
  this->alg_.home_joint_state.velocity[0] = 0.0f;
  this->alg_.home_joint_state.velocity[1] = 0.0f;
  this->alg_.home_joint_state.velocity[2] = 0.0f;
  this->alg_.home_joint_state.velocity[3] = 0.0f;
  this->alg_.home_joint_state.velocity[4] = 0.0f;
  this->alg_.home_joint_state.velocity[5] = 0.0f;
  this->alg_.home_joint_state.velocity[6] = 0.0f;
  this->alg_.home_joint_state.effort.resize(7);
  this->alg_.home_joint_state.effort[0] = 0.0f;
  this->alg_.home_joint_state.effort[1] = 0.0f;
  this->alg_.home_joint_state.effort[2] = 0.0f;
  this->alg_.home_joint_state.effort[3] = 0.0f;
  this->alg_.home_joint_state.effort[4] = 0.0f;
  this->alg_.home_joint_state.effort[5] = 0.0f;
  this->alg_.home_joint_state.effort[6] = 0.0f;
  this->alg_.home_joint_state.name.resize(7);
  this->alg_.home_joint_state.name[0] = "estirabot_joint_1";
  this->alg_.home_joint_state.name[1] = "estirabot_joint_2";
  this->alg_.home_joint_state.name[2] = "estirabot_joint_3";
  this->alg_.home_joint_state.name[3] = "estirabot_joint_4";
  this->alg_.home_joint_state.name[4] = "estirabot_joint_5";
  this->alg_.home_joint_state.name[5] = "estirabot_joint_6";
  this->alg_.home_joint_state.name[6] = "estirabot_joint_7";  

  this->alg_.current_joint_state_ = this->alg_.home_joint_state;

  joints_dropping_pose.position.resize(7);
  joints_dropping_pose.position[0] = 1.62731;
  joints_dropping_pose.position[1] = 0.999324;
  joints_dropping_pose.position[2] = -0.0757397;
  joints_dropping_pose.position[3] = 1.16821;
  joints_dropping_pose.position[4] = -3.06471;
  joints_dropping_pose.position[5] = -0.976739;
  joints_dropping_pose.position[6] = -1.5984;
  joints_dropping_pose.velocity.resize(7);
  joints_dropping_pose.velocity[0] = 0.0f;
  joints_dropping_pose.velocity[1] = 0.0f;
  joints_dropping_pose.velocity[2] = 0.0f;
  joints_dropping_pose.velocity[3] = 0.0f;
  joints_dropping_pose.velocity[4] = 0.0f;
  joints_dropping_pose.velocity[5] = 0.0f;
  joints_dropping_pose.velocity[6] = 0.0f;
  joints_dropping_pose.effort.resize(7);
  joints_dropping_pose.effort[0] = 0.0f;
  joints_dropping_pose.effort[1] = 0.0f;
  joints_dropping_pose.effort[2] = 0.0f;
  joints_dropping_pose.effort[3] = 0.0f;
  joints_dropping_pose.effort[4] = 0.0f;
  joints_dropping_pose.effort[5] = 0.0f;
  joints_dropping_pose.effort[6] = 0.0f;
  joints_dropping_pose.name.resize(7);
  joints_dropping_pose.name[0] = "estirabot_joint_1";
  joints_dropping_pose.name[1] = "estirabot_joint_2";
  joints_dropping_pose.name[2] = "estirabot_joint_3";
  joints_dropping_pose.name[3] = "estirabot_joint_4";
  joints_dropping_pose.name[4] = "estirabot_joint_5";
  joints_dropping_pose.name[5] = "estirabot_joint_6";
  joints_dropping_pose.name[6] = "estirabot_joint_7";  

  gripper_open = false;
}

TableClearingExecuteAlgNode::~TableClearingExecuteAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->current_joint_state_mutex_);
  pthread_mutex_destroy(&this->execute_grasping_mutex_);
  pthread_mutex_destroy(&this->execute_pushing_mutex_);
}

void TableClearingExecuteAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // Initialize the topic message structure
  //this->current_joint_state_JointState_msg_.data = my_var;

  // Initialize the topic message structure
  //this->action_pose_PoseStamped_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  //remove_object_srv_.request.data = my_var;
  //ROS_INFO("TableClearingExecuteAlgNode:: Sending New Request!");
  //if (remove_object_client_.call(remove_object_srv_))
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Response: %s", remove_object_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Failed to Call Server on topic remove_object ");
  //}


  //move_joints_srv_.request.data = my_var;
  //ROS_INFO("TableClearingExecuteAlgNode:: Sending New Request!");
  //if (move_joints_client_.call(move_joints_srv_))
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Response: %s", move_joints_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Failed to Call Server on topic move_joints ");
  //}


  //estirabot_gripper_ik_srv_.request.data = my_var;
  //ROS_INFO("TableClearingExecuteAlgNode:: Sending New Request!");
  //if (estirabot_gripper_ik_client_.call(estirabot_gripper_ik_srv_))
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Response: %s", estirabot_gripper_ik_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Failed to Call Server on topic estirabot_gripper_ik ");
  //}


  //estirabot_gripper_ik_srv_.request.data = my_var;
  //ROS_INFO("TableClearingExecuteAlgNode:: Sending New Request!");
  //if (estirabot_gripper_ik_from_pose_client_.call(estirabot_gripper_ik_srv_))
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Response: %s", estirabot_gripper_ik_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingExecuteAlgNode:: Failed to Call Server on topic estirabot_gripper_ik ");
  //}

  // [fill action structure and make request to the action server]
  // variable to hold the state of the current goal on the server
  //actionlib::SimpleClientGoalState close_gripper_state(actionlib::SimpleClientGoalState::PENDING);
  // to get the state of the current goal
  //alg_.unlock();
  //close_gripper_state=close_gripper_client_.getState();
  // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
  //alg_.lock();
  //if(close_gripper_state==actionlib::SimpleClientGoalState::ABORTED)
  //{
  //  do something
  //}
  //else if(close_gripper_state==actionlib::SimpleClientGoalState::SUCCEEDED)
  //{
  //  do something else
  //}
  //close_gripperMakeActionRequest();

  // variable to hold the state of the current goal on the server
  //actionlib::SimpleClientGoalState open_gripper_state(actionlib::SimpleClientGoalState::PENDING);
  // to get the state of the current goal
  //alg_.unlock();
  //open_gripper_state=open_gripper_client_.getState();
  // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
  //alg_.lock();
  //if(open_gripper_state==actionlib::SimpleClientGoalState::ABORTED)
  //{
  //  do something
  //}
  //else if(open_gripper_state==actionlib::SimpleClientGoalState::SUCCEEDED)
  //{
  //  do something else
  //}
  //open_gripperMakeActionRequest();

  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).


  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->current_joint_state_publisher_.publish(this->current_joint_state_JointState_msg_);

  // publish only if there is a message, without this check the terminal of the rviz will be 
  // filled by warning messages
  if(first_pose.header.frame_id.length() > 0)
    action_pose_publisher_.publish(first_pose);

  // Uncomment the following line to publish the topic message
  //this->action_pose_publisher_.publish(this->action_pose_PoseStamped_msg_);

}

/*  [subscriber callbacks] */
void TableClearingExecuteAlgNode::current_joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ROS_INFO("TableClearingExecuteAlgNode::current_joint_state_callback: New Message Received");

  this->alg_.current_joint_state_ = *msg;
  this->alg_.current_joint_state_.name[0] = "estirabot_joint_1";
  this->alg_.current_joint_state_.name[1] = "estirabot_joint_2";
  this->alg_.current_joint_state_.name[2] = "estirabot_joint_3";
  this->alg_.current_joint_state_.name[3] = "estirabot_joint_4";
  this->alg_.current_joint_state_.name[4] = "estirabot_joint_5";
  this->alg_.current_joint_state_.name[5] = "estirabot_joint_6";
  this->alg_.current_joint_state_.name[6] = "estirabot_joint_7";  

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->current_joint_state_mutex_enter();

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->current_joint_state_mutex_exit();
}

void TableClearingExecuteAlgNode::current_joint_state_mutex_enter(void)
{
  pthread_mutex_lock(&this->current_joint_state_mutex_);
}

void TableClearingExecuteAlgNode::current_joint_state_mutex_exit(void)
{
  pthread_mutex_unlock(&this->current_joint_state_mutex_);
}


/*  [service callbacks] */
bool TableClearingExecuteAlgNode::execute_graspingCallback(iri_table_clearing_execute::ExecuteGrasping::Request &req, iri_table_clearing_execute::ExecuteGrasping::Response &res)
{
  ROS_INFO("TableClearingExecuteAlgNode::execute_graspingCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->execute_grasping_mutex_enter();

  ROS_DEBUG("Publishing grasping pose");
  //action_pose_publisher_.publish(req.grasping_pose);
  //action_pose_publisher_.publish(req.approaching_pose);
  first_pose = req.approaching_pose;

  //ROS_INFO("TableClearingExecuteAlgNode::execute_graspingCallback: Processing New Request!");
  iri_wam_common_msgs::QueryWamInverseKinematicsFromPose srv;
  //srv.request.current_joints = this->alg_.home_joint_state;
  srv.request.current_joints = this->alg_.current_joint_state_;
  srv.request.desired_pose = req.approaching_pose;

  std::vector<sensor_msgs::JointState> joints_trajectory;
  joints_trajectory.resize(3);

  util::uint64 t_init_ik = util::GetTimeMs64(); 
  res.success = true;
  for (uint i = 0; i < 3; ++i) // we don't care about the dropping pose
  {
    switch(i)
    {
      case 0: 
        srv.request.current_joints = this->alg_.home_joint_state;
        srv.request.desired_pose = req.approaching_pose;
        break;
      case 1: 
        srv.request.current_joints = joints_trajectory[0];
        srv.request.desired_pose = req.grasping_pose;
        break;
      case 2:
        srv.request.current_joints = joints_trajectory[1];
        srv.request.desired_pose = req.post_grasping_pose;
        break;
      // case 2:
      //   srv.request.current_joints = joints_trajectory[1];
      //   srv.request.desired_pose = req.pre_dropping_pose;
      //   srv.request.desired_pose.header.frame_id =  "/estirabot_link_footprint";
      //   break;
      // case 3:
      //   srv.request.current_joints = joints_trajectory[2];
      //   srv.request.desired_pose = req.dropping_pose;
      //   srv.request.desired_pose.header.frame_id =  "/estirabot_link_footprint";
      //   break;
      default: break;
    }  
    srv.request.desired_pose.header.stamp = ros::Time::now();
    if (estirabot_gripper_ik_from_pose_client_.call(srv))
    {
      joints_trajectory[i] = srv.response.desired_joints;
    }
    else
    {
      switch(i)
      {
        case 0:
          ROS_ERROR("Impossible calling %s service or solution not found for approaching pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
          break;    
        case 1:
          ROS_ERROR("Impossible calling %s service or solution not found for grasping pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
          break;
        case 2:
          ROS_ERROR("Impossible calling %s service or solution not found for post_grasping_pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
          break;
        case 3:
          ROS_ERROR("Impossible calling %s service or solution not found for dropping pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
          break;
        default:break;        
      }
      res.success = false; 
      res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);  
      return true;
    }
  }
  joints_trajectory.push_back(joints_dropping_pose);
  res.ik_time = (float)(util::GetTimeMs64() - t_init_ik); 


  // ROS_INFO("Waiting for the joint_trajectory_action server");
  // while(!traj_client_->waitForServer(ros::Duration(5.0))){
  //     ROS_INFO("Waiting for the joint_trajectory_action server");
  // }
  
  if(this->alg_.automatic == MANUAL_EXECUTION)
  {
    char response;
    bool wrong_character = true;
    while(wrong_character)
    {
      std::cout << "\n\n The trajectory is ready to execute, do you want to execute?(y,n)";
      std::cin >> response;
      switch(response)
      {
        case 'y':
        case 'Y':
            wrong_character = false;
            std::cout << "\n";
            break;
        case 'n':
        case 'N':
            std::cout << "\nYou decided to NOT execute the trajectory\n";
            std::cout << "\nWaiting for new request...\n";
            return false; // return false because the trajectory is not executed
            break;
        default: break;
      }
    }
  } 

  util::uint64 t_init_grasp = util::GetTimeMs64();

  
  // Go to pregrasping pose - Approaching pose
  if(askForUserInput("Going to pre grasping pose"))
  {
    ROS_INFO("Going to pre grasping pose");
    if(!move2JointsPose(joints_trajectory[0],config_.velocity_max,config_.acceleration_max))
    {
      return false;
    }
    // remove the object from the table
    if( not this->real_robot)
    {
      geometry_msgs::PointStamped point;
      point.point = req.grasping_pose.pose.position;
      point.header = req.grasping_pose.header;
      remove_object_srv_.request.point = point;
      if(not this->remove_object_client_.call(remove_object_srv_)) 
      {
        ROS_WARN("Problem deleting the object");
      }
    }
  }
  
  // OPEN THE GRIPPER
  if(askForUserInput("Opening gripper"))
  {
    if(this->real_robot)
    { 
      ROS_INFO("Opening gripper");
      this->open_gripperMakeActionRequest();
      while (not gripper_open){ros::Duration(0.1).sleep();}
      //ros::Duration(1).sleep();
    }
  }
  else // go home
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }


  // Go to grasping pose
  if(askForUserInput("Going to grasping pose"))
  {
    ROS_INFO("Going to grasping pose");
    if(!move2JointsPose(joints_trajectory[1],config_.velocity_max,config_.acceleration_max))
    {
      return false;
    }
    //ros::Duration(1).sleep(); // sleep for a second
  }
  else // go home
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }
 
  
  // CLOSE THE GRIPPER
  if(askForUserInput("Closing gripper"))
  {
    if(this->real_robot)
    { 
      if(!this->close_gripperMakeActionRequest())
      {
        ROS_WARN("Closing gripper service not connected");
      }
      else
      {
        while (gripper_open){ros::Duration(0.1).sleep();}

        //ros::Duration(1).sleep(); // sleep for a second
      }
    }
  }
  else // go home
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }

  // Go to Pregrasping pose again
  if(askForUserInput("Going to post_grasping_pose"))
  {
    ROS_INFO("Going to post grasping pose again");
    if(!move2JointsPose(joints_trajectory[2],config_.velocity_max,config_.acceleration_max))
    {
      return false;
    }
    //ros::Duration(1).sleep(); // sleep for a second
  }
  else // go home
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }

  // Go to bin
  if(askForUserInput("Going to pre dropping pose"))
  {
    ROS_INFO("Going to pre dropping pose");
    if(!move2JointsPose(joints_trajectory[3],config_.velocity_max,config_.acceleration_max))
    {
      return false;
    }
    //ros::Duration(1).sleep(); // sleep for a second
  }
  else // go home 
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }

  // // the pre dropping pose is fine
  // if(askForUserInput("Going to dropping pose"))
  // {
  //   ROS_INFO("Going to dropping pose");
  //   if(!move2JointsPose(joints_trajectory[3],config_.velocity_max,config_.acceleration_max))
  //   {
  //     return false;
  //   }
  //   ros::Duration(1).sleep(); // sleep for a second
  // }
  // else // go home
  // {
  //   ROS_INFO("Going home");
  //   if(atHomePosition())
  //     return true;
  //   else
  //     return false;
  // }

  // OPEN GRIPPER
  if(askForUserInput("Opening gripper"))
  {
    if(this->real_robot)
    { 
      ROS_INFO("Opening gripper");
      this->open_gripperMakeActionRequest();
      while (not gripper_open){ros::Duration(0.1).sleep();}
      //ros::Duration(1).sleep();
    }
  }
  else // go home
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }

  // wait 
  ros::Duration(1).sleep(); // sleep for a second

  // CLOSE THE GRIPPER
  if(askForUserInput("Closing gripper"))
  {    
    if(this->real_robot)
    { 
      if(!this->close_gripperMakeActionRequest())
      {
        ROS_WARN("Closing gripper service not connected");
      }
      else
      {
        while (gripper_open){ros::Duration(0.1).sleep();}
        //ros::Duration(1).sleep();
      }
    }
  }
  else // go home
  {
    ROS_INFO("Going home");
    if(atHomePosition())
      return true;
    else
      return false;
  }
  
  // we don't want to go home in order to save time
  // Go home
  // ROS_INFO("Going home");
  // if(atHomePosition())
  //   return true;
  // else
  //   return false;

  //unlock previously blocked shared variables
  //this->execute_grasping_mutex_exit();
  //this->alg_.unlock();

  res.execution_time = (float)(util::GetTimeMs64() - t_init_grasp);
  return true;
}

void TableClearingExecuteAlgNode::execute_grasping_mutex_enter(void)
{
  pthread_mutex_lock(&this->execute_grasping_mutex_);
}

void TableClearingExecuteAlgNode::execute_grasping_mutex_exit(void)
{
  pthread_mutex_unlock(&this->execute_grasping_mutex_);
}

bool TableClearingExecuteAlgNode::execute_pushingCallback(iri_table_clearing_execute::ExecutePushing::Request &req, iri_table_clearing_execute::ExecutePushing::Response &res)
{
  ROS_INFO("TableClearingExecuteAlgNode::execute_pushingCallback: New Request Received!");
  res.executed = false;
  // check input
  if(req.pushing_cartesian_trajectory.size() == 0)
  {
    ROS_ERROR("0 poses received - Impossible trajectory\n");
    res.success = true;
    return true;
  }

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->execute_pushing_mutex_enter();

  //ROS_INFO("TableClearingExecuteAlgNode::execute_pushingCallback: Processing New Request!");
  
  // ---------get the trajectory in the joint space -------------
  ROS_DEBUG("Getting the trajectory in joint space");
  iri_wam_common_msgs::QueryWamInverseKinematicsFromPose srv;
  std::vector<sensor_msgs::JointState> joints_trajectory;
  joints_trajectory.resize(req.pushing_cartesian_trajectory.size());

  ROS_DEBUG("Publishing pose");
  action_pose_publisher_.publish(req.pushing_cartesian_trajectory[0]);
  first_pose = req.pushing_cartesian_trajectory[0];
  
  //srv.request.current_joints = this->alg_.home_joint_state;
  srv.request.current_joints = this->alg_.current_joint_state_;
  // std::cout << "Point " << 0 << 
  //       " current_joint_state_ 1: " << this->alg_.current_joint_state_.position[0] <<
  //       " current_joint_state_ 2: " << this->alg_.current_joint_state_.position[1] <<
  //       " current_joint_state_ 3: " << this->alg_.current_joint_state_.position[2] <<
  //       " current_joint_state_ 4: " << this->alg_.current_joint_state_.position[3] <<
  //       " current_joint_state_ 5: " << this->alg_.current_joint_state_.position[4] <<
  //       " current_joint_state_ 6: " << this->alg_.current_joint_state_.position[5] <<
  //       " current_joint_state_ 7: " << this->alg_.current_joint_state_.position[6] << std::endl;

  srv.request.desired_pose = req.pushing_cartesian_trajectory[0];
  srv.request.desired_pose.header.stamp = ros::Time::now(); // IMPORTANT

  ROS_INFO("Trying calling the IK service");
  util::uint64 t_init_ik = util::GetTimeMs64();
  std::cout << "Requesting IK of x: " <<  req.pushing_cartesian_trajectory[0].pose.position.x << " y: " <<
                                            req.pushing_cartesian_trajectory[0].pose.position.y << " z: " <<
                                             req.pushing_cartesian_trajectory[0].pose.position.z << std::endl 
              << "[quat] x:" << req.pushing_cartesian_trajectory[0].pose.orientation.x 
              << " y: " << req.pushing_cartesian_trajectory[0].pose.orientation.y
              << " z: " << req.pushing_cartesian_trajectory[0].pose.orientation.z
              << " w: " << req.pushing_cartesian_trajectory[0].pose.orientation.w << std::endl;
  if (estirabot_gripper_ik_from_pose_client_.call(srv))
  {
    joints_trajectory[0] = srv.response.desired_joints;
    // std::cout << "Point " << 0 << 
    //     " joint 1: " << joints_trajectory[0].position[0] <<
    //     " joint 2: " << joints_trajectory[0].position[1] <<
    //     " joint 3: " << joints_trajectory[0].position[2] <<
    //     " joint 4: " << joints_trajectory[0].position[3] <<
    //     " joint 5: " << joints_trajectory[0].position[4] <<
    //     " joint 6: " << joints_trajectory[0].position[5] <<
    //     " joint 7: " << joints_trajectory[0].position[6] << std::endl;
    
  }
  else
  {
    // joints_trajectory[0] = srv.response.desired_joints;
    // std::cout << "Point " << 0 << 
    //     " joint 1: " << joints_trajectory[0].position[0] <<
    //     " joint 2: " << joints_trajectory[0].position[1] <<
    //     " joint 3: " << joints_trajectory[0].position[2] <<
    //     " joint 4: " << joints_trajectory[0].position[3] <<
    //     " joint 5: " << joints_trajectory[0].position[4] <<
    //     " joint 6: " << joints_trajectory[0].position[5] <<
    //     " joint 7: " << joints_trajectory[0].position[6] << std::endl;
    ROS_ERROR("Impossible calling %s service or solution not found for the pre pushing pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
    res.success = false; 
    res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);
    return true;
  }

  res.success = true; 
  for (int i = 1; i < req.pushing_cartesian_trajectory.size(); ++i)
  {
    srv.request.current_joints = joints_trajectory[i-1];
    srv.request.desired_pose = req.pushing_cartesian_trajectory[i];
    std::cout << "frame_id: " << req.pushing_cartesian_trajectory[i].header.frame_id << std::endl;
    srv.request.desired_pose.header.stamp = ros::Time::now(); // IMPORTANT
    std::cout << "Requesting IK of x: " <<  req.pushing_cartesian_trajectory[i].pose.position.x << " y: " <<
                                            req.pushing_cartesian_trajectory[i].pose.position.y << " z: " <<
                                             req.pushing_cartesian_trajectory[i].pose.position.z << std::endl 
              << "[quat] x:" << req.pushing_cartesian_trajectory[i].pose.orientation.x 
              << " y: " << req.pushing_cartesian_trajectory[i].pose.orientation.y
              << " z: " << req.pushing_cartesian_trajectory[i].pose.orientation.z
              << " w: " << req.pushing_cartesian_trajectory[i].pose.orientation.w << std::endl;
    if (estirabot_gripper_ik_from_pose_client_.call(srv))
    {
      ROS_INFO("Inverse Kinematics done");
      joints_trajectory[i] = srv.response.desired_joints;       
      // std::cout << "Point " << i << 
      //     " joint 1: " << joints_trajectory[i].position[0] <<
      //     " joint 2: " << joints_trajectory[i].position[1] <<
      //     " joint 3: " << joints_trajectory[i].position[2] <<
      //     " joint 4: " << joints_trajectory[i].position[3] <<
      //     " joint 5: " << joints_trajectory[i].position[4] <<
      //     " joint 6: " << joints_trajectory[i].position[5] <<
      //     " joint 7: " << joints_trajectory[i].position[6] << std::endl;
    }
    else
    {
      ROS_ERROR("Impossible calling %s service or solution not found, for pose %d",estirabot_gripper_ik_from_pose_client_.getService().c_str(),i);
      res.success = false; 
      res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);
      return true;
    }
  }

  if(req.pushing_until_graspable)
  {
    // evaluate if the future grasping, pre grasping and post grasping poses are feasible
    srv.request.current_joints = this->alg_.home_joint_state;
    srv.request.desired_pose.header.stamp = ros::Time::now();
    srv.request.desired_pose = req.future_pre_grasp_pose;
    if (not estirabot_gripper_ik_from_pose_client_.call(srv))
    {
      ROS_ERROR("Impossible calling %s service or solution not found, for future_pre_grasp_pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
      res.success = false; 
      res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);
      return true;
    }
    srv.request.current_joints = this->alg_.home_joint_state;
    srv.request.desired_pose.header.stamp = ros::Time::now();
    srv.request.desired_pose = req.future_grasp_pose;
    if (not estirabot_gripper_ik_from_pose_client_.call(srv))
    {
      ROS_ERROR("Impossible calling %s service or solution not found, for future_grasp_pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
      res.success = false; 
      res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);
      return true;
    }
    srv.request.current_joints = this->alg_.home_joint_state;
    srv.request.desired_pose.header.stamp = ros::Time::now();
    srv.request.desired_pose = req.future_post_grasp_pose;
    if (not estirabot_gripper_ik_from_pose_client_.call(srv))
    {
      ROS_ERROR("Impossible calling %s service or solution not found, for future_post_grasp_pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
      res.success = false; 
      res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);
      return true;
    }
  }
  

  res.ik_time = (float)(util::GetTimeMs64() - t_init_ik);

  // ROS_INFO("Waiting for the joint_trajectory_action server");
  // while(!traj_client_->waitForServer(ros::Duration(5.0))){
  //     ROS_INFO("Waiting for the joint_trajectory_action server");
  // }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.frame_id = req.pushing_cartesian_trajectory[0].header.frame_id;

  
  if(this->alg_.automatic == MANUAL_EXECUTION)
  {
    char response;
    bool wrong_character = true;
    while(wrong_character)
    {
      std::cout << "\n\n The trajectory is ready to execute, do you want to execute?(y,n)";
      std::cin >> response;
      switch(response)
      {
        case 'y':
        case 'Y':
            wrong_character = false;
            std::cout << "\n";
            break;
        case 'n':
        case 'N':
            std::cout << "\nYou decided to NOT execute the trajectory\n";
            std::cout << "\nWaiting for new request...\n";
            res.executed = false;
            return false; // return false because the trajectory is not executed
            break;
        default: break;
      }
    }
  } 

  // go to the first point of the trajectory -- IMPORTANT !!!!!!!!!!!!!!!!!!!!!
  //move2JointsPose(joints_trajectory[0],0.5,0.5);
  util::uint64 t_init_push = util::GetTimeMs64();
  for (int i = 0; i < joints_trajectory.size(); ++i)
  {
   move2JointsPose(joints_trajectory[i],0.5,0.5);
   ros::Duration(0.2).sleep();
  }
  res.execution_time = (float)(util::GetTimeMs64() - t_init_push);  
  // reset the time stamp for all the trajectory points
  // for (int i = 0; i < joints_trajectory.size(); ++i)
  //   joints_trajectory[i].header.stamp = ros::Time::now();


  ROS_INFO("Action finished - going home");
  res.executed = true;
  if(atHomePosition())
  {
    std::cout << "\nWaiting for new request...\n";
    return true;
  }
  else
  {
    std::cout << "\nWaiting for new request...\n";
    return false;
  }

  //unlock previously blocked shared variables
  //this->execute_pushing_mutex_exit();
  //this->alg_.unlock();

}

void TableClearingExecuteAlgNode::execute_pushing_mutex_enter(void)
{
  pthread_mutex_lock(&this->execute_pushing_mutex_);
}

void TableClearingExecuteAlgNode::execute_pushing_mutex_exit(void)
{
  pthread_mutex_unlock(&this->execute_pushing_mutex_);
}


/*  [action callbacks] */
void TableClearingExecuteAlgNode::close_gripperDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_closeResultConstPtr& result)
{
  alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
  {
    ROS_INFO("TableClearingExecuteAlgNode::close_gripperDone: Goal Achieved!");
    gripper_open = false;
  }
  else
    ROS_INFO("TableClearingExecuteAlgNode::close_gripperDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingExecuteAlgNode::close_gripperActive()
{
  alg_.lock();
  ROS_INFO("TableClearingExecuteAlgNode::close_gripperActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingExecuteAlgNode::close_gripperFeedback(const iri_common_drivers_msgs::tool_closeFeedbackConstPtr& feedback)
{
  alg_.lock();
  ROS_INFO("TableClearingExecuteAlgNode::close_gripperFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    close_gripper_client_.cancelGoal();
    //ROS_INFO("TableClearingExecuteAlgNode::close_gripperFeedback: Cancelling Action!");
  }
  alg_.unlock();
}

void TableClearingExecuteAlgNode::open_gripperDone(const actionlib::SimpleClientGoalState& state,  const iri_common_drivers_msgs::tool_openResultConstPtr& result)
{
  alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
  {
    ROS_INFO("TableClearingExecuteAlgNode::open_gripperDone: Goal Achieved!");
    gripper_open = true;
  }
  else
    ROS_INFO("TableClearingExecuteAlgNode::open_gripperDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingExecuteAlgNode::open_gripperActive()
{
  alg_.lock();
  ROS_INFO("TableClearingExecuteAlgNode::open_gripperActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingExecuteAlgNode::open_gripperFeedback(const iri_common_drivers_msgs::tool_openFeedbackConstPtr& feedback)
{
  alg_.lock();
  ROS_INFO("TableClearingExecuteAlgNode::open_gripperFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    open_gripper_client_.cancelGoal();
    //ROS_INFO("TableClearingExecuteAlgNode::open_gripperFeedback: Cancelling Action!");
  }
  alg_.unlock();
}


/*  [action requests] */
bool TableClearingExecuteAlgNode::close_gripperMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  this->alg_.unlock();
  if(close_gripper_client_.isServerConnected())
  {
    //ROS_DEBUG("TableClearingExecuteAlgNode::close_gripperMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //close_gripper_goal_.data = my_desired_goal;
    close_gripper_client_.sendGoal(close_gripper_goal_,
                boost::bind(&TableClearingExecuteAlgNode::close_gripperDone,     this, _1, _2),
                boost::bind(&TableClearingExecuteAlgNode::close_gripperActive,   this),
                boost::bind(&TableClearingExecuteAlgNode::close_gripperFeedback, this, _1));
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingExecuteAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    // this->alg_.lock();
    // ROS_DEBUG("TableClearingExecuteAlgNode::close_gripperMakeActionRequest: HRI server is not connected");
    return false;
  }
}

bool TableClearingExecuteAlgNode::open_gripperMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  this->alg_.unlock();
  if(open_gripper_client_.isServerConnected())
  {
    //ROS_DEBUG("TableClearingExecuteAlgNode::open_gripperMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //open_gripper_goal_.data = my_desired_goal;
    open_gripper_client_.sendGoal(open_gripper_goal_,
                boost::bind(&TableClearingExecuteAlgNode::open_gripperDone,     this, _1, _2),
                boost::bind(&TableClearingExecuteAlgNode::open_gripperActive,   this),
                boost::bind(&TableClearingExecuteAlgNode::open_gripperFeedback, this, _1));
    //this->alg_.lock();
    ROS_DEBUG("TableClearingExecuteAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    //this->alg_.lock();
    ROS_DEBUG("TableClearingExecuteAlgNode::open_gripperMakeActionRequest: HRI server is not connected");
    return false;
  }
}


void TableClearingExecuteAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TableClearingExecuteAlgNode::testTrajectory()
{
  // while(!traj_client_->waitForServer(ros::Duration(5.0))){
  //               ROS_INFO("Waiting for the joint_trajectory_action server");
  // }

  control_msgs::FollowJointTrajectoryGoal goal;
  // frame_id
  goal.trajectory.header.frame_id = "/estirabot_link_footprint";
  // joint names
  goal.trajectory.joint_names.resize(7);
  goal.trajectory.joint_names[0] = "estirabot_joint_1";
  goal.trajectory.joint_names[1] = "estirabot_joint_2";
  goal.trajectory.joint_names[2] = "estirabot_joint_3";
  goal.trajectory.joint_names[3] = "estirabot_joint_4";
  goal.trajectory.joint_names[4] = "estirabot_joint_5";
  goal.trajectory.joint_names[5] = "estirabot_joint_6";
  goal.trajectory.joint_names[6] = "estirabot_joint_7";

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);
  goal.trajectory.points[0].positions[0] = 0.0;
  goal.trajectory.points[0].positions[1] = 0.0;
  goal.trajectory.points[0].positions[2] = 0.5;
  goal.trajectory.points[0].positions[3] = 0.0;
  goal.trajectory.points[0].positions[4] = 0.0;
  goal.trajectory.points[0].positions[5] = 0.5;
  goal.trajectory.points[0].positions[6] = 0.0;
  goal.trajectory.points[0].velocities.resize(7);
  goal.trajectory.points[0].velocities[0] = 0.0f;
  goal.trajectory.points[0].velocities[1] = 0.0f;
  goal.trajectory.points[0].velocities[2] = 0.0f;
  goal.trajectory.points[0].velocities[3] = 0.0f;
  goal.trajectory.points[0].velocities[4] = 0.0f;
  goal.trajectory.points[0].velocities[5] = 0.0f;
  goal.trajectory.points[0].velocities[6] = 0.0f;
  goal.trajectory.points[0].accelerations.resize(7);
  goal.trajectory.points[0].accelerations[0] = 0.0f;
  goal.trajectory.points[0].accelerations[1] = 0.0f;
  goal.trajectory.points[0].accelerations[2] = 0.0f;
  goal.trajectory.points[0].accelerations[3] = 0.0f;
  goal.trajectory.points[0].accelerations[4] = 0.0f;
  goal.trajectory.points[0].accelerations[5] = 0.0f;
  goal.trajectory.points[0].accelerations[6] = 0.0f;
  goal.trajectory.points[0].time_from_start = ros::Duration(2); // in a trajectory with mor epoints, each point should have a different time stamp

  

  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ROS_INFO("Going home");
  // traj_client_->sendGoal(goal);
  // if (!traj_client_->waitForResult(ros::Duration(10.0)))
  // { 
  //     traj_client_->cancelGoal();
  //     ROS_INFO("Action did not finish before the time out.\n"); 
  // }

  sleep(3.0);
}
void TableClearingExecuteAlgNode::testIK()
{
   
}

trajectory_msgs::JointTrajectoryPoint TableClearingExecuteAlgNode::setTrajectoryPoint(sensor_msgs::JointState joint_state, double secs_)
{

  trajectory_msgs::JointTrajectoryPoint joints_trajectory_point;

  joints_trajectory_point.positions.resize(7);
  joints_trajectory_point.positions[0] = joint_state.position[0];
  joints_trajectory_point.positions[1] = joint_state.position[1];
  joints_trajectory_point.positions[2] = joint_state.position[2];
  joints_trajectory_point.positions[3] = joint_state.position[3];
  joints_trajectory_point.positions[4] = joint_state.position[4];
  joints_trajectory_point.positions[5] = joint_state.position[5];
  joints_trajectory_point.positions[6] = joint_state.position[6];

  joints_trajectory_point.velocities.resize(7);
  joints_trajectory_point.velocities[0] = 0.02f;
  joints_trajectory_point.velocities[1] = 0.02f;
  joints_trajectory_point.velocities[2] = 0.02f;
  joints_trajectory_point.velocities[3] = 0.02f;
  joints_trajectory_point.velocities[5] = 0.02f;
  joints_trajectory_point.velocities[4] = 0.02f;
  joints_trajectory_point.velocities[6] = 0.02f;
  joints_trajectory_point.accelerations.resize(7);
  joints_trajectory_point.accelerations[0] = 0.0f;
  joints_trajectory_point.accelerations[1] = 0.0f;
  joints_trajectory_point.accelerations[2] = 0.0f;
  joints_trajectory_point.accelerations[3] = 0.0f;
  joints_trajectory_point.accelerations[4] = 0.0f;
  joints_trajectory_point.accelerations[5] = 0.0f;
  joints_trajectory_point.accelerations[6] = 0.0f;

  joints_trajectory_point.time_from_start = ros::Duration(secs_);

  return joints_trajectory_point;
}

bool TableClearingExecuteAlgNode::askForUserInput(std::string text)
{
  if(this->alg_.automatic == MANUAL_EXECUTION)
  {
    char response;
    bool wrong_character = true;
    while(wrong_character)
    {
      std::cout << "\n\n " << text << ". Do you want to execute?(y,n)";
      std::cin >> response;
      switch(response)
      {
        case 'y':
        case 'Y':
            wrong_character = false;
            std::cout << "\n";
            return true;
            break;
        case 'n':
        case 'N':
            std::cout << "\nYou decided to NOT execute the action\n";
            return false; // return false because the trajectory is not executed
            break;
        default: break;
      }
    }
  } 
  else  
    return true;
}

bool TableClearingExecuteAlgNode::move2JointsPose(sensor_msgs::JointState joint_state,double velocity , double acceleration)
{
  move_joints_srv_.request.positions = joint_state.position;
  move_joints_srv_.request.velocity = 0.5;
  move_joints_srv_.request.acceleration = 0.5;
  
  if (move_joints_client_.call(move_joints_srv_)) 
  {
    ROS_INFO("Movement in joints succedded");
    return true;
  }
  else
  {
    ROS_ERROR("Impossible calling %s",move_joints_client_.getService().c_str());
    return false;
  } 
}
bool TableClearingExecuteAlgNode::atHomePosition()
{
  return move2JointsPose(this->alg_.home_joint_state,0,0);
}
void TableClearingExecuteAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{

  return algorithm_base::main<TableClearingExecuteAlgNode>(argc, argv, "table_clearing_execute_alg_node");
}
