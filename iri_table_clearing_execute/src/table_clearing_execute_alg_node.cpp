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
            << "execution: " << execution_str << std::endl;

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
  estirabot_gripper_ik_client_ = this->public_node_handle_.serviceClient<iri_common_drivers_msgs::QueryInverseKinematics>(ik_service);

  estirabot_gripper_ik_from_pose_client_ = this->public_node_handle_.serviceClient<iri_wam_common_msgs::QueryWamInverseKinematicsFromPose>(from_pose_ik_service);

  

  // [init action servers]
  
  // [init action clients]
  traj_client_ = new TrajClient("/estirabot/estirabot_controller/follow_joint_trajectory", true);


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

  // Uncomment the following line to publish the topic message
  //this->action_pose_publisher_.publish(this->action_pose_PoseStamped_msg_);

}

/*  [subscriber callbacks] */
void TableClearingExecuteAlgNode::current_joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ROS_INFO("TableClearingExecuteAlgNode::current_joint_state_callback: New Message Received");

  this->alg_.current_joint_state_ = *msg;

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
  action_pose_publisher_.publish(req.approaching_pose);

  //ROS_INFO("TableClearingExecuteAlgNode::execute_graspingCallback: Processing New Request!");
  iri_wam_common_msgs::QueryWamInverseKinematicsFromPose srv;
  srv.request.current_joints = this->alg_.home_joint_state;
  srv.request.desired_pose = req.approaching_pose;

  std::vector<sensor_msgs::JointState> joints_trajectory;
  joints_trajectory.resize(4);

  res.success = true;
  for (uint i = 0; i < 4; ++i)
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
        srv.request.desired_pose = req.pre_dropping_pose;
        srv.request.desired_pose.header.frame_id =  "/estirabot_link_footprint";
        break;
      case 3:
        srv.request.current_joints = joints_trajectory[2];
        srv.request.desired_pose = req.dropping_pose;
        srv.request.desired_pose.header.frame_id =  "/estirabot_link_footprint";
        break;
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
          ROS_ERROR("Impossible calling %s service or solution not found for pre dropping pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
          break;
        case 3:
          ROS_ERROR("Impossible calling %s service or solution not found for dropping pose",estirabot_gripper_ik_from_pose_client_.getService().c_str());
          break;
        default:break;        
      }
      res.success = false; 
      return true;
    }
  }

  ROS_INFO("Waiting for the joint_trajectory_action server");
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
  }
  
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


  
  // Go to pregrasping pose - Approaching pose
  ROS_INFO("Going to pre grasping pose");
  this->alg_.goToPose(traj_client_,joints_trajectory[0]);
  
  // OPEN THE GRIPPER
  if(this->real_robot)
  { 
    ROS_INFO("Opening gripper");
    this->open_gripperMakeActionRequest();
    ros::Duration(1).sleep(); // sleep for a second
  }

  // Go to grasping pose
  ROS_INFO("Going to grasping pose");
  this->alg_.goToPose(traj_client_,joints_trajectory[1]);
  ros::Duration(1).sleep(); // sleep for a second

  // CLOSE THE GRIPPER
  if(this->real_robot)
  { 
    ROS_INFO("Closing gripper");
    this->close_gripperMakeActionRequest();
    ros::Duration(1).sleep(); // sleep for a second
  }

  // Go to Pregrasping pose again
  ROS_INFO("Going to pre grasping pose again");
  this->alg_.goToPose(traj_client_,joints_trajectory[0]);
  ros::Duration(1).sleep(); // sleep for a second

  // Go to bin
  ROS_INFO("Going to pre dropping pose");
  this->alg_.goToPose(traj_client_,joints_trajectory[2]);
  ros::Duration(1).sleep(); // sleep for a second

  ROS_INFO("Going to dropping pose");
  this->alg_.goToPose(traj_client_,joints_trajectory[3]);
  ros::Duration(1).sleep(); // sleep for a second

  // OPEN GRIPPER
  if(this->real_robot)
  { 
    ROS_INFO("Opening gripper");
    this->open_gripperMakeActionRequest();
    ros::Duration(1).sleep(); // sleep for a second
  }

  // wait 
  ros::Duration(1).sleep(); // sleep for a second

  // CLOSE GRIPPER
  if(this->real_robot)
  { 
    ROS_INFO("Closing gripper");
    this->close_gripperMakeActionRequest();
    ros::Duration(1).sleep(); // sleep for a second
  }
  
  // Go home
  ROS_INFO("Going home");
  this->alg_.goHome(traj_client_);

  //unlock previously blocked shared variables
  //this->execute_grasping_mutex_exit();
  //this->alg_.unlock();

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
  
  srv.request.current_joints = this->alg_.home_joint_state;
  //srv.request.current_joints = this->alg_.current_joint_state_;

  srv.request.desired_pose = req.pushing_cartesian_trajectory[0];

  ROS_INFO("Trying calling the IK service");
  if (estirabot_gripper_ik_from_pose_client_.call(srv))
  {
    joints_trajectory[0] = srv.response.desired_joints;
    std::cout << "Point " << 0 << 
        " joint 1: " << joints_trajectory[0].position[0] <<
        " joint 2: " << joints_trajectory[0].position[1] <<
        " joint 3: " << joints_trajectory[0].position[2] <<
        " joint 4: " << joints_trajectory[0].position[3] <<
        " joint 5: " << joints_trajectory[0].position[4] <<
        " joint 6: " << joints_trajectory[0].position[5] <<
        " joint 7: " << joints_trajectory[0].position[6] << std::endl;
  }
  else
  {
    ROS_ERROR("Impossible calling %s service or solution not found",estirabot_gripper_ik_from_pose_client_.getService().c_str());
    res.success = false; 
    return true;
  }

  res.success = true; 
  for (int i = 1; i < req.pushing_cartesian_trajectory.size(); ++i)
  {
    srv.request.current_joints = joints_trajectory[i-1];
    srv.request.desired_pose = req.pushing_cartesian_trajectory[i];
    std::cout << "frame_id: " << req.pushing_cartesian_trajectory[i].header.frame_id << std::endl;
    srv.request.desired_pose.header.stamp = ros::Time::now();
    std::cout << "Requesting IK of x: " << req.pushing_cartesian_trajectory[i].pose.position.x << " y: " <<
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
      std::cout << "Point " << i << 
          " joint 1: " << joints_trajectory[i].position[0] <<
          " joint 2: " << joints_trajectory[i].position[1] <<
          " joint 3: " << joints_trajectory[i].position[2] <<
          " joint 4: " << joints_trajectory[i].position[3] <<
          " joint 5: " << joints_trajectory[i].position[4] <<
          " joint 6: " << joints_trajectory[i].position[5] <<
          " joint 7: " << joints_trajectory[i].position[6] << std::endl;
    }
    else
    {
      ROS_ERROR("Impossible calling %s service or solution not found",estirabot_gripper_ik_from_pose_client_.getService().c_str());
      res.success = false; 
      return true;
    }
  }

  ROS_INFO("Waiting for the joint_trajectory_action server");
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
  }
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
            return false; // return false because the trajectory is not executed
            break;
        default: break;
      }
    }
  } 

  // reset the time stamp for all the trajectory points
  for (int i = 0; i < joints_trajectory.size(); ++i)
    joints_trajectory[i].header.stamp = ros::Time::now();

  // joint names
  goal.trajectory.joint_names.resize(7);
  goal.trajectory.joint_names[0] = "estirabot_joint_1";
  goal.trajectory.joint_names[1] = "estirabot_joint_2";
  goal.trajectory.joint_names[2] = "estirabot_joint_3";
  goal.trajectory.joint_names[3] = "estirabot_joint_4";
  goal.trajectory.joint_names[4] = "estirabot_joint_5";
  goal.trajectory.joint_names[5] = "estirabot_joint_6";
  goal.trajectory.joint_names[6] = "estirabot_joint_7";  

  goal.trajectory.points.resize(joints_trajectory.size());
  for (int i = 0; i < joints_trajectory.size(); ++i)
  {
      goal.trajectory.points[i].positions.resize(7);
      goal.trajectory.points[i].positions[0] = joints_trajectory[i].position[0];
      goal.trajectory.points[i].positions[1] = joints_trajectory[i].position[1];
      goal.trajectory.points[i].positions[2] = joints_trajectory[i].position[2];
      goal.trajectory.points[i].positions[3] = joints_trajectory[i].position[3];
      goal.trajectory.points[i].positions[4] = joints_trajectory[i].position[4];
      goal.trajectory.points[i].positions[5] = joints_trajectory[i].position[5];
      goal.trajectory.points[i].positions[6] = joints_trajectory[i].position[6];

      std::cout << "Current joint state\n";
      std::cout << " joint 1: " << joints_trajectory[i].position[0] <<
      " joint 2: " << joints_trajectory[i].position[1] <<
      " joint 3: " << joints_trajectory[i].position[2] <<
      " joint 4: " << joints_trajectory[i].position[3] <<
      " joint 5: " << joints_trajectory[i].position[4] <<
      " joint 6: " << joints_trajectory[i].position[5] <<
      " joint 7: " << joints_trajectory[i].position[6] << std::endl;

      goal.trajectory.points[i].velocities.resize(7);
      goal.trajectory.points[i].velocities[0] = 0.02f;
      goal.trajectory.points[i].velocities[1] = 0.02f;
      goal.trajectory.points[i].velocities[2] = 0.02f;
      goal.trajectory.points[i].velocities[3] = 0.02f;
      goal.trajectory.points[i].velocities[5] = 0.02f;
      goal.trajectory.points[i].velocities[4] = 0.02f;
      goal.trajectory.points[i].velocities[6] = 0.02f;
      goal.trajectory.points[i].accelerations.resize(7);
      goal.trajectory.points[i].accelerations[0] = 0.0f;
      goal.trajectory.points[i].accelerations[1] = 0.0f;
      goal.trajectory.points[i].accelerations[2] = 0.0f;
      goal.trajectory.points[i].accelerations[3] = 0.0f;
      goal.trajectory.points[i].accelerations[4] = 0.0f;
      goal.trajectory.points[i].accelerations[5] = 0.0f;
      goal.trajectory.points[i].accelerations[6] = 0.0f;
      if(i == 0)
        goal.trajectory.points[i].time_from_start = ros::Duration(3); 
      else
        goal.trajectory.points[i].time_from_start = goal.trajectory.points[i-1].time_from_start + ros::Duration(1); 

  }

  
  trajectory_msgs::JointTrajectoryPoint final_point;

  // go back to the initial position in order to avoid to collide with the interesting object when coming home
  final_point.positions.resize(7);
  final_point.positions[0] = joints_trajectory[joints_trajectory.size()-2].position[0];
  final_point.positions[1] = joints_trajectory[joints_trajectory.size()-2].position[1];
  final_point.positions[2] = joints_trajectory[joints_trajectory.size()-2].position[2];
  final_point.positions[3] = joints_trajectory[joints_trajectory.size()-2].position[3];
  final_point.positions[5] = joints_trajectory[joints_trajectory.size()-2].position[5];
  final_point.positions[4] = joints_trajectory[joints_trajectory.size()-2].position[4];
  final_point.positions[5] = joints_trajectory[joints_trajectory.size()-2].position[5];
  final_point.positions[6] = joints_trajectory[joints_trajectory.size()-2].position[6];

  final_point.velocities.resize(7);
  final_point.velocities[0] = 0.02f;
  final_point.velocities[1] = 0.02f;
  final_point.velocities[2] = 0.02f;
  final_point.velocities[3] = 0.02f;
  final_point.velocities[5] = 0.02f;
  final_point.velocities[4] = 0.02f;
  final_point.velocities[6] = 0.02f;
  final_point.accelerations.resize(7);
  final_point.accelerations[0] = 0.0f;
  final_point.accelerations[1] = 0.0f;
  final_point.accelerations[2] = 0.0f;
  final_point.accelerations[3] = 0.0f;
  final_point.accelerations[4] = 0.0f;
  final_point.accelerations[5] = 0.0f;
  final_point.accelerations[6] = 0.0f;
  final_point.time_from_start = goal.trajectory.points[joints_trajectory.size() -1].time_from_start + ros::Duration(2); 

  goal.trajectory.points.push_back(final_point);  

  goal.trajectory.header.stamp = ros::Time::now();

  ROS_INFO("sending trajectory trajectory");
  traj_client_->sendGoal(goal);
  if (!traj_client_->waitForResult(ros::Duration(3 + 3.0f/joints_trajectory.size() + 4)))
  { 
      traj_client_->cancelGoal();
      ROS_INFO("Action did not finish before the time out.\n"); 
  }

  ROS_INFO("Action finished - going home");
  this->alg_.goHome(traj_client_);
  if (!traj_client_->waitForResult(ros::Duration(5)))
  { 
      traj_client_->cancelGoal();
      ROS_INFO("Action (Go Home) did not finish before the time out.\n"); 
  }


  //unlock previously blocked shared variables
  //this->execute_pushing_mutex_exit();
  //this->alg_.unlock();

  std::cout << "\nWaiting for new request...\n";

  return true;
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
    ROS_INFO("TableClearingExecuteAlgNode::close_gripperDone: Goal Achieved!");
  else
    ROS_INFO("TableClearingExecuteAlgNode::close_gripperDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingExecuteAlgNode::close_gripperActive()
{
  alg_.lock();
  //ROS_INFO("TableClearingExecuteAlgNode::close_gripperActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingExecuteAlgNode::close_gripperFeedback(const iri_common_drivers_msgs::tool_closeFeedbackConstPtr& feedback)
{
  alg_.lock();
  //ROS_INFO("TableClearingExecuteAlgNode::close_gripperFeedback: Got Feedback!");

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
    ROS_INFO("TableClearingExecuteAlgNode::open_gripperDone: Goal Achieved!");
  else
    ROS_INFO("TableClearingExecuteAlgNode::open_gripperDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TableClearingExecuteAlgNode::open_gripperActive()
{
  alg_.lock();
  //ROS_INFO("TableClearingExecuteAlgNode::open_gripperActive: Goal just went active!");
  alg_.unlock();
}

void TableClearingExecuteAlgNode::open_gripperFeedback(const iri_common_drivers_msgs::tool_openFeedbackConstPtr& feedback)
{
  alg_.lock();
  //ROS_INFO("TableClearingExecuteAlgNode::open_gripperFeedback: Got Feedback!");

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
  // this->alg_.unlock();
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
    this->alg_.lock();
    ROS_DEBUG("TableClearingExecuteAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    this->alg_.lock();
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
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
  }

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
  traj_client_->sendGoal(goal);
  if (!traj_client_->waitForResult(ros::Duration(10.0)))
  { 
      traj_client_->cancelGoal();
      ROS_INFO("Action did not finish before the time out.\n"); 
  }

  sleep(3.0);
}
void TableClearingExecuteAlgNode::testIK()
{
   
}

void TableClearingExecuteAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TableClearingExecuteAlgNode>(argc, argv, "table_clearing_execute_alg_node");
}
