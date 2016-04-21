#include "table_clearing_execute_alg_node.h"

TableClearingExecuteAlgNode::TableClearingExecuteAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingExecuteAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->action_pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("action_pose", 1);
  
  // [init subscribers]
  
  // [init services]
  this->execute_grasping_server_ = this->public_node_handle_.advertiseService("execute_grasping", &TableClearingExecuteAlgNode::execute_graspingCallback, this);
  pthread_mutex_init(&this->execute_grasping_mutex_,NULL);

  this->execute_pushing_server_ = this->public_node_handle_.advertiseService("execute_pushing", &TableClearingExecuteAlgNode::execute_pushingCallback, this);
  pthread_mutex_init(&this->execute_pushing_mutex_,NULL);

  
  // [init clients]
  estirabot_gripper_ik_client_ = this->public_node_handle_.serviceClient<iri_common_drivers_msgs::QueryInverseKinematics>("/estirabot/estirabot_tcp_ik/get_wam_ik");

  traj_client_ = new TrajClient("/estirabot/estirabot_controller/follow_joint_trajectory", true);

  // [init action servers]
  
  // [init action clients]
}

TableClearingExecuteAlgNode::~TableClearingExecuteAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->execute_grasping_mutex_);
  pthread_mutex_destroy(&this->execute_pushing_mutex_);
}

void TableClearingExecuteAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
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

  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->action_pose_publisher_.publish(this->action_pose_PoseStamped_msg_);

}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool TableClearingExecuteAlgNode::execute_graspingCallback(iri_table_clearing_execute::ExecuteGrasping::Request &req, iri_table_clearing_execute::ExecuteGrasping::Response &res)
{
  ROS_INFO("TableClearingExecuteAlgNode::execute_graspingCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->execute_grasping_mutex_enter();

  //ROS_INFO("TableClearingExecuteAlgNode::execute_graspingCallback: Processing New Request!");
  //do operations with req and output on res
  //res.data2 = req.data1 + my_var;

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

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->execute_pushing_mutex_enter();

  //ROS_INFO("TableClearingExecuteAlgNode::execute_pushingCallback: Processing New Request!");
  
  // ---------get the trajectory in the joint space -------------
  iri_common_drivers_msgs::QueryInverseKinematics srv;
  res.ik_pose_success.resize(req.pushing_cartesian_trajectory.size());
  std::vector<sensor_msgs::JointState> joints_trajectory;
  action_pose_publisher_.publish(req.pushing_cartesian_trajectory[0]);
  for (int i = 0; i < req.pushing_cartesian_trajectory.size(); ++i)
  {
    srv.request.pose = req.pushing_cartesian_trajectory[i];
    std::cout << "frame_id: " << req.pushing_cartesian_trajectory[i].header.frame_id << std::endl;
    srv.request.pose.header.stamp = ros::Time::now();
    std::cout << "Requesting IK of x: " << req.pushing_cartesian_trajectory[i].pose.position.x << " y: " <<
                                            req.pushing_cartesian_trajectory[i].pose.position.y << " z: " <<
                                             req.pushing_cartesian_trajectory[i].pose.position.z << std::endl;
    if (estirabot_gripper_ik_client_.call(srv))
    {
      ROS_INFO("Inverse Kinematics done");
      joints_trajectory.push_back(srv.response.joints);       
      std::cout << "joint 1: " << joints_trajectory[i].position[0] <<
          "joint 2: " << joints_trajectory[i].position[1] <<
          "joint 3: " << joints_trajectory[i].position[2] <<
          "joint 4: " << joints_trajectory[i].position[3] <<
          "joint 5: " << joints_trajectory[i].position[4] <<
          "joint 6: " << joints_trajectory[i].position[5] <<
          "joint 7: " << joints_trajectory[i].position[6] << std::endl;
      res.ik_pose_success[i] = true;
    }
    else
    {
      ROS_ERROR("Failed to call service /estirabot/estirabot_tcp_ik/get_wam_ik");
      res.ik_pose_success[i] = false;
      res.success = false; 
    }
  }


  //unlock previously blocked shared variables
  //this->execute_pushing_mutex_exit();
  //this->alg_.unlock();

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

/*  [action requests] */

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
