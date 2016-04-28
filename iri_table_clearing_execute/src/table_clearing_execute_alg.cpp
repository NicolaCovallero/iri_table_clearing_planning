#include "table_clearing_execute_alg.h"

TableClearingExecuteAlgorithm::TableClearingExecuteAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TableClearingExecuteAlgorithm::~TableClearingExecuteAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TableClearingExecuteAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}



// TableClearingExecuteAlgorithm Public API

bool TableClearingExecuteAlgorithm::isAtHome(double th)
{
  double error; // square error 
  if(current_joint_state_.position.size() == 0)
  {
    ROS_ERROR("current_joint_state_ not initialized");
    return false;
  }
  for (int i = 0; i < 7; ++i)
  {
    error = abs(pow(current_joint_state_.position[i] ,2));
    if(error > th)
      return false;
  } 
  return true;
}

void TableClearingExecuteAlgorithm::goHome(TrajClient* traj_client_)
{
	  control_msgs::FollowJointTrajectoryGoal goal;
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
    goal.trajectory.points[0].positions[0] = 0.0f;
    goal.trajectory.points[0].positions[1] = 0.0f;
    goal.trajectory.points[0].positions[2] = 0.0f;
    goal.trajectory.points[0].positions[3] = 0.0f;
    goal.trajectory.points[0].positions[4] = 0.0f;
    goal.trajectory.points[0].positions[5] = 0.0f;
    goal.trajectory.points[0].positions[6] = 0.0f;
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
    goal.trajectory.points[0].time_from_start = ros::Duration(2); 

    // uint counter = 0;
    // while(!this->isAtHome())
    // {
      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      ROS_INFO("sending trajectory");
      traj_client_->sendGoal(goal);
      if (!traj_client_->waitForResult(ros::Duration(4)))
      { 
          traj_client_->cancelGoal();
          ROS_INFO("Action did not finish before the time out.\n"); 
      }
    //   counter++;
    //   if(counter >= 5)
    //   {
    //     ROS_ERROR("Attempted 5 times to go home: Something is going wrong.");
    //     return;
    //   }
    // }
    ROS_INFO("Estirabot is at home sweet home :)");
}
void TableClearingExecuteAlgorithm::goToPose(TrajClient* traj_client_, sensor_msgs::JointState joint_state)
{

  control_msgs::FollowJointTrajectoryGoal goal;
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
  goal.trajectory.points[0].positions[0] = joint_state.position[0];
  goal.trajectory.points[0].positions[1] = joint_state.position[1];
  goal.trajectory.points[0].positions[2] = joint_state.position[2];
  goal.trajectory.points[0].positions[3] = joint_state.position[3];
  goal.trajectory.points[0].positions[4] = joint_state.position[4];
  goal.trajectory.points[0].positions[5] = joint_state.position[5];
  goal.trajectory.points[0].positions[6] = joint_state.position[6];

  goal.trajectory.points[0].velocities.resize(7);
  goal.trajectory.points[0].velocities[0] = 0.02f;
  goal.trajectory.points[0].velocities[1] = 0.02f;
  goal.trajectory.points[0].velocities[2] = 0.02f;
  goal.trajectory.points[0].velocities[3] = 0.02f;
  goal.trajectory.points[0].velocities[5] = 0.02f;
  goal.trajectory.points[0].velocities[4] = 0.02f;
  goal.trajectory.points[0].velocities[6] = 0.02f;
  goal.trajectory.points[0].accelerations.resize(7);
  goal.trajectory.points[0].accelerations[0] = 0.0f;
  goal.trajectory.points[0].accelerations[1] = 0.0f;
  goal.trajectory.points[0].accelerations[2] = 0.0f;
  goal.trajectory.points[0].accelerations[3] = 0.0f;
  goal.trajectory.points[0].accelerations[4] = 0.0f;
  goal.trajectory.points[0].accelerations[5] = 0.0f;
  goal.trajectory.points[0].accelerations[6] = 0.0f;
  goal.trajectory.points[0].time_from_start = ros::Duration(3); 

  goal.trajectory.header.stamp = ros::Time::now();
  ROS_INFO("sending trajectory");
  traj_client_->sendGoal(goal);
  if (!traj_client_->waitForResult(ros::Duration(4)))
  { 
      traj_client_->cancelGoal();
      ROS_INFO("Action did not finish before the time out.\n"); 
  }

}
