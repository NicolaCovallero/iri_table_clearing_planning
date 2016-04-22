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

      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      ROS_INFO("sending trajectory");
      traj_client_->sendGoal(goal);
      if (!traj_client_->waitForResult(ros::Duration(4)))
      { 
          traj_client_->cancelGoal();
          ROS_INFO("Action did not finish before the time out.\n"); 
      }
}