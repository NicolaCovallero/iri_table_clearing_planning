#include "test_trajectory_alg_node.h"

TestTrajectoryAlgNode::TestTrajectoryAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TestTrajectoryAlgorithm>(),
  trajectory_client_("trajectory", true)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  joints_move_client_ = this->public_node_handle_.serviceClient<iri_common_drivers_msgs::QueryJointsMovement>("joints_move");

  
  // [init action servers]
  
  // [init action clients]
}

TestTrajectoryAlgNode::~TestTrajectoryAlgNode(void)
{
  // [free dynamic memory]
}

void TestTrajectoryAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  //joints_move_srv_.request.data = my_var;
  //ROS_INFO("TestTrajectoryAlgNode:: Sending New Request!");
  //if (joints_move_client_.call(joints_move_srv_))
  //{
    //ROS_INFO("TestTrajectoryAlgNode:: Response: %s", joints_move_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TestTrajectoryAlgNode:: Failed to Call Server on topic joints_move ");
  //}


  
  // [fill action structure and make request to the action server]
  // variable to hold the state of the current goal on the server
  actionlib::SimpleClientGoalState trajectory_state(actionlib::SimpleClientGoalState::PENDING);
  // to get the state of the current goal
  //alg_.unlock();
  //trajectory_state=trajectory_client_.getState();
  // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
  //alg_.lock();

  iri_common_drivers_msgs::QueryJointsMovement srv;
  srv.request.positions.resize(7);
  srv.request.positions[0] = 0.0;
  srv.request.positions[1] = 0.0;
  srv.request.positions[2] = 0.0;
  srv.request.positions[3] = 0.0;
  srv.request.positions[4] = 0.0;
  srv.request.positions[5] = 0.0;
  srv.request.positions[6] = 0.0;
  srv.request.velocity = 0.5;
  srv.request.acceleration = 0.5;

  if(!joints_move_client_.call(srv))
  {
    ROS_ERROR("Error calling %s",joints_move_client_.getService().c_str());
    exit(0);
  }

  trajectoryMakeActionRequest();
  if(trajectory_state==actionlib::SimpleClientGoalState::ABORTED)
  {
    ROS_INFO("Action Failed");
    exit(0);
  }
  else if(trajectory_state==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Action succeded");
    exit(0);
  }
  //trajectoryMakeActionRequest();

  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).


  // [publish messages]


}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void TestTrajectoryAlgNode::trajectoryDone(const actionlib::SimpleClientGoalState& state,  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
    ROS_INFO("TestTrajectoryAlgNode::trajectoryDone: Goal Achieved!");
  else
    ROS_INFO("TestTrajectoryAlgNode::trajectoryDone: %s", state.toString().c_str());

  //copy & work with requested result
  alg_.unlock();
}

void TestTrajectoryAlgNode::trajectoryActive()
{
  alg_.lock();
  //ROS_INFO("TestTrajectoryAlgNode::trajectoryActive: Goal just went active!");
  alg_.unlock();
}

void TestTrajectoryAlgNode::trajectoryFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  alg_.lock();
  //ROS_INFO("TestTrajectoryAlgNode::trajectoryFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    trajectory_client_.cancelGoal();
    //ROS_INFO("TestTrajectoryAlgNode::trajectoryFeedback: Cancelling Action!");
  }
  alg_.unlock();
}


/*  [action requests] */
bool TestTrajectoryAlgNode::trajectoryMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  // this->alg_.unlock();
  if(trajectory_client_.isServerConnected())
  {
    //ROS_DEBUG("TestTrajectoryAlgNode::trajectoryMakeActionRequest: Server is Available!");
    //send a goal to the action server
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.frame_id = "/estirabot_link_footprint";


    goal.trajectory.joint_names.resize(7);
    goal.trajectory.joint_names[0] = "estirabot_joint_1";
    goal.trajectory.joint_names[1] = "estirabot_joint_2";
    goal.trajectory.joint_names[2] = "estirabot_joint_3";
    goal.trajectory.joint_names[3] = "estirabot_joint_4";
    goal.trajectory.joint_names[4] = "estirabot_joint_5";
    goal.trajectory.joint_names[5] = "estirabot_joint_6";
    goal.trajectory.joint_names[6] = "estirabot_joint_7";  

    goal.trajectory.points.resize(2);

    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    goal.trajectory.points[ind].velocities.resize(7);
    goal.trajectory.points[ind].accelerations.resize(7);
    goal.trajectory.points[ind].time_from_start = ros::Duration(0.0f);

    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 2.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    goal.trajectory.points[ind].velocities.resize(7);
    goal.trajectory.points[ind].accelerations.resize(7);
    goal.trajectory.points[ind].time_from_start = goal.trajectory.points[ind-1].time_from_start + ros::Duration(5.0f);

    goal.trajectory.points[0].time_from_start = ros::Duration(3); 

    goal.trajectory.header.stamp = ros::Time::now();

    trajectory_goal_ = goal;
    trajectory_client_.sendGoal(trajectory_goal_,
                boost::bind(&TestTrajectoryAlgNode::trajectoryDone,     this, _1, _2),
                boost::bind(&TestTrajectoryAlgNode::trajectoryActive,   this),
                boost::bind(&TestTrajectoryAlgNode::trajectoryFeedback, this, _1));
    // this->alg_.lock();
    // ROS_DEBUG("TestTrajectoryAlgNode::MakeActionRequest: Goal Sent.");
    return true;
  }
  else
  {
    // this->alg_.lock();
    // ROS_DEBUG("TestTrajectoryAlgNode::trajectoryMakeActionRequest: HRI server is not connected");
    return false;
  }
}


void TestTrajectoryAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TestTrajectoryAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TestTrajectoryAlgNode>(argc, argv, "test_trajectory_alg_node");
}
