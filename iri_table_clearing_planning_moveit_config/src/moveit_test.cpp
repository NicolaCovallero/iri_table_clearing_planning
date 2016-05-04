#include <ros/ros.h>
#include <ros/console.h>
#include <iri_common_drivers_msgs/QueryJointsMovement.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>


int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "reach_handle_pose");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  //####### IMPORTANT ######################################################
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // from: https://groups.google.com/forum/#!topic/moveit-users/IZmpzzKR2ko
  // It seems that .move() blocks waiting for callback which won't 
  // be processed unless you have an asynch spinner. So also .plan()
  // See also: http://aeswiki.datasys.swri.edu/rositraining/Exercises/3.6
  //########################################################################


  ros::Publisher clear_planning_scene_pub = node_handle.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1000);

  moveit::planning_interface::MoveGroup group("arm");

  //We will use the :planning_scene_interface:`PlanningSceneInterface` class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //Now, we call the planner to compute the plan and visualize it.
  //Note that we are just planning, not asking move_group to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setStartStateToCurrentState();
  group.setPlanningTime(30.);  

  ROS_ERROR("REACH_POSE ready");

  while(ros::ok())
  {

    ROS_INFO("Planning ...");

    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    group_variable_values[0] = 0.0;
    group_variable_values[1] = 0.5;
    group_variable_values[2] = 0.0;
    group_variable_values[3] = 0.5;
    group_variable_values[4] = 0.0;
    group_variable_values[5] = 0.5;
    group_variable_values[6] = 0.0;

    group.setJointValueTarget(group_variable_values);

    bool success = group.plan(my_plan); //here it is PLANNING
    if(success)
    {

      ROS_INFO("Plan found. Execution ...");
      group.execute(my_plan);
      ROS_INFO("Final position reached. Yeah, everyone is happy! [Sleeing 5 sec. zzzz....]");
      sleep(5.0);

      //······go to the initial position - planning in joint-space ······
      
      // get current joint values
      group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

      group_variable_values[0] = 0.0;
      group_variable_values[1] = 0.0;
      group_variable_values[2] = 0.0;
      group_variable_values[3] = 0.0;
      group_variable_values[4] = 0.0;
      group_variable_values[5] = 0.0;
      group_variable_values[6] = 0.0;
      group.setJointValueTarget(group_variable_values);
      group.plan(my_plan);
      group.execute(my_plan);
      sleep(5.0);

    }

    // update the scene plan
    moveit_msgs::PlanningScene msg;
    clear_planning_scene_pub.publish(msg);

  
    loop_rate.sleep();
  }

  return 0;
}