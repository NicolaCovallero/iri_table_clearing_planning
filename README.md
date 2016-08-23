# iri_table_clearing_planning #

This package is the ROS implementation of [table_clearing_planning](https://bitbucket.org/NicolaCov/table_clearing_planning) algorithm. 
All the process is controlled by the decision_maker node which does the following operations:

1. it subscribes to a *sensor_msgs::PointCloud2* topic,
2. segmenting it by calling the segmentation service provided by the the [iri_tos_supervoxels](https://github.com/NicolaCovallero/iri_tos_supervoxels) package
3. Getting the predicates (iri_table_clearing_predicates)
4. Getting the plan by using the service of [iri_fast_downward_wrapper](https://bitbucket.org/NicolaCov/iri_fast_downward_wrapper) package.
5. Evaluate the IK and execute the action if there is a solution for the IK.

IMPORTANT: the goal of the plan is simply setted as a string in the decision_maker node, this node controls all the process.

## How to use the package: ##
To simulate it in gazebo launch in 6 different terminals:
```
#!C++
$ roscore
$ roslaunch iri_table_clearing_gazebo estirabot_gripper.launch
$ roslaunch iri_table_clearing_predicates iri_table_clearing_redicates 
$ roslaunch iri_fast_downward_wrapper fast_downward_server.launch 
$ roslaunch iri_table_clearing_decision_maker iri_table_clearing_decision_maker.launch 
$ roslaunch iri_table_clearing_execute iri_table_clearing_execute.launch 

```
All the launch files have different options. In the predicates package you have to specify the dimension of the gripper, as well the pushing mode and the grasping mode.
Pushing mode:

1. Parallel to the plane
2. Orthogonal to the plane (DEFAULT)

Grasping mode:

1. Orthogonal to the plane 
2. Not orthogonal: the approaching direction is choosen accordingly the 3rd principal direction (DEFAULT)

In the iri_table_clearing_decision_maker.launch file you have a lot of options, the most important are:

1. FILTERING: to filter out the point cloud
2. SAVE_EXPERIMENT: to save data of the experiment (pcd the plan executed and the elapsed times for each phases)
3. AUTOMATIC_SAVE: when saving the data it is not prompting anything and all is automatic.
4. EXECUTION: if it consider the execution of the first action of the plan
5. REPEAT: automatic iteration of the algorithm, if it is false it waits for a command of the user before to repeat

In the iri_table_clearing_execute.launch file you can choose if the execution of the action is controlled by the user or all automatic, before to execute the pushing trajectory it asks to the users, and for the grasping action before to move to each pose and open/close the gripper asks to the user.

A rqt_gui is given, launch it with the command

```
#!c++

roslaunch iri_table_clearing_config rqt_gui.launch
```


To use it with a .pcd file run in another terminal:

```
#!c++

$ rosrun pcl_ros pcd_to_pointcloud name_file.pcd 

```


## EXPERIMENT COMPARISON ##
The code can be launched in order to reproduce a similar plan to the algorithm presented in the paper "A Framework for Push-grasping in Clutter" by Dogar and Srinivasa.
The part explain in this section talk about how to run the code to obtain a similar plan to their for a table clearing task. Pay ATTENTION that this is *not their algorithm*.
This part of the codes only attempts to generate a plan that should be the same, or at least similar, to their when solving a table clearing task, this code is used only to compare the final plan, not how the probabilities are taken into account or the planning time and other things, only the final plane.
To do so, to design the experiment you have to take into account that this code still is based in our algorithm, so you should pay attention how you design the experiments in order to get a plan similar to the one they would obtain.

The algorithm decides to grasp the objects with less objects that block it from being grasped, if there are more objects with the same number of blocking object the goal object is chosen randomly. Then the robots solve the problem to grasp that object, when that object has been grasped, a new object is chosen as goal in the same way until no objects stand on the table.
In this case, when the objects are moved they are no more pushed until they can be grasped but for a distance equal to the dimension of the manipulated obejct accordingly to the pushing direction(Again, pay attention how you design the experiment).  

In the main launch file "iri_table_clearing_decision_maker.launch" there is an argument:
```
#!c++
roslaunch iri_table_clearing_decision_maker iri_table_clearing_decision_maker.launch EXPERIMENT_COMPARISON:=True
roslaunch iri_table_clearing_predicates iri_table_clearing_predicates.launch PUSHING_UNTIL_GRASPABLE:=False
roslaunch iri_fast_downward_wrapper  iri_fast_downward_server_comparison.launch
```
Pay attention that in both the launch files the "pushing_step" parameter is the same.


## TODO 
Use the ros time to measure the time
