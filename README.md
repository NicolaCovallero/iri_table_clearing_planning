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


## TODO: ## 
1. collision checking along the path for the EE
2. add costs