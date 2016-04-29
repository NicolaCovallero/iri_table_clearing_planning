# iri_table_clearing_planning #

This package is the ROS implementation of [table_clearing_planning](https://bitbucket.org/NicolaCov/table_clearing_planning) algorithm. 
Until know:
1. it subscribes to a *sensor_msgs::PointCloud2* topic,
2. segmenting it by calling the segmentation service provided by the the [iri_tos_supervoxels](https://github.com/NicolaCovallero/iri_tos_supervoxels) package
3. Getting the predicates 
4. Getting the plan by using the service of [iri_fast_downward_wrapper](https://bitbucket.org/NicolaCov/iri_fast_downward_wrapper) package.

IMPORTANT: the goal of the plan is simply setted as a string in the decision_maker node, this node controls all the process.

## How to use the package: ##
To use it open 3 different terminals and launch:

```
#!C++

$ rosrun iri_table_clearing_predicates iri_table_clearingredicates 
$ roslaunch iri_fast_downward_wrapper fast_downward_server.launch 
$ roslaunch iri_table_clearing_decision_maker decision_maker.launch 

```
To use it with a .pcd file run in another terminal:

```
#!c++

$ rosrun pcl_ros pcd_to_pointcloud name_file.pcd 

```


## TODO: ## 
1. create a function to predict the states by following the plan.
2. filter cloud
3. collision checking along the path for the EE
