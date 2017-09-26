#include "table_clearing_predicates_alg_node.h"

TableClearingPredicatesAlgNode::TableClearingPredicatesAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingPredicatesAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 30;//in [Hz]

  double  opening_width, finger_width, gripper_height, closing_region_height, closing_width,
          finger_deep, pushing_distance_plane, ee_height, ee_deep, pushing_step, 
          pushing_object_distance, approaching_distance;
  int pushing_method, on_th1,on_th2;

  this->public_node_handle_.param("pushing_method",pushing_method,PUSHING_METHOD);
  this->public_node_handle_.param("closing_width",closing_width,CLOSING_WIDTH);
  this->public_node_handle_.param("opening_width",opening_width,OPENING_WIDTH);
  this->public_node_handle_.param("finger_width",finger_width,FINGER_WIDTH);
  this->public_node_handle_.param("gripper_height",gripper_height,GRIPPER_HEIGHT);
  this->public_node_handle_.param("closing_region_height",closing_region_height,CLOSING_REGION_HEIGHT);
  this->public_node_handle_.param("finger_deep",finger_deep,FINGER_DEEP);
  this->public_node_handle_.param("pushing_distance_plane",pushing_distance_plane,PUSHING_DISTANCE_PLANE);
  this->public_node_handle_.param("ee_height",ee_height,EE_HEIGHT);
  this->public_node_handle_.param("ee_deep",ee_deep,EE_DEEP);
  this->public_node_handle_.param("pushing_step",pushing_step,PUSHING_STEP);
  this->public_node_handle_.param("pushing_object_distance",pushing_object_distance,PUSHING_OBJECT_DISTANCE);
  this->public_node_handle_.param("approaching_distance",approaching_distance,APPROACHING_DISTANCE);
  this->public_node_handle_.param("pushing_length_limit",alg_.pushing_length_limit,PUSHING_LENGTH_LIMIT);
  this->public_node_handle_.param("resolution",alg_.resolution,RESOLUTION);  
  this->public_node_handle_.param("minimum_distance",alg_.minimum_distance,MINIMUM_DISTANCE);  
  this->public_node_handle_.param("pushing_until_graspable", this->alg_.pushing_until_graspable, true);
  this->public_node_handle_.param("on_th1", on_th1, 100);
  this->public_node_handle_.param("on_th2", on_th2, 100);


  this->public_node_handle_.param("refine_segmented_objects",refine_segmented_objects,REFINE_SEGMENTED_OBJECTS);

  std::string pushing_until_graspable_str = "False";
  if(this->alg_.pushing_until_graspable)
    pushing_until_graspable_str = "True";

  std::string pushing_method_str;
  if(pushing_method == 0)
    pushing_method_str = "Orthogonal to the table plane";
  else
    pushing_method_str = "Parallel to the table plane";

  std::cout << "Parameters set: \n"
            << "opening_width: " << opening_width << std::endl
            << "closing_width: " << closing_width << std::endl
            << "finger_width: " << finger_width << std::endl
            << "gripper_height: " << gripper_height << std::endl
            << "closing_region_height: " << closing_region_height << std::endl
            << "finger_deep: " << finger_deep << std::endl
            << "pushing_distance_plane: " << pushing_distance_plane << std::endl
            << "ee_height: " << ee_height << std::endl
            << "ee_deep: " << ee_deep << std::endl
            << "pushing_step: " << pushing_step << std::endl
            << "pushing_object_distance: " << pushing_object_distance << std::endl
            << "pushing_method: " << pushing_method_str << " [" << pushing_method << "]" << std::endl
            << "approaching_distance: " << approaching_distance << std::endl 
            << "pushing_length_limit: " << alg_.pushing_length_limit << std::endl 
            << "resolution: " << alg_.resolution << std::endl
            << "minimum_distance: " << alg_.minimum_distance << std::endl
            << "pushing_until_graspable: " << pushing_until_graspable_str << std::endl
            << "refine_segmented_objects: " << refine_segmented_objects << std::endl
            << "on_th1: " << on_th1 << std::endl
            << "on_th2: " << on_th2 << std::endl;
            

  this->alg_.setPushingStep(pushing_step);
  this->alg_.setApproachingDistance(approaching_distance);
  this->alg_.setPushingMethod(pushing_method);
  this->alg_.setPushingObjectDistance(pushing_object_distance);
  this->alg_.setGripperSimpleModel(ee_height, ee_deep, opening_width + 2 * finger_width, pushing_distance_plane);         
  this->alg_.setGripperModel(opening_width, closing_width, finger_width, finger_deep, gripper_height, closing_region_height);
  this->alg_.setOnTopParameters(on_th1,on_th2);

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  this->get_symbolic_predicates_server_ = this->public_node_handle_.advertiseService("get_symbolic_predicates", &TableClearingPredicatesAlgNode::get_symbolic_predicatesCallback, this);
  pthread_mutex_init(&this->get_symbolic_predicates_mutex_,NULL);

  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TableClearingPredicatesAlgNode::~TableClearingPredicatesAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->get_symbolic_predicates_mutex_);
}

void TableClearingPredicatesAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool TableClearingPredicatesAlgNode::get_symbolic_predicatesCallback(iri_table_clearing_predicates::Predicates::Request &req, iri_table_clearing_predicates::Predicates::Response &res)
{
  ROS_INFO("TableClearingPredicatesAlgNode::get_symbolic_predicatesCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->get_symbolic_predicates_mutex_enter();

  ROS_INFO("TableClearingPredicatesAlgNode::get_symbolic_predicatesCallback: Processing New Request!");

  double t_1 = ros::Time().now().toSec();

  std::cout << "Converting the pointcloud format ... \n";
  // process the request  
  pcl::PointCloud<pcl::PointXYZRGBA> original_cloud;
  pcl::fromROSMsg(req.original_cloud,original_cloud);
  
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > segmented_objects;
  segmented_objects.resize(req.segmented_objects.size());
  std::vector<sensor_msgs::PointCloud2> segmented_objects_msg;
  for (int i = 0; i < req.segmented_objects.size(); ++i)
  {
    pcl::fromROSMsg(req.segmented_objects[i],segmented_objects[i]);
  }

  double t_2 = ros::Time().now().toSec();

  pcl::ModelCoefficients plane_coefficients;
  plane_coefficients.values.resize(4);
  plane_coefficients.values[0] = req.plane_coefficients.a;
  plane_coefficients.values[1] = req.plane_coefficients.b;
  plane_coefficients.values[2] = req.plane_coefficients.c;
  plane_coefficients.values[3] = req.plane_coefficients.d;

  // set the necessary stuff for the algorithm
  std::cout << "Initializing the perception module ... \n";
  this->alg_.setOriginalPointCloud(original_cloud);
  this->alg_.setObjectsPointCloud(segmented_objects);
  this->alg_.setPlaneCoefficients(plane_coefficients);
  
  if(refine_segmented_objects)
  {
    std::cout << "Refining objects...";
    double sec_ini = ros::Time().now().toSec();
    this->alg_.refineSegmentationByBiggestPlane();
    double sec_end = ros::Time().now().toSec();
    std::cout << " Elapsed time for refining objects: " << sec_end - sec_ini << std::endl;
  }
  

  double t_3 = ros::Time().now().toSec();
  // compute the predicates
  //std::clock_t t_init_predicates = std::clock();
  this->alg_.computeProjectionsOnTable();
  this->alg_.computeRichConvexHulls();
  this->alg_.computePrincipalDirections();
  this->alg_.computeOBBObjects(true);
  this->alg_.computeSimpleHeuristicGraspingPoses();

  this->alg_.computeBlockPredicates(false);
  this->alg_.computeOnTopPredicates(on_th1, on_th2, true);
  this->alg_.computeBlockGraspPredicates(true);

  // get the execution times
  ExecutionTimes exe_times = this->alg_.getExecutionTimes();

  // prepare the output
  res.block_predicates = this->alg_.getBlockPredicates();
  res.on_top_predicates = this->alg_.getOnTopPredicates();
  res.block_grasp_predicates = this->alg_.getBlockGraspPredicates();
  res.objects_pushing_directions = this->alg_.getPushingDirections();
  res.grasping_poses = this->alg_.getGraspingPoses();
  res.approaching_poses = this->alg_.getApproachingPoses();
  res.pushing_poses = this->alg_.getPushingPoses();
  res.obbs = this->alg_.getOBBMsg();
  res.centroids = this->alg_.getCentroids();
  res.principal_directions = this->alg_.getPrincipalDirections();
  res.pushing_object_distance = this->alg_.getPushingObjectDistance();
  res.pushing_lengths = this->alg_.getPushingLengths();
  res.pushing_grasping_poses = this->alg_.getPushingGraspingPoses();
  res.pushing_until_graspable= alg_.pushing_until_graspable;

  // set times
  res.on_predicates_time = exe_times.on_predicates;
  res.block_predicates_time = exe_times.block_predicates;
  res.block_grasp_predicates_time = exe_times.block_grasp_predicates;
  res.objects_collisions_time = exe_times.objects_collisions;
  res.ee_collisions_time = exe_times.ee_collisions;
  res.average_objects_collision_time = exe_times.average_objects_collision;
  res.average_ee_collision_time = exe_times.average_ee_collision;
  res.predicates_time = ros::Time().now().toSec() - t_3; 
 

  for (int i = 0; i < res.grasping_poses.size(); ++i)
  {
    for (int g = 0; g < res.grasping_poses[i].grasping_poses.size(); ++g)
    {
      res.grasping_poses[i].grasping_poses[g].header = req.original_cloud.header;  
    }
  }
  this->alg_.reset();// free the memory of the class

  //unlock previously blocked shared variables
  //this->get_symbolic_predicates_mutex_exit();
  //this->alg_.unlock();

  std::cout << "Time to convert from sensor_msgs::PointCloud2 to pcl::PointCloud: " << t_2 - t_1 << std::endl;
  std::cout << "Time to refine objetcs : " << t_3 - t_2 << std::endl;
  std::cout << "Time to compute the predicates : " << res.predicates_time << std::endl;

  return true;
}

void TableClearingPredicatesAlgNode::get_symbolic_predicates_mutex_enter(void)
{
  pthread_mutex_lock(&this->get_symbolic_predicates_mutex_);
}

void TableClearingPredicatesAlgNode::get_symbolic_predicates_mutex_exit(void)
{
  pthread_mutex_unlock(&this->get_symbolic_predicates_mutex_);
}


/*  [action callbacks] */

/*  [action requests] */

void TableClearingPredicatesAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->config_=config;
  this->alg_.setOnTopParameters(config.on_th1,config.on_th2);

  // this->alg_.setGripperSimpleModel( config.ee_height, config.ee_deep,
  //                                   config.opening_width + 2 * config.finger_width,
  //                                   config.pushing_distance_plane);         
  // this->alg_.setFingersModel(config.opening_width, config.finger_width, config.finger_deep,
  //                            config.gripper_height, config.closing_region_height);

  // std::cout << "Dynamic Reconfigure\n";
  // std::cout << "Parameters set: \n"
  //           << "opening_width: " << config.opening_width << std::endl
  //           << "finger_width: " << config.finger_width << std::endl
  //           << "gripper_height: " << config.gripper_height << std::endl
  //           << "closing_region_height: " << config.closing_region_height << std::endl
  //           << "finger_deep: " << config.finger_deep << std::endl
  //           << "pushing_distance_plane: " << config.pushing_distance_plane << std::endl
  //           << "ee_height: " << config.ee_height << std::endl
  //           << "ee_deep: " << config.ee_deep << std::endl; 

  this->alg_.unlock();
}

void TableClearingPredicatesAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TableClearingPredicatesAlgNode>(argc, argv, "table_clearing_predicates_alg_node");
}
