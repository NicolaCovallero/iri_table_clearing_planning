#include "table_clearing_decision_maker_alg_node.h"

TableClearingDecisionMakerAlgNode::TableClearingDecisionMakerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingDecisionMakerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 2;//in [Hz]

  ros::service::waitForService("/iri_tos_supervoxels_alg/object_segmentation",2);// 2 seconds
  ros::service::waitForService("/table_clearing_predicates_alg_node/get_symbolic_predicates",2);
  ros::service::waitForService("/get_fast_downward_plan",2);

  // [init publishers]
  
  // [init subscribers]
  this->kinect_subscriber_ = this->public_node_handle_.subscribe("/camera/depth_registered/points", 1, &TableClearingDecisionMakerAlgNode::kinect_callback, this);
  pthread_mutex_init(&this->kinect_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  get_fast_downward_plan_client_ = this->public_node_handle_.serviceClient<iri_fast_downward_wrapper::FastDownwardPlan>("/get_fast_downward_plan");

  segments_objects_client_ = this->public_node_handle_.serviceClient<iri_tos_supervoxels::object_segmentation>("/iri_tos_supervoxels_alg/object_segmentation");

  get_symbolic_predicates_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_predicates::Predicates>("/table_clearing_predicates_alg_node/get_symbolic_predicates");
  
  

  // [init action servers]
  
  // [init action clients]
}

TableClearingDecisionMakerAlgNode::~TableClearingDecisionMakerAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->kinect_mutex_);
}

void TableClearingDecisionMakerAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  ///get_fast_downward_plan_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (/get_fast_downward_plan_client_.call(/get_fast_downward_plan_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", /get_fast_downward_plan_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic /get_fast_downward_plan ");
  //}


  //segments_objects_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (segments_objects_client_.call(segments_objects_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", segments_objects_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic segments_objects ");
  //}


  //get_symbolic_predicates_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (get_symbolic_predicates_client_.call(get_symbolic_predicates_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", get_symbolic_predicates_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic get_symbolic_predicates ");
  //}


  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void TableClearingDecisionMakerAlgNode::kinect_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("TableClearingDecisionMakerAlgNode::kinect_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->kinect_mutex_enter();

  iri_tos_supervoxels::object_segmentation tos_srv;
  tos_srv.request.point_cloud = (*msg);
  if(!segments_objects_client_.call(tos_srv))
  {
    ROS_ERROR("Impossible segmenting the image");
    return;
  }

  this->alg_.setNumberObjects(tos_srv.response.objects.objects.size());

  iri_table_clearing_predicates::Predicates pre_srv;
  pre_srv.request.original_cloud = (*msg);
  pre_srv.request.segmented_objects = tos_srv.response.objects.objects;
  pre_srv.request.plane_coefficients = tos_srv.response.plane_coeff;
  if(!get_symbolic_predicates_client_.call(pre_srv))
  {
    ROS_ERROR("Impossible getting the predicates");
    return;
  } 

  //save the predicates
  this->alg_.setBlockPredicates(pre_srv.response.block_predicates);
  this->alg_.setOnTopPredicates(pre_srv.response.on_top_predicates);
  this->alg_.setBlockGraspPredicates(pre_srv.response.block_grasp_predicates);
  this->alg_.setPushingDirections(pre_srv.response.objects_pushing_directions);
  this->alg_.setGraspingPoses(pre_srv.response.grasping_poses);

  // std::vector<iri_fast_downward_wrapper::Object> objects_msg;
  // objects_msg = 
  // std::vector<iri_fast_downward_wrapper/SymbolicPredicate> symbolic_predicates;
  iri_fast_downward_wrapper::FastDownwardPlan fd_srv;
  fd_srv.request.objects = this->alg_.prepareObjectsMsg();
  fd_srv.request.symbolic_predicates = this->alg_.prepareSymbolicPredicatesMsg();
  fd_srv.request.goal = this->alg_.prepareGoalMsg();

  if(get_fast_downward_plan_client_.call(fd_srv))
  {
    ROS_ERROR("Impossible getting the Fast Downward plan");
    return;
  }


  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->kinect_mutex_exit();
}

void TableClearingDecisionMakerAlgNode::kinect_mutex_enter(void)
{
  pthread_mutex_lock(&this->kinect_mutex_);
}

void TableClearingDecisionMakerAlgNode::kinect_mutex_exit(void)
{
  pthread_mutex_unlock(&this->kinect_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TableClearingDecisionMakerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TableClearingDecisionMakerAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TableClearingDecisionMakerAlgNode>(argc, argv, "table_clearing_decision_maker_alg_node");
}
