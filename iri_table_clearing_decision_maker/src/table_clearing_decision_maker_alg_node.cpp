#include "table_clearing_decision_maker_alg_node.h"

TableClearingDecisionMakerAlgNode::TableClearingDecisionMakerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingDecisionMakerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 2;//in [Hz]
  this->alg_.setOn(false);

  std::string input_topic;
  this->public_node_handle_.param("input_topic",input_topic,INPUT_TOPIC);
  this->public_node_handle_.param("goal", this->alg_.goal, GOAL);
  int pushing_discretization;
  double pushing_step;
  this->public_node_handle_.param("pushing_discretization", pushing_discretization, PUSHING_DISCRETIZATION);
  this->public_node_handle_.param("pushing_step", pushing_step, PUSHING_STEP);
  this->alg_.setPushingDiscretizationAndStep(pushing_discretization, pushing_step);

  this->public_node_handle_.param("segmentation_service", segmentation_service, SEGMENTATION_SERVICE);
  this->public_node_handle_.param("predicates_service", predicates_service, PREDICATES_SERVICE);
  this->public_node_handle_.param("planner_service", planner_service, PLANNER_SERVICE);
  this->public_node_handle_.param("execute_pushing_service", execute_pushing_service, EXECUTE_PUSHING_SERVICE);
  this->public_node_handle_.param("execute_grasping_service", execute_grasping_service, EXECUTE_GRASPING_SERVICE);

  this->public_node_handle_.param("execution", execution, EXECUTION);


  std::cout << "input_topic: " << input_topic << std::endl
  << "goal: " << this->alg_.goal << std::endl
  << "pushing_discretization: " << pushing_discretization << std::endl
  << "pushing_step: " << pushing_step << std::endl
  << "segmentation_service: " << segmentation_service << std::endl
  << "predicates_service: " << predicates_service << std::endl
  << "planner_service: " << planner_service << std::endl
  << "execute_pushing_service: " << execute_pushing_service << std::endl
  << "execute_grasping_service: " << execute_grasping_service << std::endl
  << "execution: " << execution << std::endl;

  // [init publishers]
  this->action_trajectory_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("action_trajectory", 1);
  this->action_marker_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("action_marker", 1);
  this->objects_label_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("objects_label", 1);
  this->cloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  
  // [init subscribers]
  //this->kinect_subscriber_ = this->public_node_handle_.subscribe("/camera/depth_registered/points", 1, &TableClearingDecisionMakerAlgNode::kinect_callback, this);
  this->kinect_subscriber_ = this->public_node_handle_.subscribe(input_topic, 1, &TableClearingDecisionMakerAlgNode::kinect_callback, this);
  pthread_mutex_init(&this->kinect_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  execute_grasping_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_execute::ExecuteGrasping>(execute_grasping_service);

  execute_pushing_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_execute::ExecutePushing>(execute_pushing_service);

  get_fast_downward_plan_client_ = this->public_node_handle_.serviceClient<iri_fast_downward_wrapper::FastDownwardPlan>(planner_service);

  segments_objects_client_ = this->public_node_handle_.serviceClient<iri_tos_supervoxels::object_segmentation>(segmentation_service);

  get_symbolic_predicates_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_predicates::Predicates>(predicates_service);
  
  

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
  // wait for all the services
  while(!ros::service::waitForService(segmentation_service,ros::Duration(5.0))){}
  while(!ros::service::waitForService(planner_service,ros::Duration(5.0))){}
  while(!ros::service::waitForService(predicates_service,ros::Duration(5.0))){}
  if(execution)
  {
    while(!ros::service::waitForService(execute_pushing_service,ros::Duration(5.0))){}
  }


  // [fill msg structures]
  // Initialize the topic message structure
  //this->action_trajectory_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->action_Marker_msg_.data = my_var;

  // Initialize the topic message structure
  //this->objects_label_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->cloud_PointCloud2_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  //execute_grasping_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (execute_grasping_client_.call(execute_grasping_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", execute_grasping_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic execute_grasping ");
  //}


  //execute_pushing_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (execute_pushing_client_.call(execute_pushing_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", execute_pushing_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic execute_pushing ");
  //}


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
  // Uncomment the following line to publish the topic message
  //this->action_trajectory_publisher_.publish(this->action_trajectory_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->action_publisher_.publish(this->action_Marker_msg_);

  // Uncomment the following line to publish the topic message
  //this->objects_label_publisher_.publish(this->objects_label_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->cloud_publisher_.publish(this->cloud_PointCloud2_msg_);

  if(this->alg_.getOn())
  {
    std::cout << "\n\n------------  STARTING PLANNING FRAMEWORK------------" << std::endl << std::endl;

    sensor_msgs::PointCloud2* msg = this->alg_.getPointCloud();
    iri_tos_supervoxels::object_segmentation tos_srv;
    tos_srv.request.point_cloud = (*msg);
    if(!segments_objects_client_.call(tos_srv))
    {
      ROS_ERROR("Impossible segmenting the image");
      this->alg_.setOn(false);
      return;
    }
    std::cout << tos_srv.response.objects.objects.size() << " Object detected\n";
    if(tos_srv.response.objects.objects.size() > 0) // if we have more than 1 object do the stuff
    {
      this->alg_.showObjectsRViz(tos_srv.response.objects.objects, msg->header, this->cloud_publisher_);
      //this->showObjectsRViz(tos_srv.response.objects.objects, msg->header);
      this->alg_.setNumberObjects(tos_srv.response.objects.objects.size());

      iri_table_clearing_predicates::Predicates pre_srv;
      pre_srv.request.original_cloud = (*msg);
      pre_srv.request.segmented_objects = tos_srv.response.objects.objects;
      pre_srv.request.plane_coefficients = tos_srv.response.plane_coeff;
      if(!get_symbolic_predicates_client_.call(pre_srv))
      {
        ROS_ERROR("Impossible getting the predicates");
        this->alg_.setOn(false);
        return;
      } 

      //plots label markers 
      this->alg_.showObjectsLabelRViz(pre_srv.response.centroids,
                                      this->objects_label_publisher_,
                                      pre_srv.response.aabbs);
      this->alg_.setCentroids(pre_srv.response.centroids);
      this->alg_.setPlaneCoefficients(tos_srv.response.plane_coeff);

      //save the predicates
      this->alg_.setBlockPredicates(pre_srv.response.block_predicates);
      this->alg_.setOnTopPredicates(pre_srv.response.on_top_predicates);
      this->alg_.setBlockGraspPredicates(pre_srv.response.block_grasp_predicates);
      this->alg_.setPushingDirections(pre_srv.response.objects_pushing_directions);
      this->alg_.setGraspingPoses(pre_srv.response.grasping_poses);
      this->alg_.setPushingPoses(pre_srv.response.pushing_poses);
      this->alg_.setPrincipalDirections(pre_srv.response.principal_directions);
      this->alg_.setAABBs(pre_srv.response.aabbs);

      iri_fast_downward_wrapper::FastDownwardPlan fd_srv;
      fd_srv.request.objects = this->alg_.prepareObjectsMsg();
      fd_srv.request.symbolic_predicates = this->alg_.prepareSymbolicPredicatesMsg();
      fd_srv.request.goal = this->alg_.prepareGoalMsg();

      std::cout << "The goal set is:\n" << fd_srv.request.goal << "\n";

      if(!get_fast_downward_plan_client_.call(fd_srv))
      {
        ROS_ERROR("Impossible getting the Fast Downward plan");
        this->alg_.setOn(false);
        return;
      }
      // if(!fd_srv.response.feasible)
      //   ROS_ERROR("There is not a feasible plan for such a problem");

      this->alg_.setPlan(fd_srv.response.plan);
        
      this->alg_.showFirstActionRViz(this->action_marker_publisher_);

      iri_table_clearing_execute::ExecuteGrasping grasping_srv;
      iri_table_clearing_execute::ExecutePushing pushing_srv;
      int action = this->alg_.setAction(grasping_srv,pushing_srv);
      this->alg_.showActionTrajectory(action_trajectory_publisher_);
      if(execution)
      { 

        switch(action)
        {
          case 0:
                  if(execute_pushing_client_.call(pushing_srv)) 
                  {

                  }
                  break;
          case 1:break;
          case -1:break;
          case -2:break;
          default: break;
        }

      }
    }
    this->alg_.setOn(false);
  }
}

/*  [subscriber callbacks] */
void TableClearingDecisionMakerAlgNode::kinect_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  //ROS_INFO("TableClearingDecisionMakerAlgNode::kinect_callback: New Message Received");
  this->alg_.setFrameId(msg->header.frame_id); 
  this->alg_.setPointCloud(*msg);
  this->alg_.setOn(true);
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->kinect_mutex_enter();

  
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

  this->config_= config;
  
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
