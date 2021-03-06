#include "table_clearing_decision_maker_alg_node.h"


TableClearingDecisionMakerAlgNode::TableClearingDecisionMakerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingDecisionMakerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 30;//in [Hz]
  this->alg_.setOn(false);
  time_start = util::GetTimeMs64();
  previous_centroid_size = 0;

  std::string input_topic;
  this->public_node_handle_.param("input_topic",input_topic,INPUT_TOPIC);
  int pushing_discretization;
  double  pushing_step,dropping_pose_x,dropping_pose_y,dropping_pose_z,
          pre_dropping_pose_x,pre_dropping_pose_y,pre_dropping_pose_z,
          pushing_limit, pushing_resolution;
  this->public_node_handle_.param("pushing_discretization", pushing_discretization, PUSHING_DISCRETIZATION);
  this->public_node_handle_.param("pushing_step", pushing_step, PUSHING_STEP);
  this->alg_.setPushingDiscretizationAndStep(pushing_discretization, pushing_step);

  this->public_node_handle_.param("dropping_pose_x", dropping_pose_x, DROPPING_POSE_X);
  this->public_node_handle_.param("dropping_pose_y", dropping_pose_y, DROPPING_POSE_Y);
  this->public_node_handle_.param("dropping_pose_z", dropping_pose_z, DROPPING_POSE_Z);
  this->alg_.setDroppingPose(dropping_pose_x, dropping_pose_y, dropping_pose_z);

  this->public_node_handle_.param("pre_dropping_pose_x", pre_dropping_pose_x, PRE_DROPPING_POSE_X);
  this->public_node_handle_.param("pre_dropping_pose_y", pre_dropping_pose_y, PRE_DROPPING_POSE_Y);
  this->public_node_handle_.param("pre_dropping_pose_z", pre_dropping_pose_z, PRE_DROPPING_POSE_Z);
  this->alg_.setPreDroppingPose(pre_dropping_pose_x, pre_dropping_pose_y, pre_dropping_pose_z);

  this->public_node_handle_.param("segmentation_service", segmentation_service, SEGMENTATION_SERVICE);
  this->public_node_handle_.param("predicates_service", predicates_service, PREDICATES_SERVICE);
  this->public_node_handle_.param("planner_service", planner_service, PLANNER_SERVICE);
  this->public_node_handle_.param("complete_planner_service", complete_planner_service, COMPLETE_PLANNER_SERVICE);

  this->public_node_handle_.param("execute_pushing_service", execute_pushing_service, EXECUTE_PUSHING_SERVICE);
  this->public_node_handle_.param("execute_grasping_service", execute_grasping_service, EXECUTE_GRASPING_SERVICE);

  this->public_node_handle_.param("execution", execution, EXECUTION);
  this->public_node_handle_.param("repeat", repeat, false);
  this->public_node_handle_.param("filtering", this->alg_.filtering, FILTERING);

  // experiment stuff
  this->public_node_handle_.param("save_experiment", this->save_experiment, false);
  this->public_node_handle_.param("working_folder", this->working_folder, WORKING_FOLDER);
  this->public_node_handle_.param("automatic_save", this->automatic_save, false);

  // action costs: if we use action costs we need to write the domain pddl file
  this->public_node_handle_.param("use_action_cost", use_action_cost, false);

  this->public_node_handle_.param("experiment_comparison", experiment_comparison, false);
     

  // [init publishers]
  this->objects_matching_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("objects_matching", 1);
  this->new_objects_centroid_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("new_objects_centroid", 1);
  this->old_objects_centroid_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("old_objects_centroid", 1);
  this->pushing_directions_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("pushing_directions", 1);
  this->action_trajectory_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("action_trajectory", 1);
  this->action_marker_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("action_marker", 1);
  this->objects_label_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("objects_label", 1);
  this->cloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  

  //this->kinect_subscriber_ = this->public_node_handle_.subscribe("/camera/depth_registered/points", 1, &TableClearingDecisionMakerAlgNode::kinect_callback, this);
  this->kinect_subscriber_ = this->public_node_handle_.subscribe(input_topic, 1, &TableClearingDecisionMakerAlgNode::kinect_callback, this);
  pthread_mutex_init(&this->kinect_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  get_complete_fast_downward_plan_client_ = this->public_node_handle_.serviceClient<iri_fast_downward_wrapper::FastDownwardCompletePlan>(complete_planner_service);

  execute_grasping_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_execute::ExecuteGrasping>(execute_grasping_service);

  execute_pushing_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_execute::ExecutePushing>(execute_pushing_service);

  get_fast_downward_plan_client_ = this->public_node_handle_.serviceClient<iri_fast_downward_wrapper::FastDownwardPlan>(planner_service);

  segments_objects_client_ = this->public_node_handle_.serviceClient<iri_tos_supervoxels::object_segmentation>(segmentation_service);

  get_symbolic_predicates_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_predicates::Predicates>(predicates_service);
  
  

  // [init action servers]
  
  // [init action clients]



  std::cout << "input_topic: " << input_topic << std::endl
  << "goal: " << this->alg_.goal << std::endl
  << "pushing_discretization: " << pushing_discretization << std::endl
  << "pushing_step: " << pushing_step << std::endl
  << "segmentation_service: " << segments_objects_client_.getService().c_str() << std::endl
  << "predicates_service: " << get_symbolic_predicates_client_.getService().c_str()<< std::endl
  << "planner_service: " << get_fast_downward_plan_client_.getService().c_str() << std::endl
  << "execute_pushing_service: " << execute_pushing_client_.getService().c_str() << std::endl
  << "execute_grasping_service: " << execute_grasping_client_.getService().c_str() << std::endl
  << "execution: " << execution << std::endl
  << "filtering: " << this->alg_.filtering << std::endl
  << "dropping_pose_x: " << dropping_pose_x << std::endl
  << "dropping_pose_y: " << dropping_pose_y << std::endl
  << "dropping_pose_z: " << dropping_pose_z << std::endl
  << "pre_dropping_pose_x: " << pre_dropping_pose_x << std::endl
  << "pre_dropping_pose_y: " << pre_dropping_pose_y << std::endl
  << "pre_dropping_pose_z: " << pre_dropping_pose_z << std::endl
  << "use_action_cost: " << use_action_cost << std::endl
  << "experiment_comparison: " << experiment_comparison << std::endl;
  

  if(save_experiment)
  {
    std::cout << "Your experiment are going to be saved in folder: " << working_folder << std::endl;
    eh.setUp(working_folder);
  }

  std::cout << "The goal set is:\n" << this->alg_.goal << "\n";
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
  //this->objects_matching_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->new_objects_centroid_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->old_objects_centroid_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->pushing_directions_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->action_trajectory_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->action_Marker_msg_.data = my_var;

  // Initialize the topic message structure
  //this->objects_label_MarkerArray_msg_.data = my_var;

  // Initialize the topic message structure
  //this->cloud_PointCloud2_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  //get_complete_fast_downward_plan_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (get_complete_fast_downward_plan_client_.call(get_complete_fast_downward_plan_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", get_complete_fast_downward_plan_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic get_complete_fast_downward_plan ");
  //}


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
  //this->objects_matching_publisher_.publish(this->objects_matching_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->new_objects_centroid_publisher_.publish(this->new_objects_centroid_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->old_objects_centroid_publisher_.publish(this->old_objects_centroid_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->pushing_directions_publisher_.publish(this->pushing_directions_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->action_trajectory_publisher_.publish(this->action_trajectory_MarkerArray_msg_);


  // Uncomment the following line to publish the topic message
  //this->action_publisher_.publish(this->action_Marker_msg_);

  // Uncomment the following line to publish the topic message
  //this->objects_label_publisher_.publish(this->objects_label_MarkerArray_msg_);

  // Uncomment the following line to publish the topic message
  //this->cloud_publisher_.publish(this->cloud_PointCloud2_msg_);

  uint n_objs = 0;
  if(this->alg_.getOn())
  {
    std::cout << "\n\n------------  STARTING PLANNING FRAMEWORK------------" << std::endl << std::endl;
    this->alg_.resetGoal();

    sensor_msgs::PointCloud2Ptr msg = this->alg_.getPointCloud();
    iri_tos_supervoxels::object_segmentation tos_srv;
    tos_srv.request.point_cloud = (*msg);

    // util::uint64 t_init_seg = util::GetTimeMs64(); 
    double t_init_seg = ros::Time::now().toSec(); 
    // std::clock_t t_init_seg = std::clock();
    if(!segments_objects_client_.call(tos_srv))
    {
      ROS_ERROR("Impossible segmenting the image - Failed to call the service or the are no objects");
      this->alg_.setOn(false);
    }
    // segmentation_time = (double)(util::GetTimeMs64() - t_init_seg);
    segmentation_time = (double)(ros::Time::now().toSec() - t_init_seg);
    // segmentation_time = (double)(std::clock() - t_init_seg) / CLOCKS_PER_SEC;

    n_objs = tos_srv.response.objects.objects.size();
    std::cout << n_objs << " Object detected\n";
    if(tos_srv.response.objects.objects.size() > 0) // if we have more than 1 object do the stuff
    {
      // object matching
      matchObjects(tos_srv);
      std::cout << std::endl;

      this->alg_.showObjectsRViz(tos_srv.response.objects.objects, msg->header, this->cloud_publisher_);
      //this->showObjectsRViz(tos_srv.response.objects.objects, msg->header);
      this->alg_.setNumberObjects(tos_srv.response.objects.objects.size());

      iri_table_clearing_predicates::Predicates pre_srv;
      pre_srv.request.original_cloud = (*msg);
      pre_srv.request.segmented_objects = tos_srv.response.objects.objects;
      pre_srv.request.plane_coefficients = tos_srv.response.plane_coeff;

      //util::uint64 t_init_predicates = util::GetTimeMs64(); 
      double t_init_predicates = ros::Time::now().toSec();
      // std::clock_t t_init_predicates_clock = std::clock();
      if(!get_symbolic_predicates_client_.call(pre_srv))
      {
        ROS_ERROR("Impossible getting the predicates");
        this->alg_.setOn(false);
        return;
      } 
      // predicates_time = (double)(util::GetTimeMs64() - t_init_predicates);
      predicates_time = (double)(ros::Time::now().toSec() - t_init_predicates);
      //predicates_time = (double)(std::clock() - t_init_predicates) / CLOCKS_PER_SEC;
      // predicates_time = pre_srv.response.predicates_time;
      // float predicates_time_clock = (float)(std::clock() - t_init_predicates_clock) / CLOCKS_PER_SEC;
      std::cout << "State generated in " << predicates_time << " seconds." << std::endl;


      on_predicates_time = pre_srv.response.on_predicates_time;
      block_predicates_time = pre_srv.response.block_predicates_time;
      block_grasp_predicates_time = pre_srv.response.block_grasp_predicates_time;
      objects_collisions_time = pre_srv.response.objects_collisions_time;
      ee_collisions_time = pre_srv.response.ee_collisions_time;
      average_objects_collision_time = pre_srv.response.average_objects_collision_time;
      average_ee_collision_time = pre_srv.response.average_ee_collision_time;

      this->alg_.setCentroids(pre_srv.response.centroids);
      this->alg_.setPlaneCoefficients(tos_srv.response.plane_coeff);

      //publish label markers 
      // init message
      objects_labels_markers.markers.resize(0);
      this->alg_.showObjectsLabelRViz(pre_srv.response.centroids,
                                      objects_labels_markers,
                                      pre_srv.response.obbs);
      objects_label_publisher_.publish(objects_labels_markers);


      //save the predicates
      this->alg_.setBlockPredicates(pre_srv.response.block_predicates);
      this->alg_.setOnTopPredicates(pre_srv.response.on_top_predicates);
      this->alg_.setBlockGraspPredicates(pre_srv.response.block_grasp_predicates);
      this->alg_.setPushingDirections(pre_srv.response.objects_pushing_directions);
      this->alg_.setGraspingPoses(pre_srv.response.grasping_poses);
      this->alg_.setApproachingPoses(pre_srv.response.approaching_poses);
      this->alg_.setPushingPoses(pre_srv.response.pushing_poses);
      this->alg_.setPrincipalDirections(pre_srv.response.principal_directions);
      this->alg_.setOBBs(pre_srv.response.obbs);
      this->alg_.setPushingObjectDistance(pre_srv.response.pushing_object_distance);
      this->alg_.setPushingLengths(pre_srv.response.pushing_lengths);
      this->alg_.setPushingGraspingPoses(pre_srv.response.pushing_grasping_poses);
      this->alg_.pushing_until_graspable = pre_srv.response.pushing_until_graspable;

      bool feasible = false; // is related to the IK
      while(!feasible) // call the planner until it finds a new solution
      {
        std::string goal = this->alg_.prepareGoalMsg();
        if(experiment_comparison)
          goal = this->alg_.newGoalExperimentComparison();

        if(not use_action_cost)
        {
          iri_fast_downward_wrapper::FastDownwardPlan fd_srv;
          fd_srv.request.objects = this->alg_.prepareObjectsMsg();
          std::vector<iri_fast_downward_wrapper::SymbolicPredicate> predicates = this->alg_.prepareSymbolicPredicatesMsg();
          fd_srv.request.symbolic_predicates = predicates;
          fd_srv.request.goal = goal;

          // util::uint64 t_init_planning = util::GetTimeMs64(); 
          double t_init_planning = ros::Time::now().toSec();
          // std::clock_t t_init_planning = std::clock();
          plan_feasible = true;
          // reset plan
          alg_.plan.actions.resize(0);
          alg_.plan.cost = 0;
          if(!get_fast_downward_plan_client_.call(fd_srv))
          {
            ROS_ERROR("Impossible getting the Fast Downward plan - Or the problem is bad defined or there exist no solution");
            this->alg_.setOn(false);
            feasible = true; // there is not IK influence here
            plan_feasible = false; //there is no plan

          }
          // planning_time = (double)(util::GetTimeMs64() - t_init_planning);
          planning_time = (double)(ros::Time::now().toSec() - t_init_planning);
          // planning_time = (double)(std::clock() - t_init_planning) / CLOCKS_PER_SEC;

          this->alg_.setPlan(fd_srv.response.plan);
        }
        else // if we use the cost fot the actions
        {
          std::cout << "Planning using the costs in the action. "; 
          iri_fast_downward_wrapper::FastDownwardCompletePlan fd_srv; 

          // problem pddl file
          fd_srv.request.objects = this->alg_.prepareObjectsMsg();
          std::vector<iri_fast_downward_wrapper::SymbolicPredicate> predicates = this->alg_.prepareSymbolicPredicatesMsg();
          fd_srv.request.symbolic_predicates = predicates;
          fd_srv. request.goal = goal;

          // domain pddl file
          fd_srv.request.add_total_cost = true;
          fd_srv.request.types.push_back("obj");
          fd_srv.request.types.push_back("direction");
          fd_srv.request.domain_symbolic_predicates = this->alg_.prepareDomainPredicateMsg();
          fd_srv.request.actions = this->alg_.prepareDomainActionsMsg();

          // util::uint64 t_init_planning = util::GetTimeMs64(); 
          double t_init_planning = ros::Time::now().toSec();
          // std::clock_t t_init_planning = std::clock();
          plan_feasible = true;
          // reset plan
          alg_.plan.actions.resize(0);
          alg_.plan.cost = 0;
          if(!get_complete_fast_downward_plan_client_.call(fd_srv))
          {
            ROS_ERROR("Impossible getting the Fast Downward plan - Or the problem is bad defined or there exist no solution");
            this->alg_.setOn(false);
            feasible = true; // there is not IK influence here
            plan_feasible = false; //there is no plan

          }
          // planning_time = (double)(util::GetTimeMs64() - t_init_planning);
          planning_time = (double)(ros::Time::now().toSec() - t_init_planning);
          // planning_time = (double)(std::clock() - t_init_planning) / CLOCKS_PER_SEC;

          std::cout << "Planed in " << planning_time << " seconds." << std::endl;

          this->alg_.setPlan(fd_srv.response.plan);

        }

        if(experiment_comparison and not plan_feasible)
          alg_.updateIndicesUnfeasibleList();
        else
          alg_.idx_unfeasible.resize(0);
        
        // if(!fd_srv.response.feasible)
        //   ROS_ERROR("There is not a feasible plan for such a problem");
          
        this->alg_.showFirstActionRViz(this->action_marker_publisher_, use_action_cost);
        this->alg_.showPushingDirectionsRviz(this->pushing_directions_publisher_);

        iri_table_clearing_execute::ExecuteGrasping grasping_srv;
        iri_table_clearing_execute::ExecutePushing pushing_srv;
        int action_type = this->alg_.setAction(grasping_srv,pushing_srv,use_action_cost);
        
        this->alg_.showActionTrajectory(action_trajectory_publisher_, action_type);
        
        double ik_time = 0;
        double execution_time = 0;
        ik_feasible = true;
        if(execution)
        { 
          switch(action_type)
          {
            case 0:
              // ROS_INFO("Calling execution service for pushing action");
              if(execute_pushing_client_.call(pushing_srv)) 
              {
                if(!pushing_srv.response.success) // because the IK had no solution
                {
                  ROS_WARN("Pushing action unfeasible");
                  this->alg_.setIKUnfeasiblePredicate();
                  this->alg_.updateGoal();
                  feasible = false;
                  ik_feasible = false;
                  ik_time = pushing_srv.response.ik_time;
                  execution_time = pushing_srv.response.execution_time;
                }
                else
                {
                  feasible = true; 
                  ik_time = pushing_srv.response.ik_time;
                  execution_time = pushing_srv.response.execution_time;

                  if(pushing_srv.response.executed)
                    this->alg_.updateEstimatedCentroids();

                }
              }
              else
              {
                feasible = true; // if we cannot call the service, or we decided to not execute anything keep going on
              }
              break;
            case 1:
              //ROS_INFO("Calling execution service for grasping action");
              if(execute_grasping_client_.call(grasping_srv)) 
              { 
                if(!grasping_srv.response.success) 
                {
                  ROS_WARN("Grasping action unfeasible");
                  this->alg_.setIKUnfeasiblePredicate();
                  this->alg_.updateGoal();
                  feasible = false;
                  ik_feasible = false;
                  ik_time = grasping_srv.response.ik_time;
                  execution_time = grasping_srv.response.execution_time;

                }
                else
                {
                  feasible = true; 
                  ik_time = grasping_srv.response.ik_time;
                  execution_time = grasping_srv.response.execution_time;
                }
              }
              else
              {
                feasible = true; // if we cannot call the service, or we decided to not execute anything keep going on
              }
              break;
            case -1:break;
            case -2:break;
            default: // for the case there is no action to do 
                     feasible = true;
                     break;
          }
        } // if execution
        else
        {
          feasible = true;
        } 

        // update experiment data
        if(save_experiment) 
        {
          char response;
          if(automatic_save)
          {
            response = 'y';
          } 
          else
          {
            std::cout << "\n\n Update experiment?(y - yes / whatever key to don't save)\n";
            std::cin >> response;
          }
          if(response == 'y' || response == 'Y')
          {
          
            // set data vector
            std::vector<double> data; // just to test - This should contains the values of the timers
            data.push_back(n_objs); 
            data.push_back(filtering_time);
            data.push_back(segmentation_time);
            data.push_back(predicates_time);
            data.push_back(planning_time);
            data.push_back(ik_time);
            data.push_back(pre_srv.response.on_predicates_time);
            data.push_back(pre_srv.response.block_predicates_time);
            data.push_back(pre_srv.response.block_grasp_predicates_time);
            data.push_back(pre_srv.response.objects_collisions_time);
            data.push_back(pre_srv.response.ee_collisions_time);
            data.push_back(pre_srv.response.average_objects_collision_time);
            data.push_back(pre_srv.response.average_ee_collision_time);
            data.push_back(execution_time);
            time_from_start = util::GetTimeMs64() - time_start;
            data.push_back(time_from_start);

            std::cout << "updating experiment\n";
            eh.updateExperiment(data,alg_.plan,ik_feasible,msg,true);

            eh.savePredicates(this->alg_.blocks_predicates,this->alg_.on_top_predicates,this->alg_.block_grasp_predicates);
            // if(!fd_srv.response.success)
            //   eh.writeUnfeasiblePlan();
          
            // if there is one object and there exists no plan we consider the table cleared
            if(n_objs == 1 && alg_.plan.actions.size() == 0)
            {
              std::cout << "There is one object but the robot is not able to approach it, the experiment is finished.\n";
              eh.newExperiment();
            }

          }// endif save_experiment
        }//execution


      }//exit while

      // clear the predicates to be sure there are no interferences
      this->alg_.resetPredicates();
    }// end if supervoxels
    else if(save_experiment) // if there are no objects update experiment
    {
      std::cout << "new experiment\n";
      eh.newExperiment();
    }

    this->alg_.setOn(false);
  }
  else // it the algorithm is not set to ON it means it is still waiting for a point cloud
  {
    while(!this->alg_.getOn())
    {
      ROS_INFO("Waiting for a point cloud from %s",kinect_subscriber_.getTopic().c_str());   
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Point cloud received");
  }

  // ask the user to repeat the main thread
  if(((this->alg_.getPlanLength() == 0) || (n_objs == 0)) && (!repeat)) 
  {
    std::cout << "\n\n Do you want to Repeat?(y,n)\n";
    if(save_experiment)
      std::cout << "[to save the experiment you have to close in this way the node - Press E/e to repeat as a new experiment]\n";
    char response;
    std::cin >> response;
    bool wrong_character = true;
    while(wrong_character)
    {
      switch(response)
      {
        case 'y':
        case 'Y':
            wrong_character = false;
            std::cout << "\n";
            break;
        case 'n':
        case 'N':
            std::cout << "\n You decided to NOT repeat the task. Shutdown the node...\n";
            std::cout << "\n Please press CTRL-C\n";
            if(save_experiment)
            {
              std::cout << "Closing file experiment file\n";
              eh.closeFile();
            }
            ros::shutdown();
            break;
        case 'e':
        case 'E':
            if(save_experiment)
            {
              if(alg_.plan.actions.size() > 0) // if there was a plan
                eh.writeExperimentInterrupeted();

              std::cout << "new experiment\n";
              eh.newExperiment();
              wrong_character = false;
              std::cout << "\n";
            }
            else
            {
              std::cout << "You have chosen to do not save the experiment, E/e options is not available";
            }
            break;
        default: break;
      }
    }
  }
  else // waits enough time to be sure to have a point cloud where there is no the robot's arm
  {
    double now_ = ros::Time::now().toSec();

    //ros::Time a = (this->alg_.getPointCloud)->header.stamp;

    // wait until the point cloud has a correct time stamp (it is has a stamp after the robot gone to home)
    while( this->alg_.getPointCloud()->header.stamp.sec < now_)
    {
      ros::Duration(0.1).sleep();
    }
    


    // if(this->alg_.filtering) // if filtering is on waits more
    //   ros::Duration(3).sleep();
    // else
    //   ros::Duration(1).sleep();
  }

}

void TableClearingDecisionMakerAlgNode::kinect_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  //ROS_INFO("TableClearingDecisionMakerAlgNode::kinect_callback: New Message Received");
  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->kinect_mutex_enter();

  this->alg_.setFrameId(msg->header.frame_id); 
  
  if(this->alg_.filtering)
  {
    //long time_filt_init = util::GetTimeMs64();
    // std::clock_t time_filt_init = std::clock();
    double t_filt_init = ros::Time().now().toSec();
    if(!this->alg_.getOn())
      ROS_INFO("New Point cloud received - Let's filter it! :)");
    // http://pointclouds.org/documentation/tutorials/statistical_outlier.php
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg(*msg,cloud);

    // Filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud.makeShared());
    sor.setMeanK (10);
    sor.setStddevMulThresh (1.0);
    sor.filter (cloud);

    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> rorfilter (true); // Initializing with true will allow us to extract the removed indices
    // rorfilter.setInputCloud (cloud.makeShared());
    // rorfilter.setRadiusSearch (0.02);
    // rorfilter.setMinNeighborsInRadius (5);
    // rorfilter.setNegative (false);
    // rorfilter.filter (cloud);
  
    // cloud.width = 640;
    // cloud.height = 480;
    // pcl::FastBilateralFilter<pcl::PointXYZRGBA> filter;
    // filter.setInputCloud(cloud.makeShared());
    // filter.setSigmaS(5);
    // filter.setSigmaR(5e-3);
    // filter.applyFilter(cloud);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);   

    this->alg_.setPointCloud(cloud_msg);
    //filtering_time = util::GetTimeMs64() - time_filt_init;
    //filtering_time = float(std::clock() - time_filt_init) / CLOCKS_PER_SEC;
    filtering_time = ros::Time().now().toSec() - t_filt_init;
  }
  else
  {
    filtering_time = 0;
    if(!this->alg_.getOn())
      ROS_INFO("New Point cloud received :)");
    this->alg_.setPointCloud(*msg);
  }


  this->alg_.setOn(true);

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  this->alg_.unlock();
  this->kinect_mutex_exit();
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
void TableClearingDecisionMakerAlgNode::matchObjects(iri_tos_supervoxels::object_segmentation & tos_srv)
{

  // We have the new centroids (centroids) and the old ones (centroids_old)
  // we now match the new ones to the old ones
  
  // if this is the first frame we do not do an object matching
  if(this->alg_.centroids_old.size() == 0)
  {
    // std::cout << " no previous frame saved\n";
    return; 
  }

  std::vector<geometry_msgs::Point> centroids_objs;
  for (uint i = 0; i < tos_srv.response.objects.objects.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(tos_srv.response.objects.objects[i],*cloud);

    // compute centroid of each obejct
    pcl::CentroidPoint<pcl::PointXYZRGBA> centroid; 
    for (uint p = 0; p< cloud->points.size(); ++p)   
        centroid.add(cloud->points[p]);  // add each point
    pcl::PointXYZRGBA c;
    centroid.get(c);
    // now the centroid is translated, this because in the table_clearing_planning the centorid is the average between the segmented object ad the projection, 
    // it is a little hack, but it works
    Eigen::Vector3f proj_eigen_point;
    Eigen::Vector3f plane_origin;
    // set plane origin
    plane_origin[0] = 1.0;
    plane_origin[1] = 1.0;
    // ax+by+cz+d=0 => z = (ax+by+d)/(-c)
    plane_origin[2] = (this->alg_.plane_coefficients.a * plane_origin[0] +
                            this->alg_.plane_coefficients.b * plane_origin[1] +
                                                   this->alg_.plane_coefficients.d) / (- this->alg_.plane_coefficients.c); 
    Eigen::Vector3f plane_normal;
    plane_normal[0] = this->alg_.plane_normal.x;
    plane_normal[1] = this->alg_.plane_normal.y;
    plane_normal[2] = this->alg_.plane_normal.z;
    Eigen::Vector3f c_eigen(c.x,c.y,c.z);
    pcl::geometry::project(c_eigen,plane_origin,plane_normal,proj_eigen_point);
    double distance_to_table = (c_eigen - proj_eigen_point).norm();
    c_eigen += plane_normal * distance_to_table/2;
    geometry_msgs::Point c_;
    c_.x = c_eigen[0]; c_.y = c_eigen[1]; c_.z = c_eigen[2];
    centroids_objs.push_back(c_);
  }

  // --------------------- matching ------------------
  if (this->alg_.centroids_old.size() != centroids_objs.size())
  {
    ROS_WARN("Impossible doing matching, the number of objects differs from the previosu frame!");
    return;
  }

  // for (int i = 0; i < alg_.centroids_old.size(); ++i)
  //   std::cout << "Centroids old " << i << " x: " << alg_.centroids_old[i].x << " y: " << alg_.centroids_old[i].y << " z: " << alg_.centroids_old[i].z << std::endl;
  // for (int i = 0; i < alg_.centroids_old.size(); ++i)
  //   std::cout << "Centroids " << i << " x: " << centroids_objs[i].x << " y: " << centroids_objs[i].y << " z: " << centroids_objs[i].z << std::endl;

  std::map<uint,uint> map_; // matching map
  for (uint i = 0; i < alg_.centroids_old.size(); ++i)
  {
    uint idx;
    double dist_min = std::numeric_limits<double>::max();
    for (uint o = 0; o < centroids_objs.size(); ++o)    
    {
      double dist = sqrt( pow(this->alg_.centroids_old[i].x - centroids_objs[o].x,2) + 
                          pow(this->alg_.centroids_old[i].y - centroids_objs[o].y,2) + 
                          pow(this->alg_.centroids_old[i].z - centroids_objs[o].z,2));
      // std::cout << " i:" << i << " o:" << o << " dist: " << dist << std::endl;
      if (dist < dist_min)
      {
        idx = o;
        dist_min = dist;
      }
    }
    map_.insert(std::make_pair(i,idx));
  }

  // public current centroids
  publicCurrentCentroids(centroids_objs);

  publicPreviousCentroids(this->alg_.centroids);
  
  std::cout << "OBJECT MATCHING -> the map is\n";
  for (int i = 0; i < map_.size(); ++i)
  {
    std::cout << i << " " << map_[i] << std::endl;
  }

  // check if the map is ok, if one object is mapped to 2 there is a problem
  std::vector < uint > counter(map_.size(),0);
  for (int i = 0; i < map_.size(); ++i)
    counter[map_[i]]++;
  for (int i = 0; i < map_.size(); ++i)
    if(counter[i]>1)
    {
      ROS_WARN("Impossible doing the match between the object, the memory is dropped off.");
      this->alg_.idx_old == std::numeric_limits<uint>::max();
      return;
    }


  // substitute
  iri_tos_supervoxels::segmented_objects tmp;
  for (uint i = 0; i < tos_srv.response.objects.objects.size(); ++i)
    tmp.objects.push_back(tos_srv.response.objects.objects[map_[i]]);
  tos_srv.response.objects = tmp;

}

void TableClearingDecisionMakerAlgNode::publicCurrentCentroids(std::vector<geometry_msgs::Point> centroids_objs)
{
  
  visualization_msgs::MarkerArray markers;
  // delete all the previous markers
  for(uint i=0; i < previous_centroid_size; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->alg_.frame_id;;
    marker.header.stamp = ros::Time().now();
    marker.ns = "current_centroid";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);
  }
  this->new_objects_centroid_publisher_.publish(markers); 
  this->previous_centroid_size = centroids_objs.size();//save new size of objects

  //publish the markers
  for(uint i=0; i < centroids_objs.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->alg_.frame_id;;
    marker.header.stamp = ros::Time().now();
    marker.ns = "current_centroid";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = centroids_objs[i].x; 
    marker.pose.position.y = centroids_objs[i].y; 
    marker.pose.position.z = centroids_objs[i].z; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;// this is the redaius, they ahve to be all the same to be sphere
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    markers.markers.push_back(marker);
  }
  this->new_objects_centroid_publisher_.publish(markers);
}

void TableClearingDecisionMakerAlgNode::publicPreviousCentroids(std::vector<geometry_msgs::Point> centroids_objs)
{
  visualization_msgs::MarkerArray markers;
  // delete all the previous markers                                                       }
  for(uint i=0; i < previous_centroid_size; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->alg_.frame_id;;
    marker.header.stamp = ros::Time().now();
    marker.ns = "previous_centroid";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);
  }
  this->old_objects_centroid_publisher_.publish(markers); 
  
  this->previous_centroid_size = centroids_objs.size();//save new size of objects

  //publish the markers
  for(uint i=0; i < centroids_objs.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->alg_.frame_id;;
    marker.header.stamp = ros::Time().now();
    marker.ns = "previous_centroid";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = centroids_objs[i].x; 
    marker.pose.position.y = centroids_objs[i].y; 
    marker.pose.position.z = centroids_objs[i].z; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;// this is the redaius, they ahve to be all the same to be sphere
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markers.markers.push_back(marker);
  }
  this->old_objects_centroid_publisher_.publish(markers);
}  
