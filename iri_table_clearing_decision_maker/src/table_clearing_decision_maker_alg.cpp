#include "table_clearing_decision_maker_alg.h"

TableClearingDecisionMakerAlgorithm::TableClearingDecisionMakerAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
  this->resetGoal();
}

TableClearingDecisionMakerAlgorithm::~TableClearingDecisionMakerAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TableClearingDecisionMakerAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TableClearingDecisionMakerAlgorithm Public API
std::vector<iri_fast_downward_wrapper::Object> TableClearingDecisionMakerAlgorithm::prepareObjectsMsg()
{
	std::vector<iri_fast_downward_wrapper::Object> objects_msg;
	for (int i = 0; i < this->n_objects; ++i)
	{
		iri_fast_downward_wrapper::Object object;
		std::string object_name = "o";
		std::ostringstream convert;   // stream used for the conversion
		convert << i;
		object_name += convert.str();
		object.object_name = object_name;
		object.type = "obj";
		objects_msg.push_back(object);
	}
	return objects_msg;
}
std::vector<iri_fast_downward_wrapper::SymbolicPredicate> TableClearingDecisionMakerAlgorithm::prepareSymbolicPredicatesMsg()
{
	std::vector<iri_fast_downward_wrapper::SymbolicPredicate> blocks_predicates_msg;
	iri_fast_downward_wrapper::SymbolicPredicate tmp;

	// ---------------- Add Blocks Predicates ---------------------------------
	tmp.objects.resize(2); // the block predicates only involves 2 objects
	for (int i = 0; i < this->blocks_predicates.size(); ++i)
	{
		for (int p = 0; p < this->blocks_predicates[i].dir1.size(); ++p)
		{
			tmp.predicate_name = "block_dir1";
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << i;
			object_name += convert.str();
			tmp.objects[1] = object_name;

			std::string object_name2 = "o";
			std::ostringstream convert2;   // stream used for the conversion
			convert2 << this->blocks_predicates[i].dir1[p];
			object_name2 += convert2.str();
			tmp.objects[0] = object_name2;

			blocks_predicates_msg.push_back(tmp);
		}
		for (int p = 0; p < this->blocks_predicates[i].dir2.size(); ++p)
		{
			tmp.predicate_name = "block_dir2";
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << i;
			object_name += convert.str();
			tmp.objects[1] = object_name;

			std::string object_name2 = "o";
			std::ostringstream convert2;   // stream used for the conversion
			convert2 << this->blocks_predicates[i].dir2[p];
			object_name2 += convert2.str();
			tmp.objects[0] = object_name2;	

			blocks_predicates_msg.push_back(tmp);
		}
		for (int p = 0; p < this->blocks_predicates[i].dir3.size(); ++p)
		{
			tmp.predicate_name = "block_dir3";
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << i;
			object_name += convert.str();
			tmp.objects[1] = object_name;

			std::string object_name2 = "o";
			std::ostringstream convert2;   // stream used for the conversion
			convert2 << this->blocks_predicates[i].dir3[p];
			object_name2 += convert2.str();
			tmp.objects[0] = object_name2;

			blocks_predicates_msg.push_back(tmp);
		}
		for (int p = 0; p < this->blocks_predicates[i].dir4.size(); ++p)
		{
			tmp.predicate_name = "block_dir4";
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << i;
			object_name += convert.str();
			tmp.objects[1] = object_name;

			std::string object_name2 = "o";
			std::ostringstream convert2;   // stream used for the conversion
			convert2 << this->blocks_predicates[i].dir4[p];
			object_name2 += convert2.str();
			tmp.objects[0] = object_name2;

			blocks_predicates_msg.push_back(tmp);
		}
	}

	// ---------------- Add On Top Predicates --------------------------------
	tmp.predicate_name = "on";
	for (int i = 0; i < this->on_top_predicates.size(); ++i)
	{
		tmp.objects.resize(2); // the block predicates only involves 2 objects
		std::string object_name = "o";
		std::ostringstream convert;   // stream used for the conversion
		convert << i;
		object_name += convert.str();
		tmp.objects[0] = object_name;
		for (int p = 0; p < this->on_top_predicates[i].object.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->on_top_predicates[i].object[p];
			object_name += convert.str();
			tmp.objects[1] = object_name;
			blocks_predicates_msg.push_back(tmp);
		}
	}

	// ---------------- Add Block Grasp Predicates ----------------------------
	tmp.predicate_name = "block_grasp";
	for (int i = 0; i < this->block_grasp_predicates.size(); ++i)
	{
		tmp.objects.resize(2); // the block predicates only involves 2 objects
		std::string object_name = "o";
		std::ostringstream convert;   // stream used for the conversion
		convert << i;
		object_name += convert.str();
		tmp.objects[1] = object_name;
		for (int p = 0; p < this->block_grasp_predicates[i].object.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->block_grasp_predicates[i].object[p];
			object_name += convert.str();
			tmp.objects[0] = object_name;
			blocks_predicates_msg.push_back(tmp);
		}
	}

	// ----------------- Add Ik unfeasible predicates --------------------------
	for (int i = 0; i < this->ik_unfeasible_predicates.size(); ++i)
	{
		tmp.predicate_name = this->ik_unfeasible_predicates[i].action;
		tmp.objects.resize(1);
		tmp.objects[0] = this->ik_unfeasible_predicates[i].object; 
		blocks_predicates_msg.push_back(tmp);
	}

 	return blocks_predicates_msg;
}

std::string TableClearingDecisionMakerAlgorithm::prepareGoalMsg()
{
	// if the config file is equal to rhw default goal 
	return this->goal;
}
void TableClearingDecisionMakerAlgorithm::setNumberObjects(uint n_objects)
{
	this->n_objects = n_objects;

	// initialize vector 
	removed_object_from_goal.resize(n_objects);
	for (uint i = 0; i < n_objects; ++i)
		removed_object_from_goal[i] = false;

}
void TableClearingDecisionMakerAlgorithm::setBlockPredicates(std::vector<iri_table_clearing_predicates::BlockPredicate> blocks_predicates)
{
	this->blocks_predicates = blocks_predicates;
}
void TableClearingDecisionMakerAlgorithm::setOnTopPredicates(std::vector<iri_table_clearing_predicates::OnTopPredicate> on_top_predicates)
{
	this->on_top_predicates = on_top_predicates;
}
void TableClearingDecisionMakerAlgorithm::setBlockGraspPredicates(std::vector<iri_table_clearing_predicates::BlockGraspPredicate> block_grasp_predicates)
{
	this->block_grasp_predicates = block_grasp_predicates;
}
void TableClearingDecisionMakerAlgorithm::setPushingDirections(std::vector<iri_table_clearing_predicates::PushingDirections> pushing_directions)
{
	this->pushing_directions = pushing_directions;
}
void TableClearingDecisionMakerAlgorithm::setGraspingPoses(std::vector<iri_table_clearing_predicates::GraspingPoses> grasping_poses)
{
	this->grasping_poses = grasping_poses;
}
void TableClearingDecisionMakerAlgorithm::setApproachingPoses(std::vector<iri_table_clearing_predicates::GraspingPoses> approaching_poses)
{
	this-> approaching_poses = approaching_poses;
}
void TableClearingDecisionMakerAlgorithm::setPushingPoses(std::vector<iri_table_clearing_predicates::PushingPoses> pushing_poses)
{
	this->pushing_poses = pushing_poses;
	if(this->pushing_poses.size() == 0)
		ROS_WARN("0 Pushing Poses");
}
void TableClearingDecisionMakerAlgorithm::setPlan(iri_fast_downward_wrapper::Plan plan)
{
	this->plan = plan;
}
void TableClearingDecisionMakerAlgorithm::setFrameId(std::string frame_id)
{
	this->frame_id = frame_id;
}
visualization_msgs::Marker TableClearingDecisionMakerAlgorithm::firstActionMarker()
{
	visualization_msgs::Marker marker;
}
void TableClearingDecisionMakerAlgorithm::setCentroids(std::vector<geometry_msgs::Point> centroids)
{
	this->centroids = centroids;
}
void TableClearingDecisionMakerAlgorithm::setPlaneCoefficients(iri_tos_supervoxels::plane_coefficients plane_coefficients)
{
	this->plane_coefficients = plane_coefficients;
	this->plane_normal.x = plane_coefficients.a;
	this->plane_normal.y = plane_coefficients.b;
	this->plane_normal.z = plane_coefficients.c;
}
void TableClearingDecisionMakerAlgorithm::setPrincipalDirections(std::vector<iri_table_clearing_predicates::PrincipalDirections> principal_directions)
{
	this->principal_directions = principal_directions;
}
void TableClearingDecisionMakerAlgorithm::setPushingObjectDistance(double pushing_object_distance)
{
	this->pushing_object_distance = pushing_object_distance;
}
void TableClearingDecisionMakerAlgorithm::setAABBs(std::vector<iri_table_clearing_predicates::AABB> aabbs)
{
	this->aabbs = aabbs;
}
void TableClearingDecisionMakerAlgorithm::setPushingLengths(std::vector<iri_table_clearing_predicates::PushingLength> pushing_lengths)
{
	this->pushing_lengths = pushing_lengths;
}
void TableClearingDecisionMakerAlgorithm::setPushingGraspingPoses(std::vector<iri_table_clearing_predicates::PushingGraspingPose> pushing_grasping_poses)
{
	this->pushing_grasping_poses = pushing_grasping_poses;
}
void TableClearingDecisionMakerAlgorithm::setDroppingPose(double dropping_pose_x,double dropping_pose_y,double dropping_pose_z)
{
	this->dropping_pose.pose.position.x = dropping_pose_x;
	this->dropping_pose.pose.position.y = dropping_pose_y;
	this->dropping_pose.pose.position.z = dropping_pose_z;

	// vertical orientation - Pointing down
    tf::Quaternion quat;
    quat.setRPY(0,M_PI,0);
	this->dropping_pose.pose.orientation.w = quat.w();
	this->dropping_pose.pose.orientation.x = quat.x();
	this->dropping_pose.pose.orientation.y = quat.y();
	this->dropping_pose.pose.orientation.z = quat.z();
	this->dropping_pose.header.frame_id = "/estirabot_link_footprint";
}
void TableClearingDecisionMakerAlgorithm::setPreDroppingPose(double dropping_pose_x,double dropping_pose_y,double dropping_pose_z)
{
	this->pre_dropping_pose.pose.position.x = dropping_pose_x;
	this->pre_dropping_pose.pose.position.y = dropping_pose_y;
	this->pre_dropping_pose.pose.position.z = dropping_pose_z;

	// vertical orientation - Pointing down
    tf::Quaternion quat;
    quat.setRPY(0,M_PI,0);
	this->pre_dropping_pose.pose.orientation.w = quat.w();
	this->pre_dropping_pose.pose.orientation.x = quat.x();
	this->pre_dropping_pose.pose.orientation.y = quat.y();
	this->pre_dropping_pose.pose.orientation.z = quat.z();
	this->pre_dropping_pose.header.frame_id = "/estirabot_link_footprint";
}

void TableClearingDecisionMakerAlgorithm::showObjectsRViz(std::vector<sensor_msgs::PointCloud2> segmented_objects, std_msgs::Header header, ros::Publisher& cloud_publisher_)
{
	// std::cout << "Creating cloud message\n";
	//creating new point cloud for debugging with random color for each segmented object
	sensor_msgs::PointCloud2 segmented_objects_msg;
	pcl::PointCloud<pcl::PointXYZRGB> segmented_objects_cloud; 

	for (int i = 0; i < segmented_objects.size(); ++i)
	{
	  
	  pcl::PointCloud<pcl::PointXYZRGB> tmp;
	  pcl::fromROSMsg(segmented_objects[i],tmp);
	  float r,g,b;
	  r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	  g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	  b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	  for (int h = 0; h < tmp.points.size(); ++h)
	  {
	    tmp.points.at(h).r = r*255;
	    tmp.points.at(h).g = g*255;
	    tmp.points.at(h).b = b*255;
	    //tmp.points.at(h).a = 1;
	  }

	  segmented_objects_cloud += tmp;
	}

	segmented_objects_cloud.width = segmented_objects_cloud.points.size();
	segmented_objects_cloud.height = 1;
	segmented_objects_cloud.is_dense = true;

	//std::cout << "segmented_objects_cloud.points.size() " << segmented_objects_cloud.points.size() << "\n";
	pcl::toROSMsg(segmented_objects_cloud,segmented_objects_msg);
	//std::cout << "segmented_objects_msg.data.size() " << segmented_objects_msg.data.size() << "\n";

	segmented_objects_msg.header = header;
	segmented_objects_msg.header.stamp = ros::Time::now();
	cloud_publisher_.publish(segmented_objects_msg);
}
void TableClearingDecisionMakerAlgorithm::showObjectsLabelRViz(std::vector<geometry_msgs::Point> centroids,
								visualization_msgs::MarkerArray& objects_labels_markers,
                              	std::vector<iri_table_clearing_predicates::AABB> aabbs)
{
	visualization_msgs::MarkerArray markers;
	if(aabbs.size() != centroids.size())
		ROS_ERROR("The AABB and the centorids have not the same length - Impossible creating label markers");

	for (int i = 0; i < centroids.size(); ++i)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = this->frame_id;
		marker.header.stamp = ros::Time();
		// marker.header.seq = i;
		marker.ns = "label";
		marker.id = i;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = centroids[i].x - 0.10*this->plane_normal.x;
		marker.pose.position.y = centroids[i].y - 0.10*this->plane_normal.y;
		marker.pose.position.z = centroids[i].z - 0.10*this->plane_normal.z; //we translate it by 10 cm
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.color.r = 1.0;
		marker.color.a = 1.0;
		marker.scale.z = 0.1;
		// marker.lifetime = ros::Duration(10);
		std::string object_label = "o";
		std::ostringstream convert;   // stream used for the conversion
		convert << i;
		object_label += convert.str();
		marker.text =  object_label;
		markers.markers.push_back(marker);
	}
	objects_labels_markers = markers;
	
}
void TableClearingDecisionMakerAlgorithm::showFirstActionRViz(ros::Publisher& action_pub)
{
	// read the first action of the plan
	if(plan.actions.size() == 0)
	{
		ROS_WARN("showFirstActionRViz -> No plan is set.");
		return;
	}

	double arrow_length = 0.3; //length arrow

	visualization_msgs::Marker marker;
	marker.header.frame_id = this->frame_id;
	marker.header.stamp = ros::Time();
	// marker.header.seq = i;
	marker.ns = "first_action";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0;
	marker.scale.x = 0.02; // shaft diameter
  	marker.scale.y = 0.02; // head diameter
  	marker.scale.z = 0.07; // head length

	// get the id of the object
	std::string object = plan.actions[0].objects[0];
	object.erase(0,1); //erase the first character which is "o"
	int idx_obj;
	int Succeeded = std::sscanf ( object.c_str(), "%d", &idx_obj );
	if ( !Succeeded || Succeeded == EOF ) // check if something went wrong during the conversion
	{
		ROS_ERROR("Problem retrieving the number of the object from the plan");
		return;
	}

	// check if it is a pushing in direction 1
	if(strcmp(plan.actions[0].action_name.c_str(),"push_dir1")==0)
	{
		marker.color.g = 1.0;
		geometry_msgs::Point p1;
		p1.x = centroids[idx_obj].x;
		p1.y = centroids[idx_obj].y;
		p1.z = centroids[idx_obj].z;
		marker.points.push_back(p1);

		geometry_msgs::Point p2;
		p2.x = p1.x + this->pushing_directions[idx_obj].dir1.x * arrow_length;
		p2.y = p1.y + this->pushing_directions[idx_obj].dir1.y * arrow_length;
		p2.z = p1.z + this->pushing_directions[idx_obj].dir1.z * arrow_length;
		marker.points.push_back(p2);
	}
	else if (strcmp(plan.actions[0].action_name.c_str(),"push_dir2")==0)
	{
		marker.color.g = 1.0;
		geometry_msgs::Point p1;
		p1.x = centroids[idx_obj].x;
		p1.y = centroids[idx_obj].y;
		p1.z = centroids[idx_obj].z;
		marker.points.push_back(p1);

		geometry_msgs::Point p2;
		p2.x = p1.x + this->pushing_directions[idx_obj].dir2.x * arrow_length;
		p2.y = p1.y + this->pushing_directions[idx_obj].dir2.y * arrow_length;
		p2.z = p1.z + this->pushing_directions[idx_obj].dir2.z * arrow_length;
		marker.points.push_back(p2);
	}
	else if (strcmp(plan.actions[0].action_name.c_str(),"push_dir3")==0)
	{
		marker.color.g = 1.0;
		geometry_msgs::Point p1;
		p1.x = centroids[idx_obj].x;
		p1.y = centroids[idx_obj].y;
		p1.z = centroids[idx_obj].z;
		marker.points.push_back(p1);

		geometry_msgs::Point p2;
		p2.x = p1.x + this->pushing_directions[idx_obj].dir3.x * arrow_length;
		p2.y = p1.y + this->pushing_directions[idx_obj].dir3.y * arrow_length;
		p2.z = p1.z + this->pushing_directions[idx_obj].dir3.z * arrow_length;
		marker.points.push_back(p2);
	}
	else if (strcmp(plan.actions[0].action_name.c_str(),"push_dir4")==0)
	{
		marker.color.g = 1.0;
		geometry_msgs::Point p1;
		p1.x = centroids[idx_obj].x;
		p1.y = centroids[idx_obj].y;
		p1.z = centroids[idx_obj].z;
		marker.points.push_back(p1);

		geometry_msgs::Point p2;
		p2.x = p1.x + this->pushing_directions[idx_obj].dir4.x * arrow_length;
		p2.y = p1.y + this->pushing_directions[idx_obj].dir4.y * arrow_length;
		p2.z = p1.z + this->pushing_directions[idx_obj].dir4.z * arrow_length;
		marker.points.push_back(p2);
	}
	else if (strcmp(plan.actions[0].action_name.c_str(),"grasp")==0)
	{
		marker.color.r = 1.0;
		geometry_msgs::Point p1;
		p1.x = centroids[idx_obj].x;
		p1.y = centroids[idx_obj].y;
		p1.z = centroids[idx_obj].z;
		marker.points.push_back(p1);

		if(this->principal_directions.size() == 0 )
		{
			ROS_ERROR("Principal directions not set - Impossible creating the action marker for the grasp action");
			return;
		}

		// this is not the grasping approaching direction
		geometry_msgs::Point p2;
		p2.x = p1.x + this->principal_directions[idx_obj].p3.x * arrow_length;
		p2.y = p1.y + this->principal_directions[idx_obj].p3.y * arrow_length;
		p2.z = p1.z + this->principal_directions[idx_obj].p3.z * arrow_length;
		marker.points.push_back(p2);
	}

	action_pub.publish(marker);
}
void TableClearingDecisionMakerAlgorithm::showActionTrajectory(ros::Publisher& trajectory_pub)
{
	if(this->pushing_cartesian_trajectory.size() == 0)
	{
		ROS_ERROR("In showActioNTrajectoy - Pushing trajectory not saved");
		return;
	}
	visualization_msgs::MarkerArray markers;
	for (int i = 0; i < this->pushing_cartesian_trajectory.size(); ++i)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = this->pushing_cartesian_trajectory[i].header.frame_id;
		//marker.header.frame_id = "";
		marker.header.stamp = ros::Time();
		marker.header.seq = i;
		marker.ns = "pushing_trajectory";
		marker.id = i;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.scale.x = 0.02; // point width
	  	marker.scale.y = 0.02; // popint height
	  	geometry_msgs::Point p;
	  	p.x =  this->pushing_cartesian_trajectory[i].pose.position.x;
	  	p.y =  this->pushing_cartesian_trajectory[i].pose.position.y;
	  	p.z =  this->pushing_cartesian_trajectory[i].pose.position.z;
	  	marker.points.push_back(p);
	  	marker.pose.orientation.w = 1;
	  	markers.markers.push_back(marker);

	}
	trajectory_pub.publish(markers);
}

void TableClearingDecisionMakerAlgorithm::setOn(bool on)
{
	this->set = on;
}
bool TableClearingDecisionMakerAlgorithm::getOn()
{
	return this->set;
}
void TableClearingDecisionMakerAlgorithm::setPointCloud(sensor_msgs::PointCloud2 point_cloud)
{
	this->point_cloud = point_cloud;
}
sensor_msgs::PointCloud2* TableClearingDecisionMakerAlgorithm::getPointCloud()
{
	return &(this->point_cloud);
}
void TableClearingDecisionMakerAlgorithm::setPushingDiscretizationAndStep(int pushing_discretization, double pushing_step)
{
	this->pushing_discretization = pushing_discretization;
	this->pushing_step = pushing_step;
}
void TableClearingDecisionMakerAlgorithm::setIKUnfeasiblePredicate()
{
	if(this->plan.actions.size() == 0)
	{
		ROS_WARN("No plan - Your are trying to specify that an action if unfeasible when there are no actions in the plan.");
		return;
	}
	IKUnfeasiblePredicate pred;
	if(strcmp(plan.actions[0].action_name.c_str(),"push_dir1") == 0)
		pred.action = "ik_unfeasible_dir1";
	else if(strcmp(plan.actions[0].action_name.c_str(),"push_dir2") == 0)
		pred.action = "ik_unfeasible_dir2";
	else if(strcmp(plan.actions[0].action_name.c_str(),"push_dir3") == 0)
		pred.action = "ik_unfeasible_dir3";
	else if(strcmp(plan.actions[0].action_name.c_str(),"push_dir4") == 0)
		pred.action = "ik_unfeasible_dir4";
	else if(strcmp(plan.actions[0].action_name.c_str(),"grasp") == 0)
		pred.action = "ik_unfeasible_grasp";
	else
		ROS_ERROR("Error trying to set the IK unfeasible predicate.");

	pred.object = plan.actions[0].objects[0];
	ik_unfeasible_predicates.push_back(pred);

}

int TableClearingDecisionMakerAlgorithm::setAction( iri_table_clearing_execute::ExecuteGrasping& grasping,
                    iri_table_clearing_execute::ExecutePushing& pushing)
{

	if(plan.actions.size() == 0)
	{
		ROS_WARN("setAction -> No plan is set.");
		return -3;
	}

	pushing.request.pushing_cartesian_trajectory.resize(0);

	double step;

	// get the id of the object
	std::string object = plan.actions[0].objects[0];
	object.erase(0,1); //erase the first character which is "o"
	int idx_obj;
	int Succeeded = std::sscanf ( object.c_str(), "%d", &idx_obj );
	if ( !Succeeded || Succeeded == EOF ) // check if something went wrong during the conversion
	{
		ROS_ERROR("Problem retrieving the number of the object from the plan");
		return -2;
	}
	geometry_msgs::PoseStamped pose;

	if( strcmp(plan.actions[0].action_name.c_str(),"push_dir1")==0)
	{
		std::cout << "Action to execute: push_dir1 " << "o" << idx_obj << std::endl;

		// step = (double)((pushing_step * this->aabbs[idx_obj].deep + this->pushing_object_distance)/
		// 			(this->pushing_discretization -1)); 
		step = (double)((this->pushing_lengths[idx_obj].dir1 + this->pushing_object_distance)/
		 			(this->pushing_discretization -1));
		if(this->pushing_poses.size() == 0 )
		{
			ROS_ERROR("Pushing Poses not set in - setAction()");
			return -2;
		}
		if(this->pushing_directions.size() == 0)
		{
			ROS_ERROR("Pushing directions not set in - setAction()");
			return -2;
		}
		pose = pushing_poses[idx_obj].pose_dir1;
		pose.header.frame_id = this->frame_id;

		// add a re pushing pose in roder to avoid collisions
		pose.pose.position.x = 	pose.pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pose.pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pose.pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		pose = pushing_poses[idx_obj].pose_dir1;
		pose.header.frame_id = this->frame_id;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		for (int i = 1; i < this->pushing_discretization; ++i)
		{
			pose.pose.position.x += this->pushing_directions[idx_obj].dir1.x * step;  
			pose.pose.position.y += this->pushing_directions[idx_obj].dir1.y * step;  
			pose.pose.position.z += this->pushing_directions[idx_obj].dir1.z * step;  

			pushing.request.pushing_cartesian_trajectory.push_back(pose);
		}
		// put the last pose to be above the last one, the robot will not come back to its first pose
		pose.pose.position.x = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);
						
		this->pushing_cartesian_trajectory = pushing.request.pushing_cartesian_trajectory;

		iri_table_clearing_predicates::PushingGraspingPose pgp = this->pushing_grasping_poses[idx_obj];
		pushing.request.future_grasp_pose = pgp.grasp_dir1;
		pushing.request.future_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_pre_grasp_pose = pgp.app_dir1;
		pushing.request.future_pre_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose = pgp.grasp_dir1;
		pushing.request.future_post_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose.pose.position.x = pushing.request.future_post_grasp_pose.pose.position.x - 0.3 * this->plane_normal.x;
		pushing.request.future_post_grasp_pose.pose.position.y = pushing.request.future_post_grasp_pose.pose.position.y - 0.3 * this->plane_normal.y;
		pushing.request.future_post_grasp_pose.pose.position.z = pushing.request.future_post_grasp_pose.pose.position.z - 0.3 * this->plane_normal.z;


		return 0;
	}
	else if( strcmp(plan.actions[0].action_name.c_str(),"push_dir2")==0)
	{
		std::cout << "Action to execute: push_dir2 " << "o" << idx_obj << std::endl;

		// step = (double)((pushing_step * this->aabbs[idx_obj].deep + this->pushing_object_distance)/
		// 			(this->pushing_discretization -1)); // it is not considering the dimension of the gripper
		step = (double)((this->pushing_lengths[idx_obj].dir2 + this->pushing_object_distance)/
		 			(this->pushing_discretization -1));
		if(this->pushing_poses.size() == 0 )
		{
			ROS_ERROR("Pushing Poses not set in - setAction()");
			return -2;
		}
		if(this->pushing_directions.size() == 0)
		{
			ROS_ERROR("Pushing directions not set in - setAction()");
			return -2;
		}
		pose = pushing_poses[idx_obj].pose_dir2;
		pose.header.frame_id = this->frame_id;

		pose.pose.position.x = 	pose.pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pose.pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pose.pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		pose = pushing_poses[idx_obj].pose_dir2;
		pose.header.frame_id = this->frame_id;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		for (int i = 1; i < this->pushing_discretization; ++i)
		{
			pose.pose.position.x += this->pushing_directions[idx_obj].dir2.x * step;  
			pose.pose.position.y += this->pushing_directions[idx_obj].dir2.y * step;  
			pose.pose.position.z += this->pushing_directions[idx_obj].dir2.z * step;  

			pushing.request.pushing_cartesian_trajectory.push_back(pose);
		}
		// put the last pose to be above the last one, the robot will not come back to its first pose
		pose.pose.position.x = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		this->pushing_cartesian_trajectory = pushing.request.pushing_cartesian_trajectory;

		iri_table_clearing_predicates::PushingGraspingPose pgp = this->pushing_grasping_poses[idx_obj];		
		pushing.request.future_grasp_pose = pgp.grasp_dir2;
		pushing.request.future_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_pre_grasp_pose = pgp.app_dir2;
		pushing.request.future_pre_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose = pgp.grasp_dir2;
		pushing.request.future_post_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose.pose.position.x = pushing.request.future_post_grasp_pose.pose.position.x - 0.3 * this->plane_normal.x;
		pushing.request.future_post_grasp_pose.pose.position.y = pushing.request.future_post_grasp_pose.pose.position.y - 0.3 * this->plane_normal.y;
		pushing.request.future_post_grasp_pose.pose.position.z = pushing.request.future_post_grasp_pose.pose.position.z - 0.3 * this->plane_normal.z;


		return 0;
	}
	else if( strcmp(plan.actions[0].action_name.c_str(),"push_dir3")==0)
	{
		std::cout << "Action to execute: push_dir3 " << "o" << idx_obj << std::endl;

		// step = (double)((pushing_step * this->aabbs[idx_obj].width + this->pushing_object_distance)/ 
		// 			(this->pushing_discretization -1)); // it is not considering the dimension of the gripper
		step = (double)((this->pushing_lengths[idx_obj].dir3 + this->pushing_object_distance)/
		 			(this->pushing_discretization -1));
		if(this->pushing_poses.size() == 0 )
		{
			ROS_ERROR("Pushing Poses not set in - setAction()");
			return -2;
		}
		if(this->pushing_directions.size() == 0)
		{
			ROS_ERROR("Pushing directions not set in - setAction()");
			return -2;
		}
		
		pose = pushing_poses[idx_obj].pose_dir3;
		pose.header.frame_id = this->frame_id;

		pose.pose.position.x = 	pose.pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pose.pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pose.pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		pose = pushing_poses[idx_obj].pose_dir3;
		pose.header.frame_id = this->frame_id;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		for (int i = 1; i < this->pushing_discretization; ++i)
		{
			pose.pose.position.x += this->pushing_directions[idx_obj].dir3.x * step;  
			pose.pose.position.y += this->pushing_directions[idx_obj].dir3.y * step;  
			pose.pose.position.z += this->pushing_directions[idx_obj].dir3.z * step;  

			pushing.request.pushing_cartesian_trajectory.push_back(pose);
		}
		// put the last pose to be above the last one, the robot will not come back to its first pose
		pose.pose.position.x = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		this->pushing_cartesian_trajectory = pushing.request.pushing_cartesian_trajectory;

		iri_table_clearing_predicates::PushingGraspingPose pgp = this->pushing_grasping_poses[idx_obj];
		pushing.request.future_grasp_pose = pgp.grasp_dir3;
		pushing.request.future_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_pre_grasp_pose = pgp.app_dir3;
		pushing.request.future_pre_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose = pgp.grasp_dir3;
		pushing.request.future_post_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose.pose.position.x = pushing.request.future_post_grasp_pose.pose.position.x - 0.3 * this->plane_normal.x;
		pushing.request.future_post_grasp_pose.pose.position.y = pushing.request.future_post_grasp_pose.pose.position.y - 0.3 * this->plane_normal.y;
		pushing.request.future_post_grasp_pose.pose.position.z = pushing.request.future_post_grasp_pose.pose.position.z - 0.3 * this->plane_normal.z;


		return 0;
	}
	else if( strcmp(plan.actions[0].action_name.c_str(),"push_dir4")==0)
	{
		std::cout << "Action to execute: push_dir4 " << "o" << idx_obj << std::endl; 

		// step = (double)((pushing_step * this->aabbs[idx_obj].width + this->pushing_object_distance)/
		// 			(this->pushing_discretization -1)); // it is not considering the dimension of the gripper
		step = (double)((this->pushing_lengths[idx_obj].dir4 + this->pushing_object_distance)/
		 			(this->pushing_discretization -1));
		if(this->pushing_poses.size() == 0 )
		{
			ROS_ERROR("Pushing Poses not set in - setAction()");
			return -255;
		}
		if(this->pushing_directions.size() == 0)
		{
			ROS_ERROR("Pushing directions not set in - setAction()");
			return -2;
		}
		pose = pushing_poses[idx_obj].pose_dir4;
		pose.header.frame_id = this->frame_id;

		pose.pose.position.x = 	pose.pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pose.pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pose.pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		pose = pushing_poses[idx_obj].pose_dir4;
		pose.header.frame_id = this->frame_id;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		for (int i = 1; i < this->pushing_discretization; ++i)
		{
			pose.pose.position.x += this->pushing_directions[idx_obj].dir4.x * step;  
			pose.pose.position.y += this->pushing_directions[idx_obj].dir4.y * step;  
			pose.pose.position.z += this->pushing_directions[idx_obj].dir4.z * step;  

			pushing.request.pushing_cartesian_trajectory.push_back(pose);
		}
		// put the last pose to be above the last one, the robot will not come back to its first pose
		pose.pose.position.x = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.x
								 - dist_last_pose * this->plane_normal.x;
		pose.pose.position.y = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.y
								 - dist_last_pose * this->plane_normal.y;
		pose.pose.position.z = 	pushing.request.pushing_cartesian_trajectory.back().pose.position.z
								 - dist_last_pose * this->plane_normal.z;
		pushing.request.pushing_cartesian_trajectory.push_back(pose);

		this->pushing_cartesian_trajectory = pushing.request.pushing_cartesian_trajectory;

		iri_table_clearing_predicates::PushingGraspingPose pgp = this->pushing_grasping_poses[idx_obj];
		pushing.request.future_grasp_pose = pgp.grasp_dir4;
		pushing.request.future_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_pre_grasp_pose = pgp.app_dir4;
		pushing.request.future_pre_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose = pgp.grasp_dir4;
		pushing.request.future_post_grasp_pose.header.frame_id = this->frame_id;
		pushing.request.future_post_grasp_pose.pose.position.x = pushing.request.future_post_grasp_pose.pose.position.x - 0.3 * this->plane_normal.x;
		pushing.request.future_post_grasp_pose.pose.position.y = pushing.request.future_post_grasp_pose.pose.position.y - 0.3 * this->plane_normal.y;
		pushing.request.future_post_grasp_pose.pose.position.z = pushing.request.future_post_grasp_pose.pose.position.z - 0.3 * this->plane_normal.z;


		return 0;
	}
	else if (strcmp(plan.actions[0].action_name.c_str(),"grasp")==0)
	{
		std::cout << "Action to execute: grasp " << "o" << idx_obj << std::endl;

		geometry_msgs::PoseStamped grasping_pose, approaching_pose, post_grasping_pose;

		grasping_pose = this->grasping_poses[idx_obj].grasping_poses[0];
		grasping_pose.header.frame_id = this->frame_id;

		approaching_pose = this->approaching_poses[idx_obj].grasping_poses[0];
		approaching_pose.header.frame_id = this->frame_id;

		// post grasping pose is translated by other 10 cm
		post_grasping_pose = grasping_pose;
		post_grasping_pose.header.frame_id = this->frame_id;

		post_grasping_pose.pose.position.x = post_grasping_pose.pose.position.x - 0.3 * this->plane_normal.x;
		post_grasping_pose.pose.position.y = post_grasping_pose.pose.position.y - 0.3 * this->plane_normal.y;
		post_grasping_pose.pose.position.z = post_grasping_pose.pose.position.z - 0.3 * this->plane_normal.z;

		grasping.request.grasping_pose = grasping_pose;
		grasping.request.approaching_pose = approaching_pose;
		grasping.request.post_grasping_pose = post_grasping_pose;


		// std::cout << "pre dropping_pose \n x: " <<this->pre_dropping_pose.pose.position.x << std::endl
		// << "  y: " <<this->pre_dropping_pose.pose.position.y << std::endl
		// << "  z: " <<this->pre_dropping_pose.pose.position.z << std::endl
		// << "  w: " <<this->pre_dropping_pose.pose.orientation.w << std::endl
		// << "  quat x: " <<this->pre_dropping_pose.pose.orientation.x << std::endl
		// << "  quat y: " <<this->pre_dropping_pose.pose.orientation.y << std::endl
		// << "  quat z: " <<this->pre_dropping_pose.pose.orientation.z << std::endl;
		grasping.request.pre_dropping_pose = this->pre_dropping_pose;
		//grasping.request.dropping_pose = this->dropping_pose;
		return 1;
	}
	else
	{
		ROS_ERROR("The first action is not one of ther permitted. The current action is %s",plan.actions[0].action_name.c_str());
		return -1;
	}
}
void TableClearingDecisionMakerAlgorithm::updateGoal()
{
	std::vector<uint> indices;
	// check what object have all the ik_unfeasible predicates set to true
	for (uint o = 0; o < n_objects; ++o)
	{
		uint counter = 0;
		std::ostringstream obj_str;
		obj_str << o;
		std::string obj_sting = "o" + obj_str.str();
		for (uint i = 0; i < ik_unfeasible_predicates.size(); ++i)
		{
			if(ik_unfeasible_predicates[i].object == obj_sting)
			  counter++;
		}
		if(counter > 4) // this means that all the 5 ik_unfeasible rpedicates have been set to true
		{	
			indices.push_back(o);
			//std::cout << "One object have all the ik_ufneasible prediates set to true\n";
		}
	}

	// update the boolean vector
	for (int i = 0; i < indices.size(); ++i)
	{
		if(indices[i] > n_objects)
		{
			ROS_WARN("Wrong index of the object to remove: you wanted to remove object %d when there are %d objects.",indices[i],n_objects);
			return;
		}
		this->removed_object_from_goal[indices[i]] = true;
		//std::cout << "Set to true index " << indices[i] << std::endl;
	}


	//update goal string
	goal = "(and ";
	for (uint i = 0; i < this->n_objects; ++i)
	{
		std::ostringstream obj_str;
		obj_str << i;
		if(not removed_object_from_goal[i])
			goal+= "(removed o" + obj_str.str() +") ";
		else
			ROS_WARN("Object o%s has been removed from the goal.",obj_str.str().c_str());
	}
	goal += " )";
	std::cout << "The updated goal is: " << goal << std::endl;

}
void TableClearingDecisionMakerAlgorithm::resetGoal()
{
	goal = "(not (exists(?x - obj)(not (removed ?x))))";
}
int TableClearingDecisionMakerAlgorithm::getPlanLength()
{
	return this->plan.actions.size();
}
void TableClearingDecisionMakerAlgorithm::resetPredicates()
{
	this->ik_unfeasible_predicates.resize(0);	
	this->blocks_predicates.resize(0);
	this->on_top_predicates.resize(0);
	this->block_grasp_predicates.resize(0);
}