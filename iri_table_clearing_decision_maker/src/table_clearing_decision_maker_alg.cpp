#include "table_clearing_decision_maker_alg.h"

TableClearingDecisionMakerAlgorithm::TableClearingDecisionMakerAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);

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

void TableClearingDecisionMakerAlgorithm::showObjectsRViz(std::vector<sensor_msgs::PointCloud2> segmented_objects, std_msgs::Header header, ros::Publisher& cloud_publisher_)
{
  // std::cout << "Creating cloud message\n";
  //creating new point cloud for debugging with random color for each segmented object
  sensor_msgs::PointCloud2 segmented_objects_msg;
    pcl::PointCloud<pcl::PointXYZRGBA> segmented_objects_cloud; 

    for (int i = 0; i < segmented_objects.size(); ++i)
    {
      
      pcl::PointCloud<pcl::PointXYZRGBA> tmp;
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
        tmp.points.at(h).a = 1;
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
                              ros::Publisher& label_pub,
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
		marker.pose.position.x = centroids[i].x;
		marker.pose.position.y = centroids[i].y;
		marker.pose.position.z = centroids[i].z;
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
	label_pub.publish(markers);
}
void TableClearingDecisionMakerAlgorithm::showFirstActionRViz(ros::Publisher& action_pub)
{
	// read the first action of the plan
	if(plan.actions.size() == 0)
	{
		ROS_ERROR("showFirstActionRViz -> No plan is set.");
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