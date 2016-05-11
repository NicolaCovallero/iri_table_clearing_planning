#include "table_clearing_predicates_alg.h"

TableClearingPredicatesAlgorithm::TableClearingPredicatesAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
  this->setOnTopParameters(ON_TH1, ON_TH2);
}

TableClearingPredicatesAlgorithm::~TableClearingPredicatesAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TableClearingPredicatesAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TableClearingPredicatesAlgorithm Public API
// IriTableClearingPredicatesAlgorithm Public API
void TableClearingPredicatesAlgorithm::setGripperSimpleModel(double height, double deep, double width, double distance_plane)
{
	this->tcp.setGripperSimpleModel(height, deep, width, distance_plane);
}
void TableClearingPredicatesAlgorithm::setApproachingDistance(double approaching_distance)
{
	this->tcp.setApproachingDistance(approaching_distance);
}
void TableClearingPredicatesAlgorithm::setFingersModel(double opening_width,double closing_width, double finger_width,
               double deep, double height, double closing_height)
{
	this->tcp.setFingersModel(opening_width, closing_width, finger_width, deep, height, closing_height);
}
void TableClearingPredicatesAlgorithm::setPushingMethod(double pushing_method)
{
	this->pushing_method = pushing_method;
}
void TableClearingPredicatesAlgorithm::setOriginalPointCloud(PointCloudT original_cloud)
{
	this->tcp.setOriginalPointCloud(original_cloud);
}

void TableClearingPredicatesAlgorithm::setObjectsPointCloud(std::vector<PointCloudT > &objects)
{
	this->tcp.setObjectsPointCloud(objects);
}

void TableClearingPredicatesAlgorithm::setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients)
{
	this->tcp.setPlaneCoefficients(plane_coefficients);
}
void TableClearingPredicatesAlgorithm::setPushingStep(double pushing_step)
{
	this->tcp.setPushingStep(pushing_step);
}
void TableClearingPredicatesAlgorithm::setPushingObjectDistance(double pushing_object_distance)
{
	this->tcp.setPushingObjectDistance(pushing_object_distance);
}
void TableClearingPredicatesAlgorithm::computeAABBObjects(bool refine_centroids)
{
	this->tcp.computeAABBObjects(refine_centroids);
}

void TableClearingPredicatesAlgorithm::computeSimpleHeuristicGraspingPoses(bool vertical_poses)
{
	this->tcp.computeSimpleHeuristicGraspingPoses(vertical_poses);
}

void TableClearingPredicatesAlgorithm::computePrincipalDirections()
{
	this->tcp.computePrincipalDirections();
}
void TableClearingPredicatesAlgorithm::computeProjectionsOnTable()
{
	this->tcp.computeProjectionsOnTable();
}

void TableClearingPredicatesAlgorithm::computeProjectionsConvexHull()
{
	this->tcp.computeProjectionsConvexHull();
}

void TableClearingPredicatesAlgorithm::computeRichConvexHulls()
{
	this->tcp.computeRichConvexHulls();
}

void TableClearingPredicatesAlgorithm::computeOnTopPredicates(double th1, double th2, bool print)
{
	this->tcp.computeOnTopPredicates(th1, th2, print);
}

void TableClearingPredicatesAlgorithm::computeBlockPredicates(bool print)
{
	this->tcp.computeBlockPredicates(print,this->pushing_method);
}

void TableClearingPredicatesAlgorithm::computeBlockGraspPredicates(bool print)
{
	this->tcp.computeBlockGraspPredicates(print);
}

uint TableClearingPredicatesAlgorithm::getNumObjects()
{
	return this->tcp.getNumObjects();
}
std::vector<ObjectFull> TableClearingPredicatesAlgorithm::getFullObjects()
{
	return this->tcp.getFullObjects();
}

std::vector<AABB> TableClearingPredicatesAlgorithm::getAABBObjects()
{
	return this->tcp.getAABBObjects();
}
double TableClearingPredicatesAlgorithm::getPushingObjectDistance()
{
	return this->tcp.getPushingObjectDistance();
}

std::vector<iri_table_clearing_predicates::BlockPredicate> TableClearingPredicatesAlgorithm::getBlockPredicates()
{
	std::vector<BlocksPredicate> block_predicates = this->tcp.getBlockPredicates();
	std::vector<iri_table_clearing_predicates::BlockPredicate> block_predicates_msgs;
	
	block_predicates_msgs.resize(block_predicates.size());
	for (int i = 0; i < block_predicates_msgs.size(); ++i)
	{
		// dir 1
		for (int p = 0; p < block_predicates[i].block_dir1.size(); ++p)
		{
			block_predicates_msgs[i].dir1.push_back(block_predicates[i].block_dir1[p]);
			// std::cout << " object " << i << " is blocked by object " << block_predicates[i].block_dir1[p] << " in direction 1\n";
		}
		// dir 2
		for (int p = 0; p < block_predicates[i].block_dir2.size(); ++p)
		{
			block_predicates_msgs[i].dir2.push_back(block_predicates[i].block_dir2[p]);
			// std::cout << " object " << i << " is blocked by object " << block_predicates[i].block_dir2[p] << " in direction 2\n";
		}
		// dir 3
		for (int p = 0; p < block_predicates[i].block_dir3.size(); ++p)
		{
			block_predicates_msgs[i].dir3.push_back(block_predicates[i].block_dir3[p]);
			// std::cout << " object " << i << " is blocked by object " << block_predicates[i].block_dir3[p] << " in direction 3\n";
		}
		// dir 4
		for (int p = 0; p < block_predicates[i].block_dir4.size(); ++p)
		{
			block_predicates_msgs[i].dir4.push_back(block_predicates[i].block_dir4[p]);
			// std::cout << " object " << i << " is blocked by object " << block_predicates[i].block_dir4	[p] << " in direction 4\n";
		}
	}

	return block_predicates_msgs;
}

std::vector<iri_table_clearing_predicates::OnTopPredicate> TableClearingPredicatesAlgorithm::getOnTopPredicates()
{
	std::vector<std::vector<uint> > on_top_predicates = this->tcp.getOnTopPredicates();
	std::vector<iri_table_clearing_predicates::OnTopPredicate> on_top_predicates_msg;
	on_top_predicates_msg.resize(on_top_predicates.size());

	for (int i = 0; i < on_top_predicates.size(); ++i)
	{
		for (int p = 0; p < on_top_predicates[i].size(); ++p)
		{
			on_top_predicates_msg[i].object.push_back(on_top_predicates[i][p]);
		}
	}

	return on_top_predicates_msg;
}

std::vector<iri_table_clearing_predicates::BlockGraspPredicate> TableClearingPredicatesAlgorithm::getBlockGraspPredicates()
{
	std::vector< std::vector<uint> > block_grasp_predicates = this->tcp.getBlockGraspPredicates();
	std::vector<iri_table_clearing_predicates::BlockGraspPredicate> block_grasp_predicates_msg;
	block_grasp_predicates_msg.resize(block_grasp_predicates.size());

	for (int i = 0; i < block_grasp_predicates.size(); ++i)
	{
		block_grasp_predicates_msg[i].object.resize(0);
		for (int p = 0; p < block_grasp_predicates[i].size(); ++p)
		{
			block_grasp_predicates_msg[i].object.push_back(block_grasp_predicates[i][p]);
		}
	}

	return block_grasp_predicates_msg;
}

std::vector<iri_table_clearing_predicates::PushingDirections> TableClearingPredicatesAlgorithm::getPushingDirections()
{
	std::vector<PrincipalDirectionsProjected> principal_directions_projected = this->tcp.getProjectedPrincipalDirections();
	std::vector<iri_table_clearing_predicates::PushingDirections> pushing_directions;
	pushing_directions.resize(principal_directions_projected.size());

	for (int i = 0; i < principal_directions_projected.size(); ++i)
	{
		pushing_directions[i].dir1.x = principal_directions_projected[i].dir1[0];
		pushing_directions[i].dir1.y = principal_directions_projected[i].dir1[1];
		pushing_directions[i].dir1.z = principal_directions_projected[i].dir1[2];

		pushing_directions[i].dir2.x = principal_directions_projected[i].dir2[0];
		pushing_directions[i].dir2.y = principal_directions_projected[i].dir2[1];
		pushing_directions[i].dir2.z = principal_directions_projected[i].dir2[2];

		pushing_directions[i].dir3.x = principal_directions_projected[i].dir3[0];
		pushing_directions[i].dir3.y = principal_directions_projected[i].dir3[1];
		pushing_directions[i].dir3.z = principal_directions_projected[i].dir3[2];

		pushing_directions[i].dir4.x = principal_directions_projected[i].dir4[0];
		pushing_directions[i].dir4.y = principal_directions_projected[i].dir4[1];
		pushing_directions[i].dir4.z = principal_directions_projected[i].dir4[2];
	}

	return pushing_directions;
}

std::vector<iri_table_clearing_predicates::GraspingPoses> TableClearingPredicatesAlgorithm::getGraspingPoses()
{
	std::vector<GraspingPose> grasping_poses = this->tcp.getGraspingPoses();
	std::vector<iri_table_clearing_predicates::GraspingPoses> grasping_poses_msg;
	grasping_poses_msg.resize(this->tcp.getNumObjects());

	for (int i = 0; i < this->tcp.getNumObjects(); ++i)
	{
		// -------------------------------------------------------------
		// IMPORTANT! : The header have to be set in the ...alg_node.cpp
		// -------------------------------------------------------------	
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.pose.position.x = grasping_poses[i].translation[0];
		pose_stamped.pose.position.y = grasping_poses[i].translation[1];
		pose_stamped.pose.position.z = grasping_poses[i].translation[2];

		pose_stamped.pose.orientation.x =  grasping_poses[i].quaternion.x();
		pose_stamped.pose.orientation.y =  grasping_poses[i].quaternion.y();
		pose_stamped.pose.orientation.z =  grasping_poses[i].quaternion.z();
		pose_stamped.pose.orientation.w =  grasping_poses[i].quaternion.w();

		grasping_poses_msg[i].grasping_poses.push_back(pose_stamped);
	}

	return grasping_poses_msg;
}
std::vector<iri_table_clearing_predicates::GraspingPoses> TableClearingPredicatesAlgorithm::getApproachingPoses()
{
	std::vector<GraspingPose> approaching_poses = this->tcp.getApproachingPoses();
	std::vector<iri_table_clearing_predicates::GraspingPoses> approaching_poses_msg;
	approaching_poses_msg.resize(this->tcp.getNumObjects());

	for (int i = 0; i < this->tcp.getNumObjects(); ++i)
	{
		// -------------------------------------------------------------
		// IMPORTANT! : The header have to be set in the ...alg_node.cpp
		// -------------------------------------------------------------	
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.pose.position.x = approaching_poses[i].translation[0];
		pose_stamped.pose.position.y = approaching_poses[i].translation[1];
		pose_stamped.pose.position.z = approaching_poses[i].translation[2];

		pose_stamped.pose.orientation.x =  approaching_poses[i].quaternion.x();
		pose_stamped.pose.orientation.y =  approaching_poses[i].quaternion.y();
		pose_stamped.pose.orientation.z =  approaching_poses[i].quaternion.z();
		pose_stamped.pose.orientation.w =  approaching_poses[i].quaternion.w();

		approaching_poses_msg[i].grasping_poses.push_back(pose_stamped);
	}

	return approaching_poses_msg;
}
std::vector<iri_table_clearing_predicates::PushingPoses> TableClearingPredicatesAlgorithm::getPushingPoses()
{
	std::vector<iri_table_clearing_predicates::PushingPoses> pushing_poses_msg;
	std::vector<PushingPose> pushing_poses = this->tcp.getPushingPoses();
	pushing_poses_msg.resize(this->tcp.getNumObjects());

	for (int i = 0; i < pushing_poses.size(); ++i)
	{
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.pose.position.x = pushing_poses[i].pose_dir1.translation[0];
		pose_stamped.pose.position.y = pushing_poses[i].pose_dir1.translation[1];
		pose_stamped.pose.position.z = pushing_poses[i].pose_dir1.translation[2];

		pose_stamped.pose.orientation.x =  pushing_poses[i].pose_dir1.quaternion.x();
		pose_stamped.pose.orientation.y =  pushing_poses[i].pose_dir1.quaternion.y();
		pose_stamped.pose.orientation.z =  pushing_poses[i].pose_dir1.quaternion.z();
		pose_stamped.pose.orientation.w =  pushing_poses[i].pose_dir1.quaternion.w();

		pushing_poses_msg[i].pose_dir1 = pose_stamped;

		pose_stamped.pose.position.x = pushing_poses[i].pose_dir2.translation[0];
		pose_stamped.pose.position.y = pushing_poses[i].pose_dir2.translation[1];
		pose_stamped.pose.position.z = pushing_poses[i].pose_dir2.translation[2];

		pose_stamped.pose.orientation.x =  pushing_poses[i].pose_dir2.quaternion.x();
		pose_stamped.pose.orientation.y =  pushing_poses[i].pose_dir2.quaternion.y();
		pose_stamped.pose.orientation.z =  pushing_poses[i].pose_dir2.quaternion.z();
		pose_stamped.pose.orientation.w =  pushing_poses[i].pose_dir2.quaternion.w();

		pushing_poses_msg[i].pose_dir2 = pose_stamped;

		pose_stamped.pose.position.x = pushing_poses[i].pose_dir3.translation[0];
		pose_stamped.pose.position.y = pushing_poses[i].pose_dir3.translation[1];
		pose_stamped.pose.position.z = pushing_poses[i].pose_dir3.translation[2];

		pose_stamped.pose.orientation.x =  pushing_poses[i].pose_dir3.quaternion.x();
		pose_stamped.pose.orientation.y =  pushing_poses[i].pose_dir3.quaternion.y();
		pose_stamped.pose.orientation.z =  pushing_poses[i].pose_dir3.quaternion.z();
		pose_stamped.pose.orientation.w =  pushing_poses[i].pose_dir3.quaternion.w();

		pushing_poses_msg[i].pose_dir3 = pose_stamped;

		pose_stamped.pose.position.x = pushing_poses[i].pose_dir4.translation[0];
		pose_stamped.pose.position.y = pushing_poses[i].pose_dir4.translation[1];
		pose_stamped.pose.position.z = pushing_poses[i].pose_dir4.translation[2];

		pose_stamped.pose.orientation.x =  pushing_poses[i].pose_dir4.quaternion.x();
		pose_stamped.pose.orientation.y =  pushing_poses[i].pose_dir4.quaternion.y();
		pose_stamped.pose.orientation.z =  pushing_poses[i].pose_dir4.quaternion.z();
		pose_stamped.pose.orientation.w =  pushing_poses[i].pose_dir4.quaternion.w();

		pushing_poses_msg[i].pose_dir4 = pose_stamped;
	}

	return pushing_poses_msg;
}
void TableClearingPredicatesAlgorithm::reset()
{
	this->tcp.reset();
}
std::vector<iri_table_clearing_predicates::AABB> TableClearingPredicatesAlgorithm::getAABBMsg()
{
	std::vector<iri_table_clearing_predicates::AABB> aabb_msgs;
	std::vector<AABB> aabb = this->tcp.getAABBObjects();
	for (int i = 0; i < aabb.size() ; ++i)
	{
		iri_table_clearing_predicates::AABB aabb_msg;	
		aabb_msg.width = aabb[i].width; 
		aabb_msg.height = aabb[i].height; 
		aabb_msg.deep = aabb[i].deep; 
		aabb_msgs.push_back(aabb_msg);
	}
	return aabb_msgs;
}
std::vector<geometry_msgs::Point> TableClearingPredicatesAlgorithm::getCentroids()
{
	std::vector<geometry_msgs::Point> centroids_msg;
	std::vector<PrincipalDirectionsProjected> pdp = this->tcp.getProjectedPrincipalDirections();
	for (int i = 0; i < pdp.size(); ++i)
	{
		geometry_msgs::Point centroid_msg;
		centroid_msg.x = pdp[i].centroid[0];
		centroid_msg.y = pdp[i].centroid[1];
		centroid_msg.z = pdp[i].centroid[2];
		centroids_msg.push_back(centroid_msg);
	}

	return centroids_msg;
}
std::vector<iri_table_clearing_predicates::PrincipalDirections> TableClearingPredicatesAlgorithm::getPrincipalDirections()
{
	std::vector<OriginalPrincipalDirections> opd = this->tcp.getOriginalPrincipalDirections();	
	std::vector<iri_table_clearing_predicates::PrincipalDirections> principal_directions_msg;
	
	for (int i = 0; i < opd.size(); ++i)
	{
		iri_table_clearing_predicates::PrincipalDirections p_tmp;
		p_tmp.p1.x = opd[i].p1[0];
		p_tmp.p1.y = opd[i].p1[1];
		p_tmp.p1.z = opd[i].p1[2];

		p_tmp.p2.x = opd[i].p2[0];
		p_tmp.p2.y = opd[i].p2[1];
		p_tmp.p2.z = opd[i].p2[2];

		p_tmp.p3.x = opd[i].p3[0];
		p_tmp.p3.y = opd[i].p3[1];
		p_tmp.p3.z = opd[i].p3[2];

		principal_directions_msg.push_back(p_tmp);
	}

	return principal_directions_msg;
}
void TableClearingPredicatesAlgorithm::setOnTopParameters(double on_th1, double on_th2)
{
	this->on_th1 = on_th1;
	this->on_th2 = on_th2;
}
ExecutionTimes TableClearingPredicatesAlgorithm::getExecutionTimes()
{
	return this->tcp.getExecutionTimes();
}