#include "table_clearing_predicates_alg.h"

TableClearingPredicatesAlgorithm::TableClearingPredicatesAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
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

void TableClearingPredicatesAlgorithm::setFingersModel(double opening_width, double finger_width,
               double deep, double height, double closing_height)
{
	this->tcp.setFingersModel(opening_width, finger_width, deep, height, closing_height);
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
void TableClearingPredicatesAlgorithm::setPushingLimit(double pushing_limit)
{
	this->tcp.setPushingLimit(pushing_limit);
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

void TableClearingPredicatesAlgorithm::computeOnTopPredicates(bool print)
{
	this->tcp.computeOnTopPredicates(print);
}

void TableClearingPredicatesAlgorithm::computeBlockPredicates(bool print)
{
	this->tcp.computeBlockPredicates(print);
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

std::vector<iri_table_clearing_predicates::BlockPredicate> TableClearingPredicatesAlgorithm::getBlockPredicates()
{
	std::vector<BlocksPredicate> block_predicates = this->tcp.getBlockPredicates();
	std::vector<iri_table_clearing_predicates::BlockPredicate> block_predicates_msgs;
	
	block_predicates_msgs.resize(block_predicates.size());
	for (int i = 0; i < block_predicates_msgs.size(); ++i)
	{
		// dir 1
		for (int p = 0; p < block_predicates[i].block_dir1.size(); ++p)
			block_predicates_msgs[i].dir1.push_back(block_predicates[i].block_dir1[p]);
		// dir 2
		for (int p = 0; p < block_predicates[i].block_dir2.size(); ++p)
			block_predicates_msgs[i].dir2.push_back(block_predicates[i].block_dir2[p]);
		// dir 3
		for (int p = 0; p < block_predicates[i].block_dir3.size(); ++p)
			block_predicates_msgs[i].dir3.push_back(block_predicates[i].block_dir3[p]);
		// dir 4
		for (int p = 0; p < block_predicates[i].block_dir4.size(); ++p)
			block_predicates_msgs[i].dir4.push_back(block_predicates[i].block_dir4[p]);
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
		for (int p = 0; p < block_grasp_predicates[i].size(); ++p)
		{
			block_grasp_predicates_msg[i].object.push_back(block_grasp_predicates[i][p]);
		}
	}

	return block_grasp_predicates_msg;
}

// std::vector<PrincipalDirectionsProjected> TableClearingPredicatesAlgorithm::getProjectedPrincipalDirections()
// {
// 	return this->tcp.getProjectedPrincipalDirections();
// }
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
