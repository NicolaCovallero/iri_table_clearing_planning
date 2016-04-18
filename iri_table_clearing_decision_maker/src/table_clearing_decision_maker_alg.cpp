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
	for (int i = 0; i < n_objects; ++i)
	{
		iri_fast_downward_wrapper::Object object;
		std::string object_name = "o";
		std::ostringstream convert;   // stream used for the conversion
		convert << i;
		object_name += convert.str();
		object.object_name = object_name;
		object.type = "obj";
	}
	return objects_msg;
}
std::vector<iri_fast_downward_wrapper::SymbolicPredicate> TableClearingDecisionMakerAlgorithm::prepareSymbolicPredicatesMsg()
{
	std::vector<iri_fast_downward_wrapper::SymbolicPredicate> blocks_predicates_msg;
	iri_fast_downward_wrapper::SymbolicPredicate tmp;

	// ---------------- Add Blocks Predicates ---------------------------------
	tmp.predicate_name = "block";
	for (int i = 0; i < this->blocks_predicates.size(); ++i)
	{
		tmp.objects.resize(2); // the block predicates only involves 2 objects
		std::string object_name = "o";
		std::ostringstream convert;   // stream used for the conversion
		convert << i;
		object_name += convert.str();
		tmp.objects[0] = object_name;
		for (int p = 0; p < this->blocks_predicates[i].dir1.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->blocks_predicates[i].dir1[p];
			object_name += convert.str();
			tmp.objects[1] = object_name;
			blocks_predicates_msg.push_back(tmp);
		}
		for (int p = 0; p < this->blocks_predicates[i].dir2.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->blocks_predicates[i].dir2[p];
			object_name += convert.str();
			tmp.objects[1] = object_name;	
			blocks_predicates_msg.push_back(tmp);
		}
		for (int p = 0; p < this->blocks_predicates[i].dir3.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->blocks_predicates[i].dir3[p];
			object_name += convert.str();
			tmp.objects[1] = object_name;
			blocks_predicates_msg.push_back(tmp);
		}
		for (int p = 0; p < this->blocks_predicates[i].dir4.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->blocks_predicates[i].dir4[p];
			object_name += convert.str();
			tmp.objects[1] = object_name;
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
		tmp.objects[0] = object_name;
		for (int p = 0; p < this->block_grasp_predicates[i].object.size(); ++p)
		{
			std::string object_name = "o";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->block_grasp_predicates[i].object[p];
			object_name += convert.str();
			tmp.objects[1] = object_name;
			blocks_predicates_msg.push_back(tmp);
		}
	}

 	return blocks_predicates_msg;
}

std::string TableClearingDecisionMakerAlgorithm::prepareGoalMsg()
{
	std::string goal;

	return goal;
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