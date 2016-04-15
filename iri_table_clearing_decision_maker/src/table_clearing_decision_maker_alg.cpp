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
std::vector<iri_fast_downward_wrapper::Object> TableClearingDecisionMakerAlgorithm::prepareObjectsMsg(uint n_objects)
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
void TableClearingDecisionMakerAlgorithm::prepareBlockPredicatesMsg()
{

}
void TableClearingDecisionMakerAlgorithm::prepareBlockGraspPredicatesMsg()
{

}
void TableClearingDecisionMakerAlgorithm::prepareOnTopPredicatesMsg()
{

}
std::string TableClearingDecisionMakerAlgorithm::prepareGoalMsg()
{
	std::string goal;

	return goal;
}