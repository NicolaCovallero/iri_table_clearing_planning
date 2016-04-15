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
