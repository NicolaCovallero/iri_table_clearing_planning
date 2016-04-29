#include "table_clearing_grasp_execution_alg.h"

TableClearingGraspExecutionAlgorithm::TableClearingGraspExecutionAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TableClearingGraspExecutionAlgorithm::~TableClearingGraspExecutionAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TableClearingGraspExecutionAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TableClearingGraspExecutionAlgorithm Public API
