#include "table_clearing_push_execution_alg.h"

TableClearingPushExecutionAlgorithm::TableClearingPushExecutionAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TableClearingPushExecutionAlgorithm::~TableClearingPushExecutionAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TableClearingPushExecutionAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TableClearingPushExecutionAlgorithm Public API
