#include "table_clearing_execute_alg.h"

TableClearingExecuteAlgorithm::TableClearingExecuteAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TableClearingExecuteAlgorithm::~TableClearingExecuteAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TableClearingExecuteAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TableClearingExecuteAlgorithm Public API
