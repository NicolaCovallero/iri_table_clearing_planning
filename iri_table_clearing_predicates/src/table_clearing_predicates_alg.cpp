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
