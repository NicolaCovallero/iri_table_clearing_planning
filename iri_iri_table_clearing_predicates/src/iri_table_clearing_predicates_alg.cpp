#include "iri_table_clearing_predicates_alg.h"

IriTableClearingPredicatesAlgorithm::IriTableClearingPredicatesAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

IriTableClearingPredicatesAlgorithm::~IriTableClearingPredicatesAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void IriTableClearingPredicatesAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// IriTableClearingPredicatesAlgorithm Public API
