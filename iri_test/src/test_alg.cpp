#include "test_alg.h"

TestAlgorithm::TestAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TestAlgorithm::~TestAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TestAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TestAlgorithm Public API
