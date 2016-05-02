#include "test_trajectory_alg.h"

TestTrajectoryAlgorithm::TestTrajectoryAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TestTrajectoryAlgorithm::~TestTrajectoryAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TestTrajectoryAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TestTrajectoryAlgorithm Public API
