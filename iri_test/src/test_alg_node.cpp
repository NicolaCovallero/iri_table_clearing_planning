#include "test_alg_node.h"

TestAlgNode::TestAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TestAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  this->execute_pushing_server_ = this->public_node_handle_.advertiseService("execute_pushing", &TestAlgNode::execute_pushingCallback, this);
  pthread_mutex_init(&this->execute_pushing_mutex_,NULL);

  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TestAlgNode::~TestAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->execute_pushing_mutex_);
}

void TestAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool TestAlgNode::execute_pushingCallback(iri_table_clearing_execute::ExecutePushing::Request &req, iri_table_clearing_execute::ExecutePushing::Response &res)
{
  ROS_INFO("TestAlgNode::execute_pushingCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->execute_pushing_mutex_enter();

  //ROS_INFO("TestAlgNode::execute_pushingCallback: Processing New Request!");
  //do operations with req and output on res
  //res.data2 = req.data1 + my_var;

  //unlock previously blocked shared variables
  //this->execute_pushing_mutex_exit();
  //this->alg_.unlock();

  return true;
}

void TestAlgNode::execute_pushing_mutex_enter(void)
{
  pthread_mutex_lock(&this->execute_pushing_mutex_);
}

void TestAlgNode::execute_pushing_mutex_exit(void)
{
  pthread_mutex_unlock(&this->execute_pushing_mutex_);
}


/*  [action callbacks] */

/*  [action requests] */

void TestAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TestAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TestAlgNode>(argc, argv, "test_alg_node");
}
