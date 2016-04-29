#include "table_clearing_push_execution_alg_node.h"

TableClearingPushExecutionAlgNode::TableClearingPushExecutionAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingPushExecutionAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TableClearingPushExecutionAlgNode::~TableClearingPushExecutionAlgNode(void)
{
  // [free dynamic memory]
}

void TableClearingPushExecutionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TableClearingPushExecutionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TableClearingPushExecutionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TableClearingPushExecutionAlgNode>(argc, argv, "table_clearing_push_execution_alg_node");
}
