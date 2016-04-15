#include "iri_table_clearing_predicates_alg_node.h"

IriTableClearingPredicatesAlgNode::IriTableClearingPredicatesAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<IriTableClearingPredicatesAlgorithm>()
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

IriTableClearingPredicatesAlgNode::~IriTableClearingPredicatesAlgNode(void)
{
  // [free dynamic memory]
}

void IriTableClearingPredicatesAlgNode::mainNodeThread(void)
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

void IriTableClearingPredicatesAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void IriTableClearingPredicatesAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriTableClearingPredicatesAlgNode>(argc, argv, "iri_table_clearing_predicates_alg_node");
}
