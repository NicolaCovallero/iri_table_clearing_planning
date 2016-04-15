#include "table_clearing_decision_maker_alg_node.h"

TableClearingDecisionMakerAlgNode::TableClearingDecisionMakerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TableClearingDecisionMakerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 2;//in [Hz]

  ros::service::waitForService("/segments_objects",2);// 2 seconds
  ros::service::waitForService("iri_table_clearing_predicates/get_symbolic_predicates",2);

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  segments_objects_client_ = this->public_node_handle_.serviceClient<iri_tos_supervoxels::object_segmentation>("/segments_objects");

  get_symbolic_predicates_client_ = this->public_node_handle_.serviceClient<iri_table_clearing_predicates::Predicates>("iri_table_clearing_predicates/get_symbolic_predicates");
  
  

  // [init action servers]
  
  // [init action clients]
}

TableClearingDecisionMakerAlgNode::~TableClearingDecisionMakerAlgNode(void)
{
  // [free dynamic memory]
}

void TableClearingDecisionMakerAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  //segments_objects_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (segments_objects_client_.call(segments_objects_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", segments_objects_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic segments_objects ");
  //}


  //get_symbolic_predicates_srv_.request.data = my_var;
  //ROS_INFO("TableClearingDecisionMakerAlgNode:: Sending New Request!");
  //if (get_symbolic_predicates_client_.call(get_symbolic_predicates_srv_))
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Response: %s", get_symbolic_predicates_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("TableClearingDecisionMakerAlgNode:: Failed to Call Server on topic get_symbolic_predicates ");
  //}


  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TableClearingDecisionMakerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TableClearingDecisionMakerAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TableClearingDecisionMakerAlgNode>(argc, argv, "table_clearing_decision_maker_alg_node");
}
