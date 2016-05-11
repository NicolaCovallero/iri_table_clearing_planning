// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _table_clearing_decision_maker_alg_node_h_
#define _table_clearing_decision_maker_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "table_clearing_decision_maker_alg.h"

// [publisher subscriber headers]
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

// [service client headers]
#include <iri_table_clearing_execute/ExecuteGrasping.h>
#include <iri_table_clearing_execute/ExecutePushing.h>
#include <iri_fast_downward_wrapper/FastDownwardPlan.h>
#include <iri_tos_supervoxels/object_segmentation.h>
#include <iri_table_clearing_predicates/Predicates.h>

#include "experiment_handler.h"

// [action server client headers]


const std::string FRAME_ID = "/base_link";
const std::string INPUT_TOPIC = "/camera/depth_registered/points";
const int PUSHING_DISCRETIZATION = 10; // 10 points 
const double PUSHING_STEP = 1.5; // 0.1 meters

const std::string SEGMENTATION_SERVICE = "/iri_tos_supervoxels_alg/object_segmentation";
const std::string PREDICATES_SERVICE = "/table_clearing_predicates_alg_node/get_symbolic_predicates";
const std::string PLANNER_SERVICE = "/get_fast_downward_plan";
const std::string EXECUTE_PUSHING_SERVICE = "/table_clearing_execute_alg_node/execute_pushing";
const std::string EXECUTE_GRASPING_SERVICE = "/table_clearing_execute_alg_node/execute_grasping";

const std::string WORKING_FOLDER = "~/tests";

const bool EXECUTION = false;
const bool FILTERING = true;

/**
 * Default Position To Drop the object
 */
const double DROPPING_POSE_X = 0.5;
const double DROPPING_POSE_Y = 0.3;
const double DROPPING_POSE_Z = 0.5;
const double PRE_DROPPING_POSE_X = 0.5;
const double PRE_DROPPING_POSE_Y = 0.3;
const double PRE_DROPPING_POSE_Z = 0.5;

// services name
std::string segmentation_service, predicates_service, planner_service, execute_pushing_service, execute_grasping_service;

bool execution; //true if it is the execution wanted


/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class TableClearingDecisionMakerAlgNode : public algorithm_base::IriBaseAlgorithm<TableClearingDecisionMakerAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher action_trajectory_publisher_;
    visualization_msgs::MarkerArray action_trajectory_MarkerArray_msg_;

    ros::Publisher action_marker_publisher_;
    visualization_msgs::Marker action_Marker_msg_;

    ros::Publisher objects_label_publisher_;
    visualization_msgs::MarkerArray objects_label_MarkerArray_msg_;

    ros::Publisher cloud_publisher_;
    sensor_msgs::PointCloud2 cloud_PointCloud2_msg_;


    // [subscriber attributes]

    ros::Subscriber kinect_subscriber_;
    void kinect_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    pthread_mutex_t kinect_mutex_;
    void kinect_mutex_enter(void);
    void kinect_mutex_exit(void);


    // [service attributes]

    // [client attributes]
    ros::ServiceClient execute_grasping_client_;
    iri_table_clearing_execute::ExecuteGrasping execute_grasping_srv_;

    ros::ServiceClient execute_pushing_client_;
    iri_table_clearing_execute::ExecutePushing execute_pushing_srv_;

    ros::ServiceClient get_fast_downward_plan_client_;
    iri_fast_downward_wrapper::FastDownwardPlan get_fast_downward_plan_srv_;

    ros::ServiceClient segments_objects_client_;
    iri_tos_supervoxels::object_segmentation segments_objects_srv_;

    ros::ServiceClient get_symbolic_predicates_client_;
    iri_table_clearing_predicates::Predicates get_symbolic_predicates_srv_;


    // [action server attributes]

    // [action client attributes]


    // messages to publish
    visualization_msgs::MarkerArray objects_labels_markers;

    // experiment stuff
    bool save_experiment;
    std::string working_folder;
    ExperimentDataHandler eh;
    double planning_time,segmentation_time,ik_time,predicates_time;
    double  on_predicates_time, block_predicates_time, block_grasp_predicates_time,
            objects_collisions_time, ee_collisions_time, average_objects_collision_time,
            average_ee_collision_time;
    bool plan_feasible,ik_feasible;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    TableClearingDecisionMakerAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~TableClearingDecisionMakerAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
    void preparePredicatesMsg();

};

#endif
