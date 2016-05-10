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

#ifndef _table_clearing_decision_maker_alg_h_
#define _table_clearing_decision_maker_alg_h_

#include <iri_table_clearing_decision_maker/TableClearingDecisionMakerConfig.h>

//include table_clearing_decision_maker_alg main library
#include "iri_table_clearing_predicates/BlockPredicate.h"
#include "iri_table_clearing_predicates/BlockGraspPredicate.h"
#include "iri_table_clearing_predicates/OnTopPredicate.h"
#include "iri_table_clearing_predicates/PushingDirections.h"
#include "iri_table_clearing_predicates/GraspingPoses.h"
#include "iri_table_clearing_predicates/PushingPoses.h"
#include "iri_table_clearing_predicates/AABB.h"
#include "iri_tos_supervoxels/plane_coefficients.h"
#include "iri_table_clearing_predicates/PrincipalDirections.h"
#include "iri_table_clearing_execute/ExecutePushing.h"
#include "iri_table_clearing_execute/ExecuteGrasping.h"



#include "iri_fast_downward_wrapper/Object.h"
#include "iri_fast_downward_wrapper/SymbolicPredicate.h"

#include "iri_fast_downward_wrapper/Plan.h"

#include "sensor_msgs/PointCloud2.h"

#include "table_clearing_planning.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>

#include <string>

#include <tf/transform_datatypes.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include "experiment_handler.h"

const std::string GOAL = "(not (exists (?x - obj)(not (grasped ?x))))";

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class TableClearingDecisionMakerAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the TableClearingDecisionMakerConfig. All driver implementations
    * will then use the same variable type Config.
    */
    pthread_mutex_t access_;    

    // private attributes and methods

    /**
     * Number of objects
     */
    uint n_objects;

    struct IKUnfeasiblePredicate{

        std::string action;
        std::string object;
    };


    // std::vector<BlocksPredicate> blocks_predicates;
    // std::vector<std::vector<uint> > on_top_predicates;
    // std::vector<std::vector<uint> > block_grasp_predicates;
    std::vector<iri_table_clearing_predicates::BlockPredicate> blocks_predicates;
    std::vector<iri_table_clearing_predicates::OnTopPredicate> on_top_predicates;
    std::vector<iri_table_clearing_predicates::BlockGraspPredicate> block_grasp_predicates;
    std::vector<iri_table_clearing_predicates::PushingDirections> pushing_directions;
    std::vector<iri_table_clearing_predicates::GraspingPoses> grasping_poses;
    std::vector<iri_table_clearing_predicates::GraspingPoses> approaching_poses;
    std::vector<iri_table_clearing_predicates::PushingPoses> pushing_poses;
    std::vector<iri_table_clearing_predicates::AABB> aabbs;
    std::vector<IKUnfeasiblePredicate> ik_unfeasible_predicates;
    std::vector<geometry_msgs::Point> centroids;
    iri_tos_supervoxels::plane_coefficients plane_coefficients;
    geometry_msgs::Vector3 plane_normal;
    std::vector<iri_table_clearing_predicates::PrincipalDirections> principal_directions;

    std::string frame_id;   

    sensor_msgs::PointCloud2 point_cloud;

    bool set;

    int pushing_discretization;
    double pushing_step;
    double pushing_object_distance;

    std::vector<geometry_msgs::PoseStamped> pushing_cartesian_trajectory; 

    geometry_msgs::PoseStamped dropping_pose,pre_dropping_pose;

  public:

    iri_fast_downward_wrapper::Plan plan;
    sensor_msgs::Image  image;

    std::string goal;
    bool filtering;

   /**
    * \brief define config type
    *
    * Define a Config type with the TableClearingDecisionMakerConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_table_clearing_decision_maker::TableClearingDecisionMakerConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    TableClearingDecisionMakerAlgorithm(void);

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
    void lock(void) { pthread_mutex_lock(&this->access_); };

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
    void unlock(void) { pthread_mutex_unlock(&this->access_); };

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    * 
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void) 
    { 
      if(pthread_mutex_trylock(&this->access_)==0)
        return true;
      else
        return false;
    };

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
    void config_update(Config& config, uint32_t level=0);

    // here define all table_clearing_decision_maker_alg interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~TableClearingDecisionMakerAlgorithm(void);

    /**
     * @brief Prepare the object message
     * @details [long description]
     * 
     * @param n_objects [description]
     * @return [description]
     */
    std::vector<iri_fast_downward_wrapper::Object> prepareObjectsMsg();

    /**
     * @brief Prepare the Symbolic message for the iri_clearing_predicates server
     * @details Given the vectors of predicates it creates the Symbolic message of all the predicates.
     * 
     * @param block_predicates
     * @param on_top_predicates
     * @param block_grasp_predicates
     * @return Symbolic predicate message
     */
    std::vector<iri_fast_downward_wrapper::SymbolicPredicate> prepareSymbolicPredicatesMsg();

    /**
     * @brief Prepare Goal message
     * @details [long description]
     * @return [description]
     */
    std::string prepareGoalMsg();

    void setNumberObjects(uint n_objects);

    void setBlockPredicates(std::vector<iri_table_clearing_predicates::BlockPredicate> blocks_predicates);
    void setOnTopPredicates(std::vector<iri_table_clearing_predicates::OnTopPredicate> on_top_predicates);
    void setBlockGraspPredicates(std::vector<iri_table_clearing_predicates::BlockGraspPredicate> block_grasp_predicates);
    void setPushingDirections(std::vector<iri_table_clearing_predicates::PushingDirections> pushing_directions);
    void setGraspingPoses(std::vector<iri_table_clearing_predicates::GraspingPoses> grasping_poses);
    void setApproachingPoses(std::vector<iri_table_clearing_predicates::GraspingPoses> approaching_poses);
    void setPushingPoses(std::vector<iri_table_clearing_predicates::PushingPoses> pushing_poses); 
    void setAABBs(std::vector<iri_table_clearing_predicates::AABB> aabbs);

    void setPlan(iri_fast_downward_wrapper::Plan plan);
    void setFrameId(std::string frame_id);
    void setCentroids(std::vector<geometry_msgs::Point> centroids);
    void setPlaneCoefficients(iri_tos_supervoxels::plane_coefficients plane_coefficients);
    void setPrincipalDirections(std::vector<iri_table_clearing_predicates::PrincipalDirections> principal_directions);

    void setPushingObjectDistance(double pushing_object_distance);

    visualization_msgs::Marker firstActionMarker();

    void showObjectsRViz(std::vector<sensor_msgs::PointCloud2> segmented_objects, std_msgs::Header header, ros::Publisher& cloud_publisher_);

    void showObjectsLabelRViz(std::vector<geometry_msgs::Point> centroids,
                            visualization_msgs::MarkerArray& objects_labels_markers,
                              std::vector<iri_table_clearing_predicates::AABB> aabbs);

    void showFirstActionRViz(ros::Publisher& action_pub);

    void showActionTrajectory(ros::Publisher& trajectory_pub);

    void setOn(bool on);
    bool getOn();
    void setPointCloud(sensor_msgs::PointCloud2 point_cloud);
    sensor_msgs::PointCloud2* getPointCloud();

    void setPushingDiscretizationAndStep(int pushing_discretization, double pushing_step);

    void setDroppingPose(double dropping_pose_x,double dropping_pose_y,double dropping_pose_z);
    void setPreDroppingPose(double dropping_pose_x,double dropping_pose_y,double dropping_pose_z);
    /**
     * @brief set the ik_unfeasible predicate for the current first action
     * @details 
     */
    void setIKUnfeasiblePredicate();

    /**
     * @brief It returns 1 for the grasping action 
     * or 0 for the pushing, and -1 for a non accepted action, or -2 in case of error,
     * or -3 in case there is not plan set
     * @details [long description]
     * 
     * @param grasping [description]
     * @param pushing [description]
     * 
     * @return [description]
     */
    int setAction( iri_table_clearing_execute::ExecuteGrasping& grasping,
                    iri_table_clearing_execute::ExecutePushing& pushing);

    /**
     * @brief Return the number of actions of the plan
     * @details [long description]
     * @return [description]
     */
    int getPlanLength();

    /**
     * @details Clear predicates
     */
    void resetPredicates();

};

#endif