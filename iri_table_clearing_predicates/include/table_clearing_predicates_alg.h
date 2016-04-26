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

#ifndef _table_clearing_predicates_alg_h_
#define _table_clearing_predicates_alg_h_

#include <iri_table_clearing_predicates/TableClearingPredicatesConfig.h>

//include table_clearing_predicates_alg main library
#include "table_clearing_planning.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "iri_table_clearing_predicates/BlockPredicate.h"
#include "iri_table_clearing_predicates/BlockGraspPredicate.h"
#include "iri_table_clearing_predicates/OnTopPredicate.h"
#include "iri_table_clearing_predicates/PushingDirections.h"
#include "iri_table_clearing_predicates/GraspingPoses.h"
#include "iri_table_clearing_predicates/PushingPoses.h"
#include "iri_table_clearing_predicates/AABB.h"
#include "iri_table_clearing_predicates/PrincipalDirections.h"


/** default values */
const double  OPENING_WIDTH = 0.08;
const double  CLOSING_WIDTH = 0.03;
const double  FINGER_WIDTH = 0.02;
const double  GRIPPER_HEIGHT = 0.08;
const double  CLOSING_REGION_HEIGHT = 0.05;
const double  FINGER_DEEP = 0.03;
const double  PUSHING_DISTANCE_PLANE = 0.025;
const double  EE_HEIGHT = 0.08;
const double  EE_DEEP = 0.15;
const double  PUSHING_OBJECT_DISTANCE = 0.05;
const int PUSHING_METHOD = ORTHOGONAL_PUSHING;

const double PUSHING_STEP = 1.5; // push 1.5 times along the relative AABB dimension

// default values variables 
const double ON_TH1 = 100;
const double ON_TH2 = 100;

/**
 * 
 * 
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class TableClearingPredicatesAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the TableClearingPredicatesConfig. All driver implementations
    * will then use the same variable type Config.
    */
    pthread_mutex_t access_;    

    // private attributes and methods
    CTableClearingPlanning tcp;

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    double on_th1, on_th2;

    int pushing_method;

  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the TableClearingPredicatesConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_table_clearing_predicates::TableClearingPredicatesConfig Config;

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
    TableClearingPredicatesAlgorithm(void);

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

    // here define all table_clearing_predicates_alg interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~TableClearingPredicatesAlgorithm(void);


    /**
     * @brief Set the gripper model
     * @details Set the dimension of the bounding box of the gripper. This model will be used
     *          during the computation of the block predicates
     * 
     * @param[in] height Dimension of the gripper orthogonal to the plane during the pushing action
     * @param[in] deep Dimension of the gripper parallel to pushing direction during the pushing action
     * @param[in] width Dimension of the gripper orthogonal to pushing direction during the pushing action
     * @param[in] distance_plane Distance in meters from the table plane during the pushing action
     */
    void setGripperSimpleModel(double height, double deep, double width, double distance_plane);

        /**
     * @brief Set the fingers model
     * @details Set the fingers model. The origin is located at height/2, deep/2 and (opening_width + finger_width*2)/2
     * 
     * @image html fingers_model.png
     * 
     * @param opening_width 
     * @param finger_width [description]
     * @param deep [description]
     * @param height [description]
     * @param closing_height [description]
     */
    void setFingersModel(double opening_width,double closing_width, double finger_width,
               double deep, double height, double closing_height);

    void setPushingMethod(double pushing_method);

    /**
     * @brief Set the original point cloud, the one used for the segmentation. 
     * 
     * @param[in] original_cloud 
     */
    void setOriginalPointCloud(PointCloudT original_cloud);


    /**
     * @brief      set point clouds of the objects, reference to project the principal direction to the   plane:
     *             http://www.maplesoft.com/support/help/Maple/view.aspx?path=MathApps%2FProjectionOfVectorOntoPlane
     *
     * @param[in]  objects  vector of point clouds, one per object
     */
    void setObjectsPointCloud(std::vector<PointCloudT > &objects);

    /**
     * @brief      Set the coefficients of the table plane
     *
     * @param[in]  plane_coefficients  coefficients of the table plane
     */
    void setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients);

    /**
     * @brief Set the pushing limit
     * @details If it is not setted a default value will be used. The pushing limit is here 
     *          defined as the maximum length (in meters) you want to push the object and therefore 
     *          it acts on the block predicates computation.
     * 
     * @param[in] pushing_limit Desired value for the pushing limit.
     */
    void setPushingStep(double pushing_step);

    /**
     * 
     * @details Distance between the tcp and the object, during the pushing action
     * 
     * @param pushing_object_distance 
     */
    void setPushingObjectDistance(double pushing_object_distance);

    /**
     * @brief    compute the principal directions of the objects.
     * @details  This is better
     *           because the others that use the hulls the amount of points of the hulls 
     *           depends on the complexity of the shape, so the principal directions would
     *           be affected by the complexity of the shape and not on the quanrity of points. 
     *           This method also include a verse detection in order to have alway dir3 to
     *           the right of dir1 with respect the top of the table. 
     * @image html directions.jpg  
     */
    void computePrincipalDirections();

    /**
     * @brief Compute the Axis Aligned Bounding Box of each object
     * 
     * @param refine_centroids True if you want to refine the centroid by computing the mean of the bounding box
     */
    void computeAABBObjects(bool refine_centroids = true);  

    /**
     * @brief Compute the grasping pose with a simple heuristic.
     * @details Compute the grasping pose with a simple heuristic. 
     * Gripper centered at the centroid with rotation aligned to the principal axes.
     * The contact point is choosen to be the nearest point which projection on the table plane is 
     * the enarest on the projection of the centroid.
     * 
     * @param vertical_poses If it is set to true consider only vertical poses, with respect the table.
     */
    void computeSimpleHeuristicGraspingPoses(bool vertical_poses = false);

    /**
     * @brief      Compute the projections of the objects onto the table plane
     */
    void computeProjectionsOnTable();

    /**
     * @brief      Compute the convex hulls of the projections onto the table plane
     */
    void computeProjectionsConvexHull();


    /**
     * @brief      Compute the convex hull of each object considering also the projections of the 
     *             convex hulls on the plane. These points are then added to the original point cloud 
     *             and the convex hull is recomputed.
     *             IMPORTANT: we can project all the coud and do the convex hull, so it is computed only once
     */
    void computeRichConvexHulls();

    /**
     * @brief      Compute the on predicates
     * 
     *             To decide if an oject is on top ot the other one, we work with the convex hulls
     *              of the projections and the projected points. 
     *              Since the pow of the kinect make that the on top objects hide part of the bottom objects
     *              the strategy is the following:
     *                1) for objet 1 check if its points (choose randomly) lies inside the convex hull of the other one
     *                2) if a point lieas inside the convex hull of the other one, tdo the same with the object 2.
     *                3) if object 1 lies inside the convex hull of the object 2, but object 2 does'nt, the object 1 is on
     *                   top of object 2. MAYBE A EROSION TECHNIQUE WILL BE REQUIRED IN ORDER TO AVOID SITUATIONS WHERE 
     *                   THE PROJECTED POINTS ARE TOO MUCH CLOSE
     *                
     */
    void computeOnTopPredicates(double th1, double th2, bool print = false);

        /**
     * @brief Get block predicates
     * @details This method computes the block predicates for a each object by detecting
     *          collision with the other objects, determining also what object collides 
     *          with the current one, along each principal direction. The length of the push
     *          considered in saved in the private memeber 
     */
    void computeBlockPredicates(bool print=false);

    /**
     * @brief Get block grasp predicates
     * @details This method computes the block grasp predicates for each object by detecting 
     *          collision of the fingers model with the other objects.
     * 
     * @param print True if you want to print in the terminal the predicates.
     */
    void computeBlockGraspPredicates(bool print=false);

    uint getNumObjects();
    std::vector<ObjectFull> getFullObjects();

    std::vector<AABB> getAABBObjects();

    std::vector<iri_table_clearing_predicates::BlockPredicate> getBlockPredicates();

    std::vector<iri_table_clearing_predicates::OnTopPredicate> getOnTopPredicates();  
 
    std::vector<iri_table_clearing_predicates::BlockGraspPredicate> getBlockGraspPredicates();

    /**
     * @details Get the projected principal directions
     * @return projected principal directions
     */
    std::vector<iri_table_clearing_predicates::PushingDirections> getPushingDirections();

    std::vector<iri_table_clearing_predicates::GraspingPoses> getGraspingPoses();

    std::vector<iri_table_clearing_predicates::PushingPoses> getPushingPoses();

    std::vector<iri_table_clearing_predicates::AABB> getAABBMsg();

    std::vector<geometry_msgs::Point> getCentroids();

    std::vector<iri_table_clearing_predicates::PrincipalDirections> getPrincipalDirections();

    void setOnTopParameters(double on_th1, double on_th2);

    void reset();
};

#endif
