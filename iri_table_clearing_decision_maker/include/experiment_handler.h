#ifndef _ExperimentDataHandler_
#define _ExperimentDataHandler_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <ctime>
#include "iri_fast_downward_wrapper/Plan.h"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>



/**
 * @brief Class to handle the experiments data
 * @details Handle the experiments data writing a file with a specific format.
 * 
 */
class ExperimentDataHandler{

	/**
	 * Number of the current experiment
	 */
	uint exp_num; 

	/**
	 * Number of the action of the current experiment
	 */
	uint exp_iteration;

	std::string working_folder;
	std::string experiment_name;

	std::ofstream file;


	std::vector<std::string> labels;

	cv::Mat image;

    // TO DO:
    // 1) add the total solution time at the end
	// 2) write labels of the objects on the image
	// 3) show pushing direction 

	public:

		ExperimentDataHandler();

		~ExperimentDataHandler();

		void setUp(std::string working_folder);

		/**
		 * @brief Update the experiment number
		 * @details It just updates the exp_num member in order to make 
		 * the algorithm know it is going to do a new experiments. This has to be called externally.
		 */
		void newExperiment();
		void closeFile();


		// the function does not do any control over the size of data

		/**
		 * @brief Update the data of the experiment
		 * @details Update the data of the experiment
		 * 
		 * @param data data to save (the labels have to be specified in the cosntructor)
		 * @param plan The plan
		 * @param ik_feasible boolean variable to specify if the ik is feasible or not
		 * @param image_ptr [description]
		 */
		void updateExperiment(std::vector<double>& data,
		               iri_fast_downward_wrapper::Plan& plan,
		                bool ik_feasible,
		                cv_bridge::CvImagePtr image_ptr);


};

#endif