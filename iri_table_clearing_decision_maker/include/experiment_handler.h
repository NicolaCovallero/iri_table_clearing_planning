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


	public:

		ExperimentDataHandler(std::string working_folder);

		~ExperimentDataHandler();



		/**
		 * @brief Update the experiment number
		 * @details It just updates the exp_num member in order to make 
		 * the algorithm know it is going to do a new experiments. This has to be called externally.
		 */
		void updateExperimentNumber();
};

#endif