#include "experiment_handler.h"

using namespace std;

ExperimentDataHandler::ExperimentDataHandler()
{

}

ExperimentDataHandler::~ExperimentDataHandler()
{

}
void ExperimentDataHandler::setUp(std::string working_folder)
{
	this->working_folder = working_folder;
	time_t t = time(0); //get time now
	struct tm * now = localtime( & t );
	this->experiment_name = "";
	std::ostringstream mon_str,day_str,hour_str,min_str;
	mon_str << now->tm_mon;
	day_str << now->tm_mday;
	hour_str << now->tm_hour;
	min_str << now->tm_min;
	this->experiment_name +=  mon_str.str() + "-" + day_str.str() + "-" + hour_str.str() + "-" + min_str.str();
	
	this->exp_iteration = 0;
	exp_num = 0;

	// create folder of the experiment
	std::string command = "mkdir -p " + working_folder + "/" + experiment_name;
	system(command.c_str());

	// open the file
	std::string file_name = working_folder + "/" + experiment_name + "/data.txt";
	std::cout << "file name: " << file_name << std::endl;
	this->file.open(file_name.c_str());
	if(!this->file.is_open())
	{
		ROS_ERROR("Error creating the file");
		return;
	}

	labels.push_back("n_objects");
	labels.push_back("seg_time");
	labels.push_back("predicates_time");
	labels.push_back("planning_time");
	labels.push_back("IK");
	labels.push_back("on_predicates");
	labels.push_back("block_predicates");
	labels.push_back("block_grasp_predicates");
	labels.push_back("objects_collisions");
	labels.push_back("objects_collisions");
	labels.push_back("ee_collisions");
	labels.push_back("average_objects_collision");
	labels.push_back("average_ee_collision");
	labels.push_back("executed_action");
	labels.push_back("plan_feasible");

}

void ExperimentDataHandler::updateExperiment(std::vector<double>& data,
		               iri_fast_downward_wrapper::Plan& plan,
		                bool feasible,
		                cv_bridge::CvImagePtr image_ptr)
{
	if(this->file.is_open())
	{
		// if it is the first iteration of the experiment write number of experiment and labels
		if(exp_iteration == 0)
		{
			std::ostringstream exp_num_str;
			exp_num_str << exp_num; 
			std::cout << "writing name experiment\n";
			file << "--exp" + exp_num_str.str() + "\n";

			//write plan
			file << "plan: ";
			for (int i = 0; i < plan.actions.size(); ++i)
			{
				file << "(" + plan.actions[i].action_name + " " + plan.actions[i].objects[0] + ") ";
			}
			file << "\n";

			std::cout << "writing labels\n";
			// write labels
			for(uint i = 0; i < labels.size() ; i++)
			{
				file << labels[i] + " ";
			}
			file << "\n";


		}

		//write time and executed action 
		for (int i = 0; i < data.size(); ++i)
		{
			std::ostringstream num_str;
			num_str << data[i];
			file << num_str.str() + " ";
		}
		file << plan.actions[0].action_name;

		std::ostringstream feasible_str;
		feasible_str << feasible;
		file << feasible_str.str() + "\n";
	}
	else
	{
		ROS_ERROR("The experiment file is not open! Maybe the working folder is not a pull path (don't use ~)");
	}

	// std::ostringstream exp_num_str,exp_iteration_str;
	// exp_num_str << exp_num; 
	// exp_iteration_str << exp_iteration;
	// std::string image_path = working_folder + "/" + experiment_name + "/exp" + exp_num_str.str() + "_ac" + exp_iteration_str.str() + ".png";
	// std::cout << "saving image: " << image_path << std::endl;
	// cv::Mat img(480, 640, CV_8UC3,cv::Scalar(255,255,255));
	// //image = image_ptr->image;
	// image = img;

	// try {
	// 	cv::imwrite(image_path.c_str(), image );
 //    }
 //    catch (runtime_error& ex) {
 //        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
 //        return;
 //    }

	exp_iteration++;

}

void ExperimentDataHandler::closeFile()
{
	file.close();
}
void ExperimentDataHandler::newExperiment()
{
	exp_iteration = 0;
	exp_num++;
}

