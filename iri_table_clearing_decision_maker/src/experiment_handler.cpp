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
	labels.push_back("filtering_time[ms]");
	labels.push_back("seg_time[ms]");
	labels.push_back("predicates_time[ms]");
	labels.push_back("planning_time[ms]");
	labels.push_back("IK[ms]");
	labels.push_back("on_predicates[ms]");
	labels.push_back("block_predicates[ms]");
	labels.push_back("block_grasp_predicates[ms]");
	labels.push_back("objects_collisions[ms]");
	labels.push_back("ee_collisions[ms]");
	labels.push_back("average_objects_collision[ms]");
	labels.push_back("average_ee_collision[ms]");
	labels.push_back("action_execution_time[ms]");
	labels.push_back("time_from_start[ms]");
	labels.push_back("action");
	labels.push_back("ik_feasible");
	labels.push_back("time_to_save_pcl");


}

void ExperimentDataHandler::updateExperiment(std::vector<double>& data,
		               iri_fast_downward_wrapper::Plan& plan,
		               bool ik_feasible, sensor_msgs::PointCloud2* cloud_msg,bool save_pcl)
{
	if(this->file.is_open())
	{
		// if it is the first iteration of the experiment write number of experiment and labels
		if(exp_iteration == 0)
		{
			std::ostringstream exp_num_str;
			exp_num_str << exp_num; 
			file << "--exp" + exp_num_str.str() + "\n";

			// ask for user for comments about the experiment
			//std::cout << "\nNew experiment, please write some comments about this experiment:";
			//std::string comments;
			// little hack - putting twice it works as expected
			//getline(std::cin, comments); 
			//getline(std::cin, comments); 
    		//file << "comments: " + comments + "\n";

			//write plan
			file << "plan: ";
			for (int i = 0; i < plan.actions.size(); ++i)
			{
				file << "(" + plan.actions[i].action_name + " " + plan.actions[i].objects[0] + ") ";
			}
			file << "\n";

			// write labels
			file << "labels: ";			
			for(uint i = 0; i < labels.size() ; i++)
			{
				file << labels[i] + " ";
			}
			file << "\n";


		}

		//write time and executed action 
		file << "data: ";
		for (int i = 0; i < data.size(); ++i)
		{
			std::ostringstream num_str;
			num_str << data[i];
			file << num_str.str() + " ";
		}

		// write plan action
		if(plan.actions.size() > 0)
			file << plan.actions[0].action_name + "-" + plan.actions[0].objects[0] + " ";
		else 
			file << "no_plan ";

		std::ostringstream ik_feasible_str;
		ik_feasible_str << ik_feasible;
		file << ik_feasible_str.str() + " ";

		//save point cloud
		if(save_pcl)
		{
			int time_init = util::GetTimeMs64();
			pcl::PointCloud<pcl::PointXYZRGBA>cloud;
			pcl::fromROSMsg(*cloud_msg,cloud);
			std::ostringstream pcd_name;
			pcd_name << exp_num;
			std::string pcd_name_str = working_folder + "/" + experiment_name + "/cloud_";
			pcd_name_str += pcd_name.str();
			pcd_name << exp_iteration;
		    pcd_name_str += "_" + pcd_name.str() + ".pcd";
			pcl::io::savePCDFileASCII (pcd_name_str.c_str(), cloud);
			int time_to_save_pcl = util::GetTimeMs64() - time_init;
			std::ostringstream save_pcl_time_str;
			save_pcl_time_str << time_to_save_pcl;
			file << save_pcl_time_str.str() + " ";
		}

		file << "\n";

		//write all the plan
		for (uint i = 0; i < plan.actions.size(); ++i)
		{
			file << "(" + plan.actions[i].action_name + " " + plan.actions[i].objects[0] + ") ";
		}
		file << "\n";		
		
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
	//update only if it has been done something dyring this experiment
	if(exp_iteration == 0)
		return;
	exp_iteration = 0;
	exp_num++;
}

void ExperimentDataHandler::writeUnfeasiblePlan()
{
	file << "NO_FEASIBLE\n";
}
void ExperimentDataHandler::writeExperimentInterrupeted()
{
	if(exp_iteration != 0)
		file << "EXP_INTERRUPTED\n";
}
void ExperimentDataHandler::savePredicates(std::vector<iri_table_clearing_predicates::BlockPredicate> blocks_predicates,
			std::vector<iri_table_clearing_predicates::OnTopPredicate> on_top_predicates,
			std::vector<iri_table_clearing_predicates::BlockGraspPredicate> block_grasp_predicates)
{
	std::ofstream predicates_file;
	std::string predicates_file_name =  working_folder + "/" + experiment_name + "/predicates_frame";
	std::ostringstream predicates_file_str;
	predicates_file_str << exp_iteration;
	predicates_file_name += predicates_file_str.str();
	predicates_file.open(predicates_file_name.c_str());
	//write block predicates
	for(uint i = 0; i < blocks_predicates.size(); i++)
	{
		std::ostringstream object_;
		object_ << i;
		std::string object_name = "o" + object_.str();
		//for each direction
		for (uint o = 0; o < blocks_predicates[i].dir1.size(); ++o)
		{
			std::ostringstream object_2;
			object_2 << blocks_predicates[i].dir1[o];
			std::string object_name2 = "o" + object_2.str();
			predicates_file << "block_dir1 " + object_name2 + " " + object_name + "\n";
		}
		for (uint o = 0; o < blocks_predicates[i].dir2.size(); ++o)
		{
			std::ostringstream object_2;
			object_2 << blocks_predicates[i].dir2[o];
			std::string object_name2 = "o" + object_2.str();
			predicates_file << "block_dir2 " + object_name2 + " " + object_name + "\n";
		}
		for (uint o = 0; o < blocks_predicates[i].dir3.size(); ++o)
		{
			std::ostringstream object_2;
			object_2 << blocks_predicates[i].dir3[o];
			std::string object_name2 = "o" + object_2.str();
			predicates_file << "block_dir3 " + object_name2 + " " + object_name + "\n";
		}
		for (uint o = 0; o < blocks_predicates[i].dir4.size(); ++o)
		{
			std::ostringstream object_2;
			object_2 << blocks_predicates[i].dir4[o];
			std::string object_name2 = "o" + object_2.str();
			predicates_file << "block_dir4 " + object_name2 + " " + object_name + "\n";
		}
	}
	//write block grasp predicates
	for(uint i = 0; i < block_grasp_predicates.size(); i++)
	{
		std::ostringstream object_;
		object_ << i;
		std::string object_name = "o" + object_.str();
		for (uint o = 0; o < block_grasp_predicates[i].object.size(); ++o)
		{
			std::ostringstream object_2;
			object_2 <<  block_grasp_predicates[i].object[o];
			std::string object_name2 = "o" + object_2.str();
			predicates_file << "block_grasp " + object_name2 + " " + object_name + "\n";
		}
	}
	//write on predicates
	for(uint i = 0; i < on_top_predicates.size(); i++)
	{
		std::ostringstream object_;
		object_ << i;
		std::string object_name = "o" + object_.str();
		for (uint o = 0; o < on_top_predicates[i].object.size(); ++o)
		{
			std::ostringstream object_2;
			object_2 << on_top_predicates[i].object[o];
			std::string object_name2 = "o" + object_2.str();
			predicates_file << "on " + object_name2 + " " + object_name + "\n";
		}
	}
	predicates_file.close();
}