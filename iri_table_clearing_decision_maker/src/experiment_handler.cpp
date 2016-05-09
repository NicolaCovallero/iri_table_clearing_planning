#include "experiment_handler.h"

using namespace std;

ExperimentDataHandler::ExperimentDataHandler(std::string working_folder)
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
}

ExperimentDataHandler::~ExperimentDataHandler()
{

}
void ExperimentDataHandler::updateExperimentNumber()
{
	this->exp_num++;
	this->exp_iteration = 0;
}



