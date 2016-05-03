cmake_minimum_required(VERSION 2.8.3)
project(iri_table_clearing_execute)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm actionlib sensor_msgs geometry_msgs iri_common_drivers_msgs 
	message_generation 
	geometry_msgs 
	std_msgs 
	trajectory_msgs 
	control_msgs 
	sensor_msgs
	iri_base_driver
	iri_action_server 
	iri_common_drivers_msgs
	iri_table_clearing_common_msgs
	iri_wam_common_msgs
	iri_action_server
	moveit_core 
 	moveit_ros_planning
 	moveit_ros_planning_interface
	)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)
find_package(iriutils REQUIRED)
# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ExecuteGrasping.srv
  ExecutePushing.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   control_msgs
   iri_table_clearing_common_msgs
)

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/TableClearingExecute.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS iri_base_algorithm actionlib sensor_msgs geometry_msgs iri_common_drivers_msgs
    message_generation
    geometry_msgs std_msgs
    trajectory_msgs 
    control_msgs 
    iri_base_driver 
    iri_action_server
    iri_common_drivers_msgs
	iri_wam_common_msgs
	iri_action_server
	sensor_msgs 
	iri_table_clearing_common_msgs
	moveit_core 
	moveit_ros_planning
	moveit_ros_planning_interface
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
#  DEPENDS 
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/table_clearing_execute_alg.cpp src/table_clearing_execute_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} sensor_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} geometry_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} iri_common_drivers_msgs_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
