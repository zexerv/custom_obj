#include "moveit_custom_objective_plugin/custom_ompl_planner_manager.h"
#include "moveit_custom_objective_plugin/simple_penalty_objective.h"

// --- Necessary Includes ---
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_list_macros.h>
#include <map>
#include <string>
#include <memory>
#include <ros/ros.h>

namespace moveit_custom_objective_plugin
{

// --- Constructor ---
CustomOmplPlannerManager::CustomOmplPlannerManager() : planning_interface::PlannerManager() {}

// --- Destructor ---
CustomOmplPlannerManager::~CustomOmplPlannerManager() {}

// --- Initialize ---
bool CustomOmplPlannerManager::initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
{
    robot_model_ = model;
    node_handle_ = ros::NodeHandle(ns);
    ROS_INFO("Initializing CustomOmplPlannerManager wrapper in namespace '%s'...", node_handle_.getNamespace().c_str());
    try {
        default_ompl_interface_ = std::make_shared<ompl_interface::OMPLInterface>(robot_model_, node_handle_);
        default_ompl_interface_->setPlannerConfigurations(planner_configs_);
    } catch (const std::exception& e) {
        ROS_FATAL("CustomOmplPlannerManager: Failed to create internal OMPLInterface: %s", e.what());
        return false;
    }
    ROS_INFO("CustomOmplPlannerManager: Wrapper initialized successfully.");
    return true;
}

// --- getDescription ---
std::string CustomOmplPlannerManager::getDescription() const { return "Custom OMPL Plugin w/ SimplePenaltyObjective"; }

// --- getPlannerConfigurations ---
void CustomOmplPlannerManager::getPlannerConfigurations(planning_interface::PlannerConfigurationMap& configs) const
{
     if (!default_ompl_interface_) { ROS_ERROR_NAMED("CustomOmplPlannerManager", "PCM not init!"); configs.clear(); return; }
     // *** Fix 2: Correctly call 0-arg method and assign ***
     configs = default_ompl_interface_->getPlannerConfigurations();
}

// --- setPlannerConfigurations ---
void CustomOmplPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) {
     planner_configs_ = pconfig;
     if (default_ompl_interface_) { default_ompl_interface_->setPlannerConfigurations(planner_configs_); }
}

// --- getPlanningContext ---
planning_interface::PlanningContextPtr CustomOmplPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    ROS_INFO_NAMED("CustomOmplPlannerManager", "Wrapper getPlanningContext called for group '%s', planner '%s'", req.group_name.c_str(), req.planner_id.c_str());
    if (!default_ompl_interface_) { /* Error handling */ error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return nullptr; }

    // 1. Delegate to the internal default OMPL Interface
    planning_interface::PlanningContextPtr base_context_ptr =
        default_ompl_interface_->getPlanningContext(planning_scene, req, error_code);

    // 2. Check if the default interface succeeded
    if (!base_context_ptr || error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) { /* Error handling */ return base_context_ptr; }

    ROS_INFO_NAMED("CustomOmplPlannerManager", "Internal OMPL Interface provided context. Setting custom objective.");

    // 3. Cast to the specific OMPL context type
    ompl_interface::ModelBasedPlanningContextPtr ompl_context =
        std::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(base_context_ptr);
    if (!ompl_context) { /* Error handling */ error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }

    // --- Set Custom Objective ---
    try {
        ompl::geometric::SimpleSetupPtr simple_setup = ompl_context->getOMPLSimpleSetup();
        if (!simple_setup) { /* Error handling */ error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }
        ompl::base::ProblemDefinitionPtr problem_def = simple_setup->getProblemDefinition();
        if (!problem_def) { /* Error handling */ error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }
        ompl::base::SpaceInformationPtr space_info = simple_setup->getSpaceInformation();
        if (!space_info) { /* Error handling */ error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }

        ompl::base::OptimizationObjectivePtr my_objective(new SimplePenaltyObjective(space_info));
        problem_def->setOptimizationObjective(my_objective);
        ROS_INFO_NAMED("CustomOmplPlannerManager", "Successfully set custom objective '%s'.", my_objective->getDescription().c_str());
    } catch (const std::exception& e) {
         ROS_ERROR_NAMED("CustomOmplPlannerManager", "Exception setting custom objective: %s", e.what());
         error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
         return base_context_ptr;
    }
    // --- End Custom Objective ---

    return base_context_ptr;
}

// --- canServiceRequest ---
 bool CustomOmplPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
 {
     // *** Fix 3: Implement check directly using stored configs ***
     // (Since default_ompl_interface_ doesn't have this method publicly)
     std::string config_lookup_key = req.planner_id + "[" + req.group_name + "]";
     bool known_config = planner_configs_.count(config_lookup_key);
     if(!known_config) {
         known_config = planner_configs_.count(req.planner_id); // Fallback check
     }
     ROS_DEBUG_STREAM_NAMED("CustomOmplPlannerManager", "canServiceRequest check for '" << config_lookup_key << "' or '" << req.planner_id << "': " << (known_config ? "Yes" : "No"));
     return known_config;
 }

} // END namespace moveit_custom_objective_plugin

// --- PLUGINLIB REGISTRATION ---
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_custom_objective_plugin::CustomOmplPlannerManager, planning_interface::PlannerManager)