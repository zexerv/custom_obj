#include "moveit_custom_objective_plugin/custom_ompl_planner_manager.h"
#include "moveit_custom_objective_plugin/manipulability_objective.h" // Or manipulability_objective.h if renamed

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
#include <vector> // <<< Include vector

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
        // Create internal helper BEFORE trying to set its configs
        default_ompl_interface_ = std::make_shared<ompl_interface::OMPLInterface>(robot_model_, node_handle_);

        // Now pass the configs that MoveIt gave us earlier via setPlannerConfigurations()
        // The OMPLInterface likely uses these to load planner details from params
        if (!planner_configs_.empty()) {
             ROS_INFO("Passing %zu stored planner configurations to internal OMPLInterface.", planner_configs_.size());
             default_ompl_interface_->setPlannerConfigurations(planner_configs_);
        } else {
             ROS_WARN("No planner configurations were passed before initialize. Internal OMPLInterface might lack planner details.");
             // It might still load defaults from param server using node_handle_ though.
        }

        // *** ADDED DIAGNOSTIC ***
        // Immediately check what configurations the internal helper reports
        planning_interface::PlannerConfigurationMap internal_configs = default_ompl_interface_->getPlannerConfigurations();
        ROS_INFO("Internal OMPLInterface reports %zu configurations after initialization.", internal_configs.size());
        for (const auto& pair : internal_configs) {
            ROS_INFO("  - Internal Config: %s (Group: %s)", pair.first.c_str(), pair.second.group.c_str());
        }


    } catch (const std::exception& e) {
        ROS_FATAL("CustomOmplPlannerManager: Failed to create internal OMPLInterface: %s", e.what());
        return false;
    }
    ROS_INFO("CustomOmplPlannerManager: Wrapper initialized successfully.");
    return true;
}

// --- getDescription ---
std::string CustomOmplPlannerManager::getDescription() const { return "Custom OMPL Plugin w/ SimplePenaltyObjective"; } // TODO: Update this string later

// // --- getPlannerConfigurations ---
// void CustomOmplPlannerManager::getPlannerConfigurations(planning_interface::PlannerConfigurationMap& configs) const
// {
//      configs.clear(); // Start with an empty map
//      ROS_INFO_NAMED("CustomOmplPlannerManager", "getPlannerConfigurations called..."); // <<< ADDED
//      if (!default_ompl_interface_) {
//          ROS_ERROR_NAMED("CustomOmplPlannerManager", "getPlannerConfigurations: Internal OMPL Interface not initialized!");
//          return;
//      }

//      // Delegate to the internal default OMPL Interface
//      planning_interface::PlannerConfigurationMap internal_configs = default_ompl_interface_->getPlannerConfigurations();
//      ROS_INFO_NAMED("CustomOmplPlannerManager", "Internal OMPLInterface provided %zu configurations.", internal_configs.size()); // <<< ADDED

//      // Assign to the output parameter
//      configs = internal_configs;

//      ROS_INFO_NAMED("CustomOmplPlannerManager", "Returning %zu configurations.", configs.size()); // <<< ADDED
//      // Log returned configs for debugging
//      // for (const auto& pair : configs) {
//      //     ROS_DEBUG_NAMED("CustomOmplPlannerManager", "  - Returning Config: %s (Group: %s)", pair.first.c_str(), pair.second.group.c_str());
//      // }

// }


// --- getPlannerConfigurations ---
// Use matching parameter name 'planner_configs'

// --- getPlannerConfigurations ---
// Parameter name 'planner_configs' matches header, override removed in header
// void CustomOmplPlannerManager::getPlannerConfigurations(planning_interface::PlannerConfigurationMap& planner_configs) const 
// {
//      planner_configs.clear(); // Start empty
//      ROS_INFO_NAMED("CustomOmplPlannerManager", "getPlannerConfigurations called..."); // <<< CHECK THIS LOG
//      if (!default_ompl_interface_) { 
//          ROS_ERROR_NAMED("CustomOmplPlannerManager", "getPlannerConfigurations: Internal OMPL Interface not initialized!");
//          return; 
//      }

//      // Delegate and assign
//      planning_interface::PlannerConfigurationMap internal_configs = default_ompl_interface_->getPlannerConfigurations();
//      ROS_INFO_NAMED("CustomOmplPlannerManager", "Internal OMPLInterface provided %zu configurations.", internal_configs.size()); // <<< CHECK THIS LOG (X)

//      // Assign to the output parameter
//      planner_configs = internal_configs;

//      ROS_INFO_NAMED("CustomOmplPlannerManager", "Returning %zu configurations.", planner_configs.size()); // <<< CHECK THIS LOG (Y)
// }

void CustomOmplPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
    if (!default_ompl_interface_) {
         ROS_ERROR_NAMED("CustomOmplPlannerManager", "getPlanningAlgorithms: Internal OMPL Interface not initialized!");
         algs.clear();
         return;
     }

     // Get the map of configurations from the internal interface
     const planning_interface::PlannerConfigurationMap& known_configs = default_ompl_interface_->getPlannerConfigurations();
     
     // Extract the names (keys) into the result vector
     algs.clear();
     algs.reserve(known_configs.size());
     for (const auto& config_pair : known_configs) {
         algs.push_back(config_pair.first); // config_pair.first is the name (e.g., "RRTstar[ur5e]")
     }
     ROS_INFO_NAMED("CustomOmplPlannerManager", "getPlanningAlgorithms: Reporting %zu algorithms.", algs.size());
}
// void CustomOmplPlannerManager::getPlannerConfigurations(planning_interface::PlannerConfigurationMap& planner_configs) const 
// {
//      planner_configs.clear(); // Start empty
//      ROS_INFO_NAMED("CustomOmplPlannerManager", "getPlannerConfigurations called..."); 
//      if (!default_ompl_interface_) { 
//          ROS_ERROR_NAMED("CustomOmplPlannerManager", "getPlannerConfigurations: Internal OMPL Interface not initialized!");
//          return; 
//      }

//      // Delegate and assign
//      planning_interface::PlannerConfigurationMap internal_configs = default_ompl_interface_->getPlannerConfigurations();
//      ROS_INFO_NAMED("CustomOmplPlannerManager", "Internal OMPLInterface provided %zu configurations.", internal_configs.size()); 

//      // Assign to the output parameter (now named planner_configs)
//      planner_configs = internal_configs;

//      ROS_INFO_NAMED("CustomOmplPlannerManager", "Returning %zu configurations.", planner_configs.size()); 
// }
// --- setPlannerConfigurations ---
void CustomOmplPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) {
     planner_configs_ = pconfig; // Store configs passed by MoveIt *before* initialize
     ROS_INFO_NAMED("CustomOmplPlannerManager", "setPlannerConfigurations called with %zu configurations.", pconfig.size()); // <<< Changed to INFO
     // Don't immediately pass to default_ompl_interface_ here, as it might not exist yet.
     // Pass them inside initialize() instead.
}

// --- getPlanningContext (Keep implemented version) ---
planning_interface::PlanningContextPtr CustomOmplPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    // ... (Keep the implementation from the last step that compiled) ...
    // ... (It correctly delegates, casts, gets SimpleSetup, sets objective) ...
    
    // --- Start of copy from previous working version ---
    std::string config_lookup_key = req.planner_id + "[" + req.group_name + "]";
    ROS_INFO_NAMED("CustomOmplPlannerManager", "Wrapper getPlanningContext called for group '%s', planner '%s' (config key: %s)",
                 req.group_name.c_str(), req.planner_id.c_str(), config_lookup_key.c_str());

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
        if (!simple_setup) { /* Error handling */ ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get SimpleSetup"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }
        ompl::base::ProblemDefinitionPtr problem_def = simple_setup->getProblemDefinition();
        if (!problem_def) { /* Error handling */ ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get ProblemDefinition"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }
        ompl::base::SpaceInformationPtr space_info = simple_setup->getSpaceInformation();
        if (!space_info) { /* Error handling */ ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get SpaceInformation"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }

        // Get JMG needed for objective constructor
        const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(req.group_name);
        if (!jmg) { /* Error handling */ ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get JMG for objective"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }

        // Create objective, passing space_info, robot_model, and jmg
        // ompl::base::OptimizationObjectivePtr my_objective(new ManipulabilityObjective(space_info, robot_model_, jmg)); // Or ManipulabilityObjective if renamed
        ompl::base::OptimizationObjectivePtr my_objective(new ManipulabilityMotionObjective(space_info, robot_model_, jmg)); 
        
        problem_def->setOptimizationObjective(my_objective);
        ROS_INFO_NAMED("CustomOmplPlannerManager", "Successfully set custom objective '%s'.", my_objective->getDescription().c_str());

        // Don't re-configure here, base context should be configured
        // ompl_context->configure(node_handle_, false); 

    } catch (const std::exception& e) {
         ROS_ERROR_NAMED("CustomOmplPlannerManager", "Exception setting custom objective: %s", e.what());
         error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
         return base_context_ptr;
    }
    // --- End Custom Objective ---
    return base_context_ptr; // Return the modified base context
    // --- End of copy ---
}

// --- canServiceRequest (Keep corrected version) ---
 bool CustomOmplPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
 {
     if (!default_ompl_interface_) return false;
     // Delegate to internal interface's getPlannerConfigurations to check
     const planning_interface::PlannerConfigurationMap& known_configs = default_ompl_interface_->getPlannerConfigurations();
     std::string config_lookup_key = req.planner_id + "[" + req.group_name + "]";
     bool known_config = known_configs.count(config_lookup_key);
     if(!known_config) { known_config = known_configs.count(req.planner_id); } // Fallback check
     return known_config;
 }

} // END namespace moveit_custom_objective_plugin

// --- PLUGINLIB REGISTRATION ---
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_custom_objective_plugin::CustomOmplPlannerManager, planning_interface::PlannerManager)