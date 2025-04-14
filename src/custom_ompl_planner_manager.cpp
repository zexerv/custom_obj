#include "moveit_custom_objective_plugin/custom_ompl_planner_manager.h"
#include "moveit_custom_objective_plugin/manipulability_objective.h"
#include "moveit_custom_objective_plugin/pose_goal_region.h"

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_list_macros.h>
#include <map>
#include <string>
#include <memory>
#include <vector>
#include <stdexcept>
#include <ros/ros.h>

namespace moveit_custom_objective_plugin
{

CustomOmplPlannerManager::CustomOmplPlannerManager() : planning_interface::PlannerManager() {}

CustomOmplPlannerManager::~CustomOmplPlannerManager() {}

bool CustomOmplPlannerManager::initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
{
    robot_model_ = model;
    node_handle_ = ros::NodeHandle(ns);
    ROS_INFO("Initializing CustomOmplPlannerManager wrapper in namespace '%s'...", node_handle_.getNamespace().c_str());
    try {
        default_ompl_interface_ = std::make_shared<ompl_interface::OMPLInterface>(robot_model_, node_handle_);
        if (!planner_configs_.empty()) {
             ROS_INFO("Passing %zu stored planner configurations to internal OMPLInterface.", planner_configs_.size());
             default_ompl_interface_->setPlannerConfigurations(planner_configs_);
        } else {
             ROS_WARN("No planner configurations were passed before initialize.");
        }
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

std::string CustomOmplPlannerManager::getDescription() const { return "Custom OMPL Plugin w/ Manipulability Objective and PoseGoalRegion"; }

void CustomOmplPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
    if (!default_ompl_interface_) {
         ROS_ERROR_NAMED("CustomOmplPlannerManager", "getPlanningAlgorithms: Internal OMPL Interface not initialized!");
         algs.clear();
         return;
     }
     const planning_interface::PlannerConfigurationMap& known_configs = default_ompl_interface_->getPlannerConfigurations();
     algs.clear();
     algs.reserve(known_configs.size());
     for (const auto& config_pair : known_configs) {
         algs.push_back(config_pair.first);
     }
     ROS_INFO_NAMED("CustomOmplPlannerManager", "getPlanningAlgorithms: Reporting %zu algorithms.", algs.size());
}

void CustomOmplPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) {
     planner_configs_ = pconfig;
     ROS_INFO_NAMED("CustomOmplPlannerManager", "setPlannerConfigurations called with %zu configurations.", pconfig.size());
}

 bool CustomOmplPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
 {
     if (!default_ompl_interface_) return false;
     const planning_interface::PlannerConfigurationMap& known_configs = default_ompl_interface_->getPlannerConfigurations();
     std::string config_lookup_key = req.planner_id + "[" + req.group_name + "]";
     bool known_config = known_configs.count(config_lookup_key);
     if(!known_config) { known_config = known_configs.count(req.planner_id); }
     return known_config;
 }

planning_interface::PlanningContextPtr CustomOmplPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    std::string config_lookup_key = req.planner_id + "[" + req.group_name + "]";
    ROS_INFO_NAMED("CustomOmplPlannerManager", "Wrapper getPlanningContext called for group '%s', planner '%s' (config key: %s)",
                 req.group_name.c_str(), req.planner_id.c_str(), config_lookup_key.c_str());

    if (!default_ompl_interface_) {
        ROS_ERROR_NAMED("CustomOmplPlannerManager", "getPlanningContext: Internal OMPL Interface not initialized!");
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return nullptr;
    }

    planning_interface::PlanningContextPtr base_context_ptr =
        default_ompl_interface_->getPlanningContext(planning_scene, req, error_code);

    if (!base_context_ptr || error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
         ROS_ERROR_NAMED("CustomOmplPlannerManager", "Internal OMPL Interface failed to provide context. Error code: %d", error_code.val);
         return base_context_ptr;
    }

    ROS_INFO_NAMED("CustomOmplPlannerManager", "Internal OMPL Interface provided base context. Proceeding with modifications.");

    ompl_interface::ModelBasedPlanningContextPtr ompl_context =
        std::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(base_context_ptr);

    if (!ompl_context) {
        ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed to cast base context to ModelBasedPlanningContext!");
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return base_context_ptr;
    }

    ompl::geometric::SimpleSetupPtr simple_setup = ompl_context->getOMPLSimpleSetup();
    if (!simple_setup) { ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get SimpleSetup from OMPL context"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }
    ompl::base::ProblemDefinitionPtr problem_def = simple_setup->getProblemDefinition();
    if (!problem_def) { ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get ProblemDefinition from SimpleSetup"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }
    ompl::base::SpaceInformationPtr space_info = simple_setup->getSpaceInformation();
    if (!space_info) { ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get SpaceInformation from SimpleSetup"); error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr; }

    try {
        const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(req.group_name);
        if (!jmg) {
            ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get JMG '%s' for objective.", req.group_name.c_str());
            error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; return base_context_ptr;
        }
        ompl::base::OptimizationObjectivePtr obj(new ManipulabilityMotionObjective(space_info, robot_model_, jmg));
        problem_def->setOptimizationObjective(obj);
        ROS_INFO_NAMED("CustomOmplPlannerManager", "Successfully set custom objective '%s'.", obj->getDescription().c_str());
    } catch (const std::exception& e) {
        ROS_ERROR_NAMED("CustomOmplPlannerManager", "Exception setting custom objective: %s", e.what());
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return base_context_ptr;
    }

    bool pose_goal_set = false;
    if (!req.goal_constraints.empty() &&
        !req.goal_constraints[0].position_constraints.empty() &&
        !req.goal_constraints[0].orientation_constraints.empty())
    {
        ROS_DEBUG_NAMED("CustomOmplPlannerManager", "Goal constraints detected. Checking for EEF pose goal.");

        const moveit_msgs::PositionConstraint& pc = req.goal_constraints[0].position_constraints[0];
        const moveit_msgs::OrientationConstraint& oc = req.goal_constraints[0].orientation_constraints[0];

        if (pc.link_name == oc.link_name && !pc.link_name.empty() &&
            !pc.constraint_region.primitive_poses.empty() &&
            (oc.orientation.x != 0 || oc.orientation.y != 0 || oc.orientation.z != 0 || oc.orientation.w != 0)
            )
        {
            std::string eef_link_name = pc.link_name;
            geometry_msgs::PoseStamped target_pose;

            target_pose.header = pc.header;
            target_pose.pose = pc.constraint_region.primitive_poses[0];
            target_pose.pose.orientation = oc.orientation;

            double pos_tolerance = 0.01;
            double ori_tolerance = 0.02;
            const std::string& planner_config_name_for_log = req.planner_id;

            try {
                const std::map<std::string, std::string>& config_map = ompl_context->getSpecification().config_;

                auto pos_it = config_map.find("goal_position_tolerance");
                if (pos_it != config_map.end()) {
                    pos_tolerance = std::stod(pos_it->second);
                    ROS_DEBUG_NAMED("CustomOmplPlannerManager", "Found 'goal_position_tolerance': %f in config for '%s'", pos_tolerance, planner_config_name_for_log.c_str());
                } else {
                    ROS_WARN_NAMED("CustomOmplPlannerManager", "Key 'goal_position_tolerance' not found in planner config for '%s'. Using default: %.4f", planner_config_name_for_log.c_str(), pos_tolerance);
                }

                auto ori_it = config_map.find("goal_orientation_tolerance");
                if (ori_it != config_map.end()) {
                    ori_tolerance = std::stod(ori_it->second);
                     ROS_DEBUG_NAMED("CustomOmplPlannerManager", "Found 'goal_orientation_tolerance': %f in config for '%s'", ori_tolerance, planner_config_name_for_log.c_str());
                } else {
                     ROS_WARN_NAMED("CustomOmplPlannerManager", "Key 'goal_orientation_tolerance' not found in planner config for '%s'. Using default: %.4f", planner_config_name_for_log.c_str(), ori_tolerance);
                }
            } catch (const std::invalid_argument& e) {
                ROS_ERROR_NAMED("CustomOmplPlannerManager", "Invalid argument converting tolerance string from config for '%s' to double: %s. Using defaults.", planner_config_name_for_log.c_str(), e.what());
                pos_tolerance = 0.01; ori_tolerance = 0.02;
            } catch (const std::out_of_range& e) {
                ROS_ERROR_NAMED("CustomOmplPlannerManager", "Out of range converting tolerance string from config for '%s' to double: %s. Using defaults.", planner_config_name_for_log.c_str(), e.what());
                pos_tolerance = 0.01; ori_tolerance = 0.02;
            }

            ROS_INFO_NAMED("CustomOmplPlannerManager", "Detected Pose Goal for link '%s' in frame '%s'. Overriding OMPL goal.", eef_link_name.c_str(), target_pose.header.frame_id.c_str());
            ROS_INFO_NAMED("CustomOmplPlannerManager", " Target Pose: Pos(%.3f, %.3f, %.3f), Ori(%.3f, %.3f, %.3f, %.3f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
            ROS_INFO_NAMED("CustomOmplPlannerManager", " Using Tolerances (from config '%s' or defaults): Pos=%.4f, Ori=%.4f", planner_config_name_for_log.c_str(), pos_tolerance, ori_tolerance);

            const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(req.group_name);
            if (!jmg) {
                ROS_ERROR_NAMED("CustomOmplPlannerManager", "Failed get JMG '%s' for PoseGoalRegion.", req.group_name.c_str());
                error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                return base_context_ptr;
            }

            try {
                auto pose_goal = std::make_shared<custom_ompl_plugin::PoseGoalRegion>(
                    space_info, planning_scene, jmg, target_pose, eef_link_name, pos_tolerance, ori_tolerance);

                problem_def->clearGoal();
                problem_def->setGoal(pose_goal);
                pose_goal_set = true;
                ROS_INFO_NAMED("CustomOmplPlannerManager", "Successfully set PoseGoalRegion as the OMPL goal.");

            } catch (const std::exception& e) {
                 ROS_ERROR_NAMED("CustomOmplPlannerManager", "Exception creating/setting PoseGoalRegion: %s", e.what());
                 error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                 return base_context_ptr;
            }
        } else {
             ROS_DEBUG_NAMED("CustomOmplPlannerManager", "Constraints present, but not a matched position/orientation pair or data invalid. Using default goal.");
        }
    } else {
         ROS_DEBUG_NAMED("CustomOmplPlannerManager", "No goal constraints found or not matching pose structure. Using default OMPL goal.");
    }

    if (!pose_goal_set) {
        ROS_INFO_NAMED("CustomOmplPlannerManager", "No pose goal override applied. Proceeding with default goal settings from OMPLInterface.");
    }

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return base_context_ptr;
}


} // END namespace moveit_custom_objective_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_custom_objective_plugin::CustomOmplPlannerManager, planning_interface::PlannerManager)