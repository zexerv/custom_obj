#ifndef CUSTOM_OMPL_PLANNER_MANAGER_H
#define CUSTOM_OMPL_PLANNER_MANAGER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <string>
#include <vector> // <<< Include vector
#include <map>
#include <memory>
#include <ros/ros.h>

namespace moveit_custom_objective_plugin
{

class CustomOmplPlannerManager : public planning_interface::PlannerManager
{
public:
    CustomOmplPlannerManager();
    ~CustomOmplPlannerManager() override;

    bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;
    std::string getDescription() const override;

    // --- REMOVED getPlannerConfigurations ---
    // void getPlannerConfigurations(planning_interface::PlannerConfigurationMap& planner_configs) const; 

    // --- ADDED getPlanningAlgorithms ---
    void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

    planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                              const planning_interface::MotionPlanRequest& req,
                                                              moveit_msgs::MoveItErrorCodes& error_code) const override;

    bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

    void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) override;

private:
    ros::NodeHandle node_handle_;
    robot_model::RobotModelConstPtr robot_model_;
    std::shared_ptr<ompl_interface::OMPLInterface> default_ompl_interface_;
    planning_interface::PlannerConfigurationMap planner_configs_;
};

} // namespace moveit_custom_objective_plugin

#endif // CUSTOM_OMPL_PLANNER_MANAGER_H