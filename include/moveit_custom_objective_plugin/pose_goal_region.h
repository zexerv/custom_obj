#ifndef MOVEITSTUFF_POSE_GOAL_REGION_H
#define MOVEITSTUFF_POSE_GOAL_REGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Exception.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h> // <<< CORRECT
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h> // For pose conversions
#include <Eigen/Geometry>
#include <ros/console.h> // For ROS_WARN, ROS_INFO etc.
#include <string>
#include <memory> // For std::shared_ptr

namespace custom_ompl_plugin
{

class PoseGoalRegion : public ompl::base::GoalSampleableRegion
{
public:
    /**
     * @brief Constructor
     * @param si SpaceInformationPtr for the state space
     * @param scene ConstPtr to the planning scene for collision checking and FK
     * @param jmg Pointer to the JointModelGroup for which the pose goal is defined
     * @param target_pose The target pose for the end-effector link
     * @param eef_link_name The name of the end-effector link
     * @param pos_tolerance Position tolerance (meters)
     * @param ori_tolerance Orientation tolerance (radians)
     */
    PoseGoalRegion(const ompl::base::SpaceInformationPtr &si,
                   const planning_scene::PlanningSceneConstPtr& scene,
                   const moveit::core::JointModelGroup* jmg,
                   const geometry_msgs::PoseStamped& target_pose,
                   const std::string& eef_link_name,
                   double pos_tolerance,
                   double ori_tolerance);

    // --- OMPL GoalSampleableRegion Interface ---

    /**
     * @brief Check if a state satisfies the goal condition (pose within tolerance and collision-free).
     * This is the MOST important method.
     */
    bool isSatisfied(const ompl::base::State *st) const override;

    /**
     * @brief Check if a state satisfies the goal condition, also returning the distance.
     * OMPL often uses this version.
     */
    bool isSatisfied(const ompl::base::State *st, double *distance) const override;

    /**
     * @brief Calculate the distance from a state to the goal region.
     * Used by planners like RRT* to guide the search.
     */
    double distanceGoal(const ompl::base::State *st) const override;

    /**
     * @brief Generate a sample state within the goal region.
     * Optional but helpful for biasing the search. Can be hard to implement efficiently.
     */
    void sampleGoal(ompl::base::State *st) const override;

    /**
     * @brief Return the maximum number of samples we are willing to generate in sampleGoal().
     */
    unsigned int maxSampleCount() const override;

private:
    // Helper to get RobotState from OMPL state
    void stateOMPLToRobotState(const ompl::base::State *st, moveit::core::RobotState& robot_state) const;

    // --- Member Variables ---
    planning_scene::PlanningSceneConstPtr planning_scene_;
    const moveit::core::JointModelGroup* jmg_; // Joint group associated with the goal EEF
    // const moveit_ompl::MoveItStateSpace* moveit_state_space_; // Casted state space pointer
    const ompl_interface::ModelBasedStateSpace* moveit_state_space_; // <<< CORRECT
    geometry_msgs::PoseStamped target_pose_stamped_; // The goal pose (in planning frame)
    std::string eef_link_name_;       // Name of the link whose pose is checked
    double pos_tolerance_sq_;          // Squared position tolerance for efficiency
    double ori_tolerance_;             // Orientation tolerance (angle in radians)

    Eigen::Isometry3d target_pose_eigen_; // Target pose converted to Eigen
};

typedef std::shared_ptr<PoseGoalRegion> PoseGoalRegionPtr;

} // namespace custom_ompl_plugin

#endif // MOVEITSTUFF_POSE_GOAL_REGION_H