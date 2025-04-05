#ifndef MANIPULABILITY_MOTION_OBJECTIVE_H
#define MANIPULABILITY_MOTION_OBJECTIVE_H

// Base OMPL class
#include <ompl/base/OptimizationObjective.h>
// OMPL helpers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>

// MoveIt Core for Kinematics & State representation
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
// MoveIt OMPL Interface helpers (needed for state conversion)
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

// Eigen for matrix math (Jacobian)
#include <eigen3/Eigen/Dense>

// ROS Logging
#include <ros/console.h>

// Standard C++ Libraries
#include <limits>
#include <vector>    // For getting joint values
#include <sstream>   // For formatting log messages
#include <iomanip>   // For std::setprecision

namespace moveit_custom_objective_plugin
{

// --- Constants ---
const double NEAR_SINGULARITY_THRESHOLD_MM = 1e-4;
const double HIGH_PENALTY_VALUE_MM = 1e6;
const std::string LOGNAME_MM = "ManipMotionObjective"; // Logger name for this class
const double DEFAULT_PATH_LENGTH_WEIGHT = 1.0;
const double DEFAULT_MANIPULABILITY_WEIGHT = 0.01; // Use the tuned weight
const double DEFAULT_EPSILON = 1e-6;

// --- Objective Class ---
// Inherits directly from OptimizationObjective
class ManipulabilityMotionObjective : public ompl::base::OptimizationObjective
{
public:
    ManipulabilityMotionObjective(const ompl::base::SpaceInformationPtr &si,
                                  const moveit::core::RobotModelConstPtr& robot_model,
                                  const moveit::core::JointModelGroup* jmg)
      : ompl::base::OptimizationObjective(si), // Call base constructor
        robot_model_(robot_model),
        jmg_(jmg),
        manipulability_weight_(DEFAULT_MANIPULABILITY_WEIGHT), // Use constant
        path_length_weight_(DEFAULT_PATH_LENGTH_WEIGHT),     // Use constant
        epsilon_(DEFAULT_EPSILON)                           // Use constant
    {
        description_ = "PathLength + Weighted Inverse Manipulability (in MotionCost, Logged)";
        if (!robot_model_) { ROS_ERROR_NAMED(LOGNAME_MM, "Received null RobotModel pointer!"); }
        if (!jmg_) { ROS_ERROR_NAMED(LOGNAME_MM, "Received null JointModelGroup pointer!"); }
        ROS_INFO_NAMED(LOGNAME_MM, "Created for group '%s'. PathWeight: %.3f, ManipWeight: %.3f",
                       jmg_ ? jmg_->getName().c_str() : "NULL", path_length_weight_, manipulability_weight_);

        // Tell OMPL we are minimizing cost
        setCostThreshold(infiniteCost());
    }

    // stateCost is trivial, just return 0 (we put logic in motionCost)
    ompl::base::Cost stateCost(const ompl::base::State *s) const override
    {
        // ROS_ERROR_NAMED(LOGNAME_MM,"stateCost called - THIS SHOULD NOT HAPPEN OFTEN"); // Keep commented unless debugging virtual dispatch
        return ompl::base::Cost(0.0);
    }

    // motionCost includes path length AND manipulability cost of destination state s2
    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        // 1. Calculate Path Length Cost
        double path_length_cost = si_->distance(s1, s2);

        // 2. Calculate Manipulability Cost for the destination state s2
        double manip_cost_s2 = HIGH_PENALTY_VALUE_MM; // Default high penalty
        double measure_s2 = -1.0;                     // Default invalid measure

        if (!jmg_ || !robot_model_ || !si_) {
             ROS_ERROR_THROTTLE_NAMED(1.0, LOGNAME_MM, "motionCost: JMG, RobotModel or SI is null!");
             return ompl::base::Cost(path_length_weight_ * path_length_cost + manip_cost_s2); // Return high penalty cost
        }

        try {
            // Get and check state space
            const ompl::base::StateSpace *space = si_->getStateSpace().get();
            const ompl_interface::ModelBasedStateSpace *model_space = dynamic_cast<const ompl_interface::ModelBasedStateSpace*>(space);
            if (!model_space) { throw std::runtime_error("Failed cast to ModelBasedStateSpace"); }

            // Create temp state and copy OMPL state s2 into it
            moveit::core::RobotState temp_state_s2(robot_model_);
            model_space->copyToRobotState(temp_state_s2, s2);

            // *** Log the Joint Values for s2 ***
            std::vector<double> joint_values_s2;
            temp_state_s2.copyJointGroupPositions(jmg_, joint_values_s2);
            std::stringstream ss_joints;
            ss_joints << "Joints(s2): [";
            for(size_t i=0; i<joint_values_s2.size(); ++i) {
                ss_joints << std::fixed << std::setprecision(3) << joint_values_s2[i] << (i == joint_values_s2.size() - 1 ? "" : ", ");
            }
            ss_joints << "]";
            ROS_DEBUG_NAMED(LOGNAME_MM, "%s", ss_joints.str().c_str());
            // *** End Joint Logging ***

            // Update FK transforms *after* copying joints
            temp_state_s2.update(true);

            // Get Jacobian
            const moveit::core::LinkModel* tip_link = jmg_->getLinkModels().back();
             if (!tip_link) { throw std::runtime_error("Failed to get tip link"); }
            Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian_s2;
            if (!temp_state_s2.getJacobian(jmg_, tip_link, reference_point, jacobian_s2, false))
            {
                 ROS_WARN_THROTTLE_NAMED(1.0, LOGNAME_MM, "motionCost: Failed Jacobian for s2. Assigning high cost.");
                 measure_s2 = 0.0; // Treat as singularity
                 manip_cost_s2 = HIGH_PENALTY_VALUE_MM;
            }
            else
            {
                // Calculate Manipulability Measure
                Eigen::MatrixXd jjt_s2 = jacobian_s2 * jacobian_s2.transpose();
                double determinant_s2 = jjt_s2.determinant();
                measure_s2 = (determinant_s2 > 0.0) ? std::sqrt(determinant_s2) : 0.0;

                // Calculate Cost based on Measure
                if (measure_s2 < NEAR_SINGULARITY_THRESHOLD_MM) { // Near singularity
                    manip_cost_s2 = HIGH_PENALTY_VALUE_MM;
                } else {
                    manip_cost_s2 = manipulability_weight_ / measure_s2; // Inverse measure
                }
            }

            // Log components calculated in this step
            ROS_DEBUG_NAMED(LOGNAME_MM, "motionCost: PathLength=%.4f, Manip(s2)=%.4f -> ManipCost=%.4f",
                           path_length_cost, measure_s2, manip_cost_s2);

        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE_NAMED(1.0, LOGNAME_MM, "Exception in motionCost: %s", e.what());
            manip_cost_s2 = HIGH_PENALTY_VALUE_MM; // Ensure high penalty on exception
        } catch (...) {
            ROS_ERROR_THROTTLE_NAMED(1.0, LOGNAME_MM, "Unknown exception in motionCost");
            manip_cost_s2 = HIGH_PENALTY_VALUE_MM; // Ensure high penalty on exception
        }

        // 3. Return combined weighted cost
        double total_cost = path_length_weight_ * path_length_cost + manip_cost_s2;
        ROS_DEBUG_NAMED(LOGNAME_MM, "motionCost: Total Combined Cost = %.4f", total_cost);
        return ompl::base::Cost(total_cost);

    }

private:
    const moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* jmg_;
    double manipulability_weight_;
    double path_length_weight_;
    double epsilon_;
};

} // namespace moveit_custom_objective_plugin

#endif // MANIPULABILITY_MOTION_OBJECTIVE_H