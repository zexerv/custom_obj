#ifndef MANIPULABILITY_MOTION_OBJECTIVE_H // Renamed header guard
#define MANIPULABILITY_MOTION_OBJECTIVE_H

// Inherit directly from the base OptimizationObjective
#include <ompl/base/OptimizationObjective.h> 
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>

// MoveIt & System Headers
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <eigen3/Eigen/Dense>
#include <ros/console.h>
#include <limits>

namespace moveit_custom_objective_plugin
{

const double NEAR_SINGULARITY_THRESHOLD_MM = 1e-4; // Renamed constant
const double HIGH_PENALTY_VALUE_MM = 1e6;      // Renamed constant
const std::string LOGNAME_MM = "ManipMotionObjective"; // Renamed logger

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
        manipulability_weight_(1.0), // Weight for the state cost component
        path_length_weight_(1.0),   // Weight for the path length component
        epsilon_(1e-6)
    {
        // Set description and specify minimization
        description_ = "PathLength + InvManipulability (in MotionCost)";
        ROS_INFO_NAMED(LOGNAME_MM, "Created for group '%s'. PathWeight: %.2f, ManipWeight: %.2f",
                       jmg_ ? jmg_->getName().c_str() : "NULL", path_length_weight_, manipulability_weight_);
        
        // Tell OMPL we are minimizing cost
        setCostThreshold(infiniteCost()); 
    }

    // stateCost is now trivial, just return 0
    ompl::base::Cost stateCost(const ompl::base::State *s) const override
    {
        // ROS_ERROR_NAMED(LOGNAME_MM, "!!!!!!!!!! stateCost Function WAS CALLED !!!!!!!!!!"); // Keep this if you want to be 100% sure it's ignored
        return ompl::base::Cost(0.0);
    }

    // motionCost now includes path length AND manipulability cost of destination state s2
    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        // 1. Calculate Path Length Cost
        double path_length_cost = si_->distance(s1, s2);

        // 2. Calculate Manipulability Cost for the destination state s2
        double manip_cost_s2 = HIGH_PENALTY_VALUE_MM; // Default high penalty
        double measure_s2 = -1.0;

        if (!jmg_ || !robot_model_ || !si_) {
             ROS_ERROR_THROTTLE_NAMED(1.0, LOGNAME_MM, "motionCost: JMG, RobotModel or SI is null!");
             // Return just path length cost if calculation isn't possible
             return ompl::base::Cost(path_length_weight_ * path_length_cost);
        }

        try {
            const ompl::base::StateSpace *space = si_->getStateSpace().get();
            const ompl_interface::ModelBasedStateSpace *model_space = dynamic_cast<const ompl_interface::ModelBasedStateSpace*>(space);
            if (!model_space) { /* Handle cast error */ throw std::runtime_error("Failed cast to ModelBasedStateSpace"); }

            moveit::core::RobotState temp_state_s2(robot_model_);
            model_space->copyToRobotState(temp_state_s2, s2); // Convert s2
            temp_state_s2.update(true); // Update transforms

            const moveit::core::LinkModel* tip_link = jmg_->getLinkModels().back();
            Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian_s2;
            if (!temp_state_s2.getJacobian(jmg_, tip_link, reference_point, jacobian_s2, false))
            {
                 ROS_WARN_THROTTLE_NAMED(1.0, LOGNAME_MM, "motionCost: Failed Jacobian for s2. Assigning high cost.");
                 measure_s2 = 0.0;
                 manip_cost_s2 = HIGH_PENALTY_VALUE_MM;
            }
            else
            {
                Eigen::MatrixXd jjt_s2 = jacobian_s2 * jacobian_s2.transpose();
                double determinant_s2 = jjt_s2.determinant();
                measure_s2 = (determinant_s2 > 0.0) ? std::sqrt(determinant_s2) : 0.0;

                if (measure_s2 < NEAR_SINGULARITY_THRESHOLD_MM) { // Near singularity
                    manip_cost_s2 = HIGH_PENALTY_VALUE_MM;
                } else {
                    manip_cost_s2 = manipulability_weight_ / measure_s2; // Inverse measure
                }
            }
            // Log components
            ROS_DEBUG_NAMED(LOGNAME_MM, "motionCost: PathLength=%.4f, Manip(s2)=%.4f -> ManipCost=%.4f",
                           path_length_cost, measure_s2, manip_cost_s2);

        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE_NAMED(1.0, LOGNAME_MM, "Exception in motionCost: %s", e.what());
            manip_cost_s2 = HIGH_PENALTY_VALUE_MM; // Assign high penalty on exception
        } catch (...) {
            ROS_ERROR_THROTTLE_NAMED(1.0, LOGNAME_MM, "Unknown exception in motionCost");
            manip_cost_s2 = HIGH_PENALTY_VALUE_MM; // Assign high penalty on exception
        }
        
        // 3. Return combined weighted cost
        // NOTE: Weighting manip_cost_s2 by path_length_cost makes longer moves
        // in low-manipulability areas more expensive. Or just add them: path_length_weight_ * path_length_cost + manip_cost_s2
        return ompl::base::Cost(path_length_weight_ * path_length_cost + manip_cost_s2); 

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