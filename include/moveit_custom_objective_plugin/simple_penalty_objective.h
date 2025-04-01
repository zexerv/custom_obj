#ifndef SIMPLE_PENALTY_OBJECTIVE_H
#define SIMPLE_PENALTY_OBJECTIVE_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ros/console.h> // For ROS_DEBUG

namespace moveit_custom_objective_plugin
{

class SimplePenaltyObjective : public ompl::base::PathLengthOptimizationObjective
{
public:
    SimplePenaltyObjective(const ompl::base::SpaceInformationPtr &si) :
        ompl::base::PathLengthOptimizationObjective(si)
    {
        description_ = "Path Length + Penalty if Joint 0 Positive";
        ROS_DEBUG("SimplePenaltyObjective created.");
    }

    // Override stateCost to add a penalty
    ompl::base::Cost stateCost(const ompl::base::State *s) const override
    {
        ompl::base::Cost parent_cost = ompl::base::PathLengthOptimizationObjective::stateCost(s); // Usually 0
        double penalty = 0.0;
        try {
            // Access joint values - assumes RealVectorStateSpace
            const double* joint_values = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;

            // Check if the first joint (index 0) has a positive value
            if (joint_values[0] > 1e-3) { // Use small tolerance
                penalty = 0.5; // Assign a penalty cost
                // ROS_DEBUG_STREAM_NAMED("SimplePenaltyObjective", "Applying penalty: joint 0 = " << joint_values[0]);
            }
        } catch (const std::bad_cast& e) {
            ROS_ERROR_ONCE_NAMED("SimplePenaltyObjective", "State type cast failed in stateCost: %s", e.what());
        } catch (...) {
             ROS_ERROR_ONCE_NAMED("SimplePenaltyObjective", "Unknown exception in stateCost");
        }


        return ompl::base::Cost(parent_cost.value() + penalty);
    }

    // Inherit motionCost from PathLengthOptimizationObjective
};

} // namespace moveit_custom_objective_plugin

#endif // SIMPLE_PENALTY_OBJECTIVE_H
