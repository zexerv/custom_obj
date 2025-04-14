#include <moveit_custom_objective_plugin/pose_goal_region.h>

namespace custom_ompl_plugin
{

// --- Constructor ---
PoseGoalRegion::PoseGoalRegion(const ompl::base::SpaceInformationPtr &si,
                               const planning_scene::PlanningSceneConstPtr& scene,
                               const moveit::core::JointModelGroup* jmg,
                               const geometry_msgs::PoseStamped& target_pose,
                               const std::string& eef_link_name,
                               double pos_tolerance,
                               double ori_tolerance)
    : ompl::base::GoalSampleableRegion(si), // Pass SpaceInformationPtr to base class
      planning_scene_(scene),
      jmg_(jmg),
      target_pose_stamped_(target_pose),
      eef_link_name_(eef_link_name),
      pos_tolerance_sq_(pos_tolerance * pos_tolerance), // Store squared
      ori_tolerance_(ori_tolerance)
{
    // Ensure the target pose is in the planning frame used by OMPL/PlanningScene
    if (!target_pose_stamped_.header.frame_id.empty() &&
         target_pose_stamped_.header.frame_id != planning_scene_->getPlanningFrame())
    {
        // This should ideally be handled *before* calling the constructor,
        // potentially using TF in the planner manager. For now, warn.
        ROS_ERROR("PoseGoalRegion: Target pose frame '%s' does not match planning frame '%s'. Pose comparisons may be incorrect!",
                  target_pose_stamped_.header.frame_id.c_str(), planning_scene_->getPlanningFrame().c_str());
         // Throwing an exception might be safer:
         // throw std::runtime_error("PoseGoalRegion: Target pose frame mismatch.");
    }
    else if (target_pose_stamped_.header.frame_id.empty())
    {
        ROS_WARN("PoseGoalRegion: Target pose has empty frame_id. Assuming it's in the planning frame '%s'.",
                 planning_scene_->getPlanningFrame().c_str());
         target_pose_stamped_.header.frame_id = planning_scene_->getPlanningFrame();
    }


    // // Cast the state space - needed for converting OMPL state to RobotState
    // moveit_state_space_ = dynamic_cast<const moveit_ompl::MoveItStateSpace*>(si_->getStateSpace().get());
    // if (!moveit_state_space_)
    // {
    //      throw std::runtime_error("PoseGoalRegion: Could not cast StateSpace to MoveItStateSpace!");
    // }

        // Cast the state space - needed for converting OMPL state to RobotState
    moveit_state_space_ = dynamic_cast<const ompl_interface::ModelBasedStateSpace*>(si_->getStateSpace().get()); // <<< CORRECT TYPE
    if (!moveit_state_space_)
    {
        throw std::runtime_error("PoseGoalRegion: Could not cast StateSpace to ModelBasedStateSpace!"); // <<< CORRECT MESSAGE
    }
    // Convert target pose to Eigen Isometry for efficient calculations later
    tf2::fromMsg(target_pose_stamped_.pose, target_pose_eigen_);

    // Set the threshold distance (used by some planners like RRTConnect)
    // Using position tolerance seems reasonable as a primary threshold
    threshold_ = pos_tolerance;

    ROS_INFO("PoseGoalRegion initialized for link '%s'. Pos Tolerance: %.4f, Ori Tolerance: %.4f rad",
             eef_link_name_.c_str(), sqrt(pos_tolerance_sq_), ori_tolerance_);
}

// --- Helper Function ---
void PoseGoalRegion::stateOMPLToRobotState(const ompl::base::State *st, moveit::core::RobotState& robot_state) const
{
    // Use the casted MoveItStateSpace pointer to perform the conversion
    moveit_state_space_->copyToRobotState(robot_state, st);
    robot_state.update(true); // Force update joint transforms and link transforms
}

// --- Core Logic: isSatisfied (primary version) ---
bool PoseGoalRegion::isSatisfied(const ompl::base::State *st) const
{
    // 1. Check Collision First (often faster than FK + comparison)
    // Use SpaceInformation's validity checker which should be set up by MoveIt
    if (!si_->isValid(st)) {
        // Uncomment for debugging:
        // ROS_INFO_THROTTLE(1.0, "PoseGoalRegion::isSatisfied: State rejected due to collision.");
        return false;
    }

    // 2. Convert OMPL state to RobotState
    // Create a temporary RobotState based on the PlanningScene's model
    moveit::core::RobotState robot_state(planning_scene_->getRobotModel());
    stateOMPLToRobotState(st, robot_state); // Fill robot_state using helper

    // 3. Get End Effector Pose (ensure link name is valid)
    if (!robot_state.knowsFrameTransform(eef_link_name_)) {
         ROS_ERROR_THROTTLE(5.0, "PoseGoalRegion::isSatisfied: Unknown link name '%s' in RobotState!", eef_link_name_.c_str());
         return false;
    }
    const Eigen::Isometry3d& current_pose_eigen = robot_state.getGlobalLinkTransform(eef_link_name_);

    // 4. Check Position Tolerance (using squared distance)
    double pos_diff_sq = (current_pose_eigen.translation() - target_pose_eigen_.translation()).squaredNorm();
    if (pos_diff_sq > pos_tolerance_sq_) {
         // Uncomment for debugging:
         // ROS_INFO_THROTTLE(1.0, "PoseGoalRegion::isSatisfied: Position tolerance (%.4f > %.4f) failed.", sqrt(pos_diff_sq), sqrt(pos_tolerance_sq_));
         return false;
    }

    // 5. Check Orientation Tolerance (angle between quaternions)
    Eigen::Quaterniond q_current(current_pose_eigen.linear());
    Eigen::Quaterniond q_target(target_pose_eigen_.linear());
    // Ensure quaternions are normalized (should be, but safety check)
    // q_current.normalize();
    // q_target.normalize();
    double angle_diff = q_current.angularDistance(q_target);
    if (angle_diff > ori_tolerance_) {
        // Uncomment for debugging:
        // ROS_INFO_THROTTLE(1.0, "PoseGoalRegion::isSatisfied: Orientation tolerance (%.4f > %.4f) failed.", angle_diff, ori_tolerance_);
        return false;
    }

    // If all checks pass:
    ROS_INFO_THROTTLE(2.0, "PoseGoalRegion::isSatisfied: Goal state FOUND!"); // Report when satisfied
    return true;
}

// --- Core Logic: isSatisfied (version returning distance) ---
bool PoseGoalRegion::isSatisfied(const ompl::base::State *st, double *distance) const
{
    // Calculate the distance first
    *distance = distanceGoal(st);
    // Then check satisfaction using the primary method
    return isSatisfied(st);
}


// --- Distance Metric ---
double PoseGoalRegion::distanceGoal(const ompl::base::State *st) const
{
    // Convert OMPL state to RobotState
    moveit::core::RobotState robot_state(planning_scene_->getRobotModel());
    stateOMPLToRobotState(st, robot_state);

    // Get End Effector Pose
     if (!robot_state.knowsFrameTransform(eef_link_name_)) {
          ROS_ERROR_THROTTLE(5.0, "PoseGoalRegion::distanceGoal: Unknown link name '%s'. Returning large distance.", eef_link_name_.c_str());
          return std::numeric_limits<double>::max(); // Return large distance on error
     }
    const Eigen::Isometry3d& current_pose_eigen = robot_state.getGlobalLinkTransform(eef_link_name_);

    // --- Calculate combined distance ---
    // Option 1: Position distance only (often good enough for guidance)
    double pos_dist = (current_pose_eigen.translation() - target_pose_eigen_.translation()).norm();
    // return pos_dist;

    // Option 2: Weighted position and orientation distance
     Eigen::Quaterniond q_current(current_pose_eigen.linear());
     Eigen::Quaterniond q_target(target_pose_eigen_.linear());
     // q_current.normalize(); // Should be normalized already
     // q_target.normalize();
     double ori_dist = q_current.angularDistance(q_target); // Angle in radians

     // Combine: Tuning the weight is important.
     // Example: Weight orientation less if position is more critical
     double weight_ori = 0.5; // Example weight (tune this!)
     return pos_dist + weight_ori * ori_dist;

    // Note: Distance should ideally reflect the cost function to some extent,
    // but simple geometric distance is usually sufficient for goal guidance.
}


// --- Goal Sampling (Basic - Throws Exception) ---
// Implement a real strategy here if needed later.
void PoseGoalRegion::sampleGoal(ompl::base::State * /*st*/) const
{
    ROS_WARN_ONCE("PoseGoalRegion::sampleGoal is not implemented. Planner will rely on exploration and isSatisfied().");
    // Throwing an exception informs OMPL that goal sampling isn't easily supported.
    // Planners that rely heavily on goal sampling might not work optimally, but
    // RRT/RRT* often work reasonably well without explicit goal sampling.
    throw ompl::Exception("PoseGoalRegion sampling not implemented");

    // --- Placeholder for a future IK-based approach ---
    /*
    // 1. Get an IK solver instance (e.g., from PlanningScene or RobotModel)
    // const moveit::core::JointModelGroup* jmg = robot_state->getJointModelGroup(jmg_name_);
    // kinematics::KinematicsBaseConstPtr solver = jmg->getSolverInstance();

    // 2. Call IK for the target_pose_stamped_
    // std::vector<double> seed_state; // Use current state or default pose
    // robot_state->copyJointGroupPositions(jmg, seed_state);
    // std::vector<double> ik_solution;
    // moveit_msgs::MoveItErrorCodes error_code;
    // if (solver->getPositionIK(target_pose_stamped_.pose, seed_state, ik_solution, error_code)) {

    //     // 3. Create a temporary RobotState with the solution
    //     moveit::core::RobotState ik_state(robot_model_);
    //     ik_state.setJointGroupPositions(jmg, ik_solution);
    //     ik_state.update();

    //     // 4. Convert RobotState to OMPL state
    //     ompl::base::ScopedState<> ompl_state(si_);
    //     moveit_state_space_->copyToOMPLState(ompl_state.get(), ik_state);

    //     // 5. Check if the OMPL state is valid (collision-free)
    //     if (si_->isValid(ompl_state.get())) {
    //         si_->copyState(st, ompl_state.get()); // Copy valid state to output
    //         ROS_INFO_THROTTLE(5.0,"PoseGoalRegion::sampleGoal succeeded via IK.");
    //         return;
    //     } else {
    //          ROS_INFO_THROTTLE(5.0,"PoseGoalRegion::sampleGoal: IK solution is in collision.");
    //     }
    // } else {
    //      ROS_INFO_THROTTLE(5.0,"PoseGoalRegion::sampleGoal: IK solver failed.");
    // }

    // If IK fails or is in collision, fall back to throwing exception or random sampling
    throw ompl::Exception("PoseGoalRegion sampling failed");
    */
}

// --- Max Sample Count ---
// How many times sampleGoal should be called by the planner.
// Since our sampleGoal currently throws, this value is less critical,
// but set it to a reasonable number if you implement sampling later.
unsigned int PoseGoalRegion::maxSampleCount() const
{
    return 100; // Allow more attempts if sampleGoal is implemented
}


} // namespace custom_ompl_plugin