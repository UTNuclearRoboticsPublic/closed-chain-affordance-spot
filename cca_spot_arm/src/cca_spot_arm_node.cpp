/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See repo README.md Task
// Examples section for task-description examples.
/*************************************/
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <thread>

class CcaSpotArm : public cca_ros::CcaRos
{
  public:
    explicit CcaSpotArm(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cca_ros::CcaRos(node_name, node_options)
    {
    }

    // Function to run the planner for a given task and/or execute that task on the robot
    bool run(const cca_ros::PlanningRequest &planning_request)
    {

	cca_ros::PlanningResponse response = this->plan(planning_request);
        motion_status_ = response.status;
	return response.result.success;
    }
    // Function overload to plan multiple tasks at once
    bool run(const std::vector<cca_ros::PlanningRequest> &planning_requests)
    {

	cca_ros::PlanningResponse response = this->plan(planning_requests);
        motion_status_ = response.status;
	return response.result.success;
    }

    // Function to block until the robot completes the planned trajectory
    void block_until_trajectory_execution()
    {
        rclcpp::Rate loop_rate(4);
        auto start_time = std::chrono::steady_clock::now();

        while (*motion_status_ != cca_ros::Status::SUCCEEDED)
        {
            if (*motion_status_ == cca_ros::Status::UNKNOWN)
            {
                RCLCPP_ERROR(this->get_logger(), "Motion was interrupted mid-execution.");
                auto current_time = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 60)
                {
                    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for motion to complete.");
                    return;
                }
            }
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Exiting due to ROS signal");
                return;
            }
            loop_rate.sleep();
        }
    }

  private:
    std::shared_ptr<cca_ros::Status> motion_status_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<CcaSpotArm>("cca_ros", node_options);

    RCLCPP_INFO(node->get_logger(), "CCA Planner is active");

    // Spin the node so joint states can be read
    std::jthread spinner_thread([node]() { rclcpp::spin(node); });

    /// REQUIRED INPUT: Task description. For quick start, the following block provides an example task description to
    /// do a simple linear motion along the z-axis from the current robot configuration. Edit as needed. See this
    /// package's demo folder or repo README.md for various other examples that cover motions including rotation, screw,
    /// cartesian goal, ee orientation jog, etc. It is also possible to plan multiple of these tasks together as a long
    /// joint trajectory.
    ///------------------------------------------------------------------///
    // cca_ros::PlanningRequest req;
    //
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);
    //
    // // Affordance info
    // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
    // req.task_description.affordance_info.location = Eigen::Vector3d::Zero();
    //
    // // Goals
    // req.task_description.goal.affordance = 0.1; // Set desired goal for the affordance

    std::vector<cca_ros::PlanningRequest> reqs;
    cca_ros::PlanningRequest approach_req;
    approach_req.planner_config.ik_max_itr = 10000;
    approach_req.execute_trajectory = true;

    const double GRIPPER_OPEN = -1.57;  // radians
    const double GRIPPER_CLOSED = 0.0; // radians
    // Task description
    // Specify planning type
    approach_req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);
    // Affordance info
    approach_req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    approach_req.task_description.affordance_info_from.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
    approach_req.task_description.affordance_info_from.frame_name = "trashcan_frame";
    approach_req.task_description.affordance_info_from.axis_in_final_pose = affordance_util::axis_to_vec(affordance_util::Axis::Y);

    // Affordative pose
    approach_req.task_description.canonical_pose_from.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
    approach_req.task_description.canonical_pose_from.frame_name = approach_req.task_description.affordance_info_from.frame_name;
    Eigen::Isometry3d canonical_pose_post_transform = Eigen::Isometry3d::Identity();

    // Set grasp orientation
    canonical_pose_post_transform.linear().col(0) = affordance_util::axis_to_vec(affordance_util::Axis::Y_MINUS);
    canonical_pose_post_transform.linear().col(1) = affordance_util::axis_to_vec(affordance_util::Axis::X_MINUS);
    canonical_pose_post_transform.linear().col(2) = affordance_util::axis_to_vec(affordance_util::Axis::Z_MINUS);
    approach_req.task_description.canonical_pose_from.post_transform = canonical_pose_post_transform.matrix();

   // Goals
   approach_req.task_description.goal.affordance = 0.2;
   approach_req.task_description.goal.gripper = GRIPPER_OPEN;

   // Other things
   approach_req.task_description.gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
   // approach_req.task_description.trajectory_density = 30;
   // approach_req.time_step.robot_and_gripper = 0.5;
   // approach_req.time_step.robot = 0.5;
   // approach_req.time_step.gripper = 0.5;

   reqs.push_back(approach_req);

   cca_ros::PlanningRequest hone_in_req;
   hone_in_req.task_description.affordance_info_from = approach_req.task_description.affordance_info_from;
   hone_in_req.task_description.affordance_info = approach_req.task_description.affordance_info;

   // Goals
   hone_in_req.task_description.goal.affordance = -0.1;
   hone_in_req.task_description.goal.gripper = GRIPPER_CLOSED;

   // Other things
   hone_in_req.task_description.gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
   // hone_in_req.task_description.trajectory_density = 20;
   // hone_in_req.time_step.robot_and_gripper = 0.5;
   // hone_in_req.time_step.robot = 0.5;
   // hone_in_req.time_step.gripper = 0.5;

   reqs.push_back(hone_in_req);
    ///------------------------------------------------------------------///

    // Run CCA planner and executor
    if (node->run(reqs))
    {
        RCLCPP_INFO(node->get_logger(), "Successfully called CCA action");
        node->block_until_trajectory_execution(); // Optionally, block until execution
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "CCA action failed");
        rclcpp::shutdown();
    }

    rclcpp::shutdown();
    return 0;
}
