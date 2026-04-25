/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-Chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See repo README.md Task
//    Examples section for task-description examples.
/*************************************/

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <stdexcept>
#include <thread>

class CcaRobot : public cca_ros::CcaRos
{
  public:
    // Status polling frequency in Hz
    static constexpr int kStatusPollRateHz = 4;

    explicit CcaRobot(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cca_ros::CcaRos(node_name, node_options)
    {
    }

    /**
     * @brief Plan and optionally execute a single task.
     *
     * Must be called before block_until_trajectory_execution().
     *
     * @param planning_request The task planning request.
     * @return true if planning succeeded, false otherwise.
     */
    [[nodiscard]] bool run(const cca_ros::PlanningRequest &planning_request)
    {
        cca_ros::PlanningResponse response = this->plan(planning_request);
        motion_status_ = response.status;
        execution_timeout_ = planning_request.execution_timeout;
        return response.result.success;
    }

    /**
     * @brief Plan and optionally execute multiple tasks as a single trajectory.
     *
     * Must be called before block_until_trajectory_execution().
     * The execution timeout is taken from the first request in the vector.
     *
     * @param planning_requests Non-empty vector of task planning requests.
     * @return true if planning succeeded, false otherwise.
     * @throws std::invalid_argument if planning_requests is empty.
     */
    [[nodiscard]] bool run(const std::vector<cca_ros::PlanningRequest> &planning_requests)
    {
        if (planning_requests.empty())
        {
            throw std::invalid_argument("planning_requests must not be empty.");
        }

        cca_ros::PlanningResponse response = this->plan(planning_requests);
        motion_status_ = response.status;
        execution_timeout_ = planning_requests.front().execution_timeout;
        return response.result.success;
    }

    /**
     * @brief Block until the robot completes the planned trajectory or an exit condition is met.
     *
     * Exit conditions (in priority order):
     *   1. Motion succeeds   (SUCCEEDED status)
     *   2. Motion fails      (FAILED status)
     *   3. Motion interrupted (UNKNOWN status)
     *   4. Execution timeout  (configured via PlanningRequest::execution_timeout)
     *   5. ROS shutdown
     *
     * @pre run() must have been called successfully before invoking this method.
     * @throws std::logic_error if called before a successful run().
     */
    void block_until_trajectory_execution()
    {
        if (!motion_status_)
        {
            throw std::logic_error(
                "block_until_trajectory_execution() called before a successful run(). "
                "Call run() first.");
        }

        rclcpp::Rate loop_rate(kStatusPollRateHz);
        const auto start_time = std::chrono::steady_clock::now();

        while (rclcpp::ok())
        {
            const cca_ros::Status current_status = *motion_status_;

            if (current_status == cca_ros::Status::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_node()->get_logger(), "CCA trajectory execution completed successfully.");
                return;
            }

            if (current_status == cca_ros::Status::FAILED)
            {
                RCLCPP_ERROR(this->get_node()->get_logger(), "CCA trajectory execution failed.");
                return;
            }

            if (current_status == cca_ros::Status::UNKNOWN)
            {
                RCLCPP_ERROR(this->get_node()->get_logger(), "Motion was interrupted mid-execution.");
                return;
            }

            const auto elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time);
            if (elapsed >= execution_timeout_)
            {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Timeout waiting for motion to complete after %ld second(s).",
                             execution_timeout_.count());
                return;
            }

            loop_rate.sleep();
        }

        RCLCPP_ERROR(this->get_node()->get_logger(), "Exiting due to ROS shutdown signal.");
    }

  private:
    /// Shared pointer to the current motion execution status. Null until run() is called.
    std::shared_ptr<cca_ros::Status> motion_status_;

    /// Maximum time to wait for trajectory execution. Set by the most recent run() call.
    std::chrono::seconds execution_timeout_{60};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CcaRobot>("cca_ros", rclcpp::NodeOptions{});

    // Spin the node in a background thread to handle ROS communication.
    // std::jthread automatically joins on destruction, ensuring clean shutdown.
    std::jthread spinner_thread([node]() { rclcpp::spin(node->get_node()); });

    /// REQUIRED INPUT: Task description.
    /// The example below performs a 10 cm linear motion along the z-axis from the current
    /// robot configuration. Edit as needed.
    ///
    /// See this package's demo folder or repo README.md for examples covering rotation,
    /// screw, Cartesian goal, end-effector orientation jog, and multi-task trajectories.
    ///------------------------------------------------------------------///

    cca_ros::PlanningRequest req;
    req.planning_group    = "arm";
    req.execution_timeout = std::chrono::seconds(60);

    // Task: translational affordance along the z-axis
    req.task_description =
        cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);
    req.task_description.affordance_info.type     = affordance_util::ScrewType::TRANSLATION;
    req.task_description.affordance_info.axis     = affordance_util::axis_to_vec(affordance_util::Axis::Z);
    req.task_description.affordance_info.location = affordance_util::axis_to_vec(affordance_util::Axis::ORIGIN);

    // Goal: 10 cm motion along z-axis
    req.task_description.goal.affordance = 0.1;

    if (node->run(req))
    {
        RCLCPP_INFO(node->get_node()->get_logger(),
                    "Successfully planned trajectory for '%s' group.",
                    req.planning_group.c_str());
        node->block_until_trajectory_execution();
    }
    else
    {
        RCLCPP_ERROR(node->get_node()->get_logger(),
                     "CCA planning failed for '%s' group.",
                     req.planning_group.c_str());
    }

    ///------------------------------------------------------------------///

    rclcpp::shutdown();
    return 0;
}
