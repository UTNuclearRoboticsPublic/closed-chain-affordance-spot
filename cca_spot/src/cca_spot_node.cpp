/*************************************/

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

class CcaSpot : public cca_ros::CcaRos
{
  public:
    explicit CcaSpot(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cca_ros::CcaRos(node_name, node_options)
    {
    }

    // Function to run the planner for a given task and/or execute that task on the robot
    bool run(const cca_ros::PlanningRequest &planning_request)
    {
        motion_status_ = planning_request.status;

        return this->plan_visualize_and_execute(planning_request);
    }
    // Function overload to plan multiple tasks at once
    bool run(const cca_ros::PlanningRequests &planning_requests)
    {
        motion_status_ = planning_requests.status;

        return this->plan_visualize_and_execute(planning_requests);
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
    bool includes_gripper_goal_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcaSpot>("cca_ros", node_options);

    RCLCPP_INFO(node->get_logger(), "CCA Planner is active");

    // Spin the node so joint states can be read
    std::thread spinner_thread([node]() { rclcpp::spin(node); });

    /// REQUIRED INPUT: Task description. For quick start, the following block provides an example task description to
    /// do a simple linear motion along the z-axis from the current robot configuration. Edit as needed. See this
    /// package's demo folder or repo README.md for various other examples that cover motions including rotation, screw,
    /// cartesian goal, ee orientation jog, etc. It is also possible to plan multiple of these tasks together as a long
    /// joint trajectory.
    cca_ros::PlanningRequest req;
    req.planner_config.accuracy = 0.01;

    ///------------------------------------------------------------------///
    //---PELICAN 1---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << -0.30301,0.05926,1.30502,-0.41824,-1.04204,0.09252)
    //         .finished(); 

    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info
    // req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(0, -1, 0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.945425, -0.133055, -0.464318);
    // req.task_description.goal.affordance = 0.628319; // 
    // // req.task_description.goal.affordance = 1.392689; // Max

    // req.task_description.vir_screw_order = affordance_util::VirtualScrewOrder::NONE; // To juxtapose visual

    ///------------------------------------------------------------------///
    ////---PELICAN 2---//
        
    //req.start_state.robot =(Eigen::VectorXd(6) << -0.10534,0.30302,1.23042,-0.08751,-1.45407,0.00974)
    //        .finished(); 
    //// Specify planning type
    //req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    //// Affordance info 
    //req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    //req.task_description.affordance_info.axis = Eigen::Vector3d(0 ,-1, 0);
    //req.task_description.affordance_info.location = Eigen::Vector3d(0.895717 ,-0.0469712,  -0.516762);
    //req.task_description.goal.affordance = 0.785398; // 
    //// req.task_description.goal.affordance = M_PI/2.0; // Max
    ///------------------------------------------------------------------///
    //---PELICAN 3---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << -0.05821,0.30447,1.13644,-0.10949,-1.35613,-0.00345)
    //         .finished(); 
    // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // Affordance info -- Pelican 3
    // req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(0 ,-1, 0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.933395 ,-0.0165794,  -0.508373);
    // req.task_description.goal.affordance = 0.837758; // 
    // req.task_description.goal.affordance = 1.53; // Max
    ///------------------------------------------------------------------///
    //---PELICAN 4---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << -0.15015,0.29943,0.86748,0.05041,-0.75459,-1.67907)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- Pelican 4
    // req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(0 ,-1, 0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(1.0107 ,-0.115859 ,-0.519666);
    // req.task_description.goal.affordance = 0.448799; // 
    // // req.task_description.goal.affordance = 1.152038; // Max
    ///------------------------------------------------------------------///

    ////---PELICAN 5---//
        
    //req.start_state.robot =(Eigen::VectorXd(6) << -0.22924,0.31410,0.93179,-0.22954,-1.15600,0.06513)
    //        .finished(); 
    //// Specify planning type
    //req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    //// Affordance info -- Pelican 5
    //req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    //req.task_description.affordance_info.axis = Eigen::Vector3d(0 ,-1, 0);
    //req.task_description.affordance_info.location = Eigen::Vector3d(0.992715, -0.122151, -0.481894);
    //req.task_description.goal.affordance = 0.698132; // 
    //// req.task_description.goal.affordance = 1.218319; // Max
    ///------------------------------------------------------------------///
    //---DUFFLEBAG 1---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << -0.00842,-0.66004,0.64303,-0.01130,-0.02389,0.01587)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- Dufflebag 1
    // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(0 ,-1,  0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.864747 ,-0.00722478,0.297397);
    // req.task_description.goal.affordance = 0.15; // 
    // // req.task_description.goal.affordance = 0.23; // Max
    ///------------------------------------------------------------------///
    //---DUFFLEBAG 2---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << 0.04015,-0.04417,1.15805,-0.15848,-0.22914,0.01404)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- Dufflebag 2
    // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(0 ,-1  ,0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.705944 ,0.0353738 ,-0.465464);
    // req.task_description.goal.affordance = 0.3; // 
    // // req.task_description.goal.affordance = 0.438907; // Max
    ///------------------------------------------------------------------///
    //---DUFFLEBAG 3---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << -0.22567,-0.46629,2.00996,-0.10792,0.04345,-0.03772)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- Dufflebag 3
    // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(1 ,0 ,0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.375161 ,-0.087067 ,-0.444478);
    // req.task_description.goal.affordance = 0.3; // 
    // // req.task_description.goal.affordance = 0.438897; // Max
    ///------------------------------------------------------------------///
    // ---DUFFLEBAG 4---//
        
     // req.start_state.robot =(Eigen::VectorXd(6) << -0.04231,0.20482,0.52777,-2.53572,-0.20273,2.50973)
     //         .finished(); 
     // // Specify planning type
     // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

     // // Affordance info -- Dufflebag 4
     // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
     // req.task_description.affordance_info.axis = Eigen::Vector3d(-1  ,0  ,0);
     // req.task_description.affordance_info.location = Eigen::Vector3d(0.802418 ,-0.0115271  ,-0.434995);
     // req.task_description.goal.affordance = 0.3; // 
     // // req.task_description.goal.affordance = 0.78; // Max
    ///------------------------------------------------------------------///
    ////---DUFFLEBAG 5---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << 0.87980,-0.32586,1.69539,1.23820,-0.64217,-0.35225)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- Dufflebag 5
    // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(1 ,0 ,0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.431742  ,0.348325 ,-0.417611);
    // req.task_description.goal.affordance = 0.2; // 
    // // req.task_description.goal.affordance = 0.32; // Max
    ///------------------------------------------------------------------///
    ////---NUT 1---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << 0.23386,-0.54563,1.08410,-0.34835,-0.50256,0.11729)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- NUT 1
    // req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(1.0 ,0.0,  0.0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.835545  ,0.312095 ,0.0213549);
    // req.task_description.affordance_info.pitch = 0.01;
    // req.task_description.goal.affordance =0.785398; // 
    // // req.task_description.goal.affordance = 2.642888; // Max
    ///------------------------------------------------------------------///
    ////---NUT 2---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) << 0.30567,-0.61542,0.98007,1.56896,-0.08781,-2.46185)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- NUT 2
    // req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(-1.0, 0.0, 0.0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.82715  ,0.323029 ,0.0522127);
    // req.task_description.affordance_info.pitch = 0.01;
    // req.task_description.goal.affordance =0.785398; // 
    // // req.task_description.goal.affordance = 15*M_PI; // Max
    ///------------------------------------------------------------------///
    //---NUT 3---//
        
    // req.start_state.robot =(Eigen::VectorXd(6) <<-0.03947,-0.72948,1.24571,-1.59876,0.75301,2.38479 )
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- NUT 3
    // req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(1,  0,  0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.760197 ,-0.243811 ,0.0245757);
    // req.task_description.affordance_info.pitch = 0.01;
    // req.task_description.goal.affordance =0.785398; // 
    // // req.task_description.goal.affordance = 14.206378; // Max
    ///------------------------------------------------------------------///
    ////---NUT 4---//
    // req.start_state.robot =(Eigen::VectorXd(6) <<-0.26365,-0.32357,0.72647,0.59626,-0.49524,-0.49605)
    //         .finished(); 
    // // Specify planning type
    // req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // // Affordance info -- NUT 4
    // req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
    // req.task_description.affordance_info.axis = Eigen::Vector3d(1,  0,  0);
    // req.task_description.affordance_info.location = Eigen::Vector3d(0.864756 ,-0.367491 ,0.0217763);
    // req.task_description.affordance_info.pitch = 0.01;
    // req.task_description.goal.affordance =0.785398; // 
    // // req.task_description.goal.affordance = 1.245632; // Max
    ///------------------------------------------------------------------///
    ////---NUT 5---//
    req.start_state.robot =(Eigen::VectorXd(6) <<-0.24940,-0.44696,0.92226,0.66251,-0.53774,-0.30187)
            .finished(); 
    // Specify planning type
    req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // Affordance info -- NUT 5
    req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
    req.task_description.affordance_info.axis = Eigen::Vector3d(1,  0,  0);
    req.task_description.affordance_info.location = Eigen::Vector3d(0.84111, -0.357813, 0.0218069);
    req.task_description.affordance_info.pitch = 0.01;
    req.task_description.goal.affordance =1.0472; // 
    // req.task_description.goal.affordance = 1.861276; // Max
    ///------------------------------------------------------------------///
    req.task_description.trajectory_density = 50;

    // Run CCA planner and executor
    if (node->run(req))
    {
        RCLCPP_INFO(node->get_logger(), "Successfully called CCA action");
        node->block_until_trajectory_execution(); // Optionally, block until execution
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "CCA action failed");
        rclcpp::shutdown();
    }

    if (spinner_thread.joinable())
    {
        spinner_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
