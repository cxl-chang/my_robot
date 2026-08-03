#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using namespace std::placeholders;

class commander
{
public:
    commander(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        arm = std::make_shared<MoveGroupInterface>(node, "arm");
        arm->setMaxAccelerationScalingFactor(1.0);
        arm->setMaxVelocityScalingFactor(1.0);
        gripper = std::make_shared<MoveGroupInterface>(node, "gripper");
        gripper_sub_ = node_->create_subscription<Bool>(
            "gripper_cmd", 10, std::bind(&commander::gripperCallback, this, _1)
        );

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_cmd", 10, std::bind(&commander::jonitCmdCallback, this, _1)
        );
    }

    void planAndExecute(const std::shared_ptr<MoveGroupInterface>& groupInterface)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (groupInterface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            RCLCPP_INFO(node_->get_logger(), "Planning was successful.");
            groupInterface->execute(plan);
            RCLCPP_INFO(node_->get_logger(), "Execution of the plan has started.");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
        }
    }

    void goToArmNamedTarget(const std::string& target_name) 
    {
        arm->setStartStateToCurrentState();
        arm->setNamedTarget(target_name);
        planAndExecute(arm);
    }

    void goToArmJointTarget(const std::vector<double>& target_joints)
    {
        arm->setStartStateToCurrentState();
        arm->setJointValueTarget(target_joints);
        planAndExecute(arm);
    }

    void goToArmPoseTarget(double x, double y, double z, 
                            double roll, double pitch, double yaw, bool useCartisian = false) 
    {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_footlink";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();

        arm->setStartStateToCurrentState();

        if(!useCartisian)
        {
            arm->setPoseTarget(target_pose);
            planAndExecute(arm);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            const double jump_threshold = 0.0;
            const double eef_step = 0.01; // 1cm

            double fraction = arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            if (fraction == 1) {
                arm->execute(trajectory);
            }
        }
    }


     void openGripper()
    {
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        planAndExecute(gripper);
    }

    void closeGripper()
    {
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_closed");
        planAndExecute(gripper);
    }


private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MoveGroupInterface> arm;
    std::shared_ptr<MoveGroupInterface> gripper;

    rclcpp::Subscription<Bool>::SharedPtr gripper_sub_;

    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;

    void gripperCallback(const Bool& msg)
    {
        if (msg.data) {
            openGripper();
        } else {
            closeGripper();
        }
    }

    void jonitCmdCallback(const FloatArray & msg)
    {
        auto joints = msg.data;
        if(joints.size() == 6) { 
            goToArmJointTarget(joints);
        }
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = rclcpp::Node::make_shared("my_robot_commander", options);
    auto commander_node = std::make_shared<commander>(node);
    rclcpp::spin(node);

    // Your code here

    rclcpp::shutdown();
    return 0;
}