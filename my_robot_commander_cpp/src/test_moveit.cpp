#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_moveit_node");
  RCLCPP_INFO(node->get_logger(), "Test MoveIt Node has started.");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
  RCLCPP_INFO(node->get_logger(), "MoveGroupInterface for 'arm' has been created.");

  arm_group->setMaxAccelerationScalingFactor(1.0);
  arm_group->setMaxVelocityScalingFactor(1.0);
  RCLCPP_INFO(node->get_logger(), "Max acceleration and velocity scaling factors set to 1.0.");

  arm_group->setStartStateToCurrentState();
  arm_group->setNamedTarget("pose_2");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to 'pose_2' was successful.");
    arm_group->execute(my_plan);
    RCLCPP_INFO(node->get_logger(), "Execution of the plan to 'pose_2' has started.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning to 'pose_2' failed.");
  }


  //gripper
  auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
  RCLCPP_INFO(node->get_logger(), "MoveGroupInterface for 'gripper' has been created.");
  gripper_group->setStartStateToCurrentState();
  gripper_group->setNamedTarget("gripper_closed");
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
  bool gripper_success = (gripper_group->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (gripper_success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning for 'gripper_closed' was successful.");
    gripper_group->execute(gripper_plan);
    RCLCPP_INFO(node->get_logger(), "Execution of the plan for 'gripper_closed' has started.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning for 'gripper_closed' failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}