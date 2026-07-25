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

//   arm_group->setStartStateToCurrentState();
//   arm_group->setNamedTarget("pose_2");
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (arm_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   if (success)
//   {
//     RCLCPP_INFO(node->get_logger(), "Planning to 'pose_2' was successful.");
//     arm_group->execute(my_plan);
//     RCLCPP_INFO(node->get_logger(), "Execution of the plan to 'pose_2' has started.");
//   }
//   else
//   {
//     RCLCPP_ERROR(node->get_logger(), "Planning to 'pose_2' failed.");
//   }


//   //gripper
//   auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
//   RCLCPP_INFO(node->get_logger(), "MoveGroupInterface for 'gripper' has been created.");
//   gripper_group->setStartStateToCurrentState();
//   gripper_group->setNamedTarget("gripper_closed");
//   moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
//   bool gripper_success = (gripper_group->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   if (gripper_success)
//   {
//     RCLCPP_INFO(node->get_logger(), "Planning for 'gripper_closed' was successful.");
//     gripper_group->execute(gripper_plan);
//     RCLCPP_INFO(node->get_logger(), "Execution of the plan for 'gripper_closed' has started.");
//   }
//   else
//   {
//     RCLCPP_ERROR(node->get_logger(), "Planning for 'gripper_closed' failed.");
//   }

  //=========================joint goal=========================
    // std::vector<double> joint_group_positions = {0.0, 0.0, -1.8, 0.0, 0.0, 0.0};
    // arm_group->setStartStateToCurrentState();
    // arm_group->setJointValueTarget(joint_group_positions);
    // moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    // bool joint_success = (arm_group->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (joint_success)
    // {
    //     RCLCPP_INFO(node->get_logger(), "Planning to joint goal was successful.");
    //     arm_group->execute(joint_plan);
    //     RCLCPP_INFO(node->get_logger(), "Execution of the plan to joint goal has started.");
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning to joint goal failed.");
    // }

    //=========================pose goal=========================
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_footlink";
    target_pose.pose.position.x = 0.843; //0.843, -0.235, 0.478
    target_pose.pose.position.y = -0.235;
    target_pose.pose.position.z = 0.478;

    tf2::Quaternion q;
    q.setRPY(-2.551, -0.877, -0.828); // -2.551, -0.877, -0.828
    q = q.normalize();
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();

    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    bool pose_success = (arm_group->plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (pose_success)
    {
        RCLCPP_INFO(node->get_logger(), "Planning to pose goal was successful.");
        arm_group->execute(pose_plan);
        RCLCPP_INFO(node->get_logger(), "Execution of the plan to pose goal has started.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning to pose goal failed.");
    }


  rclcpp::shutdown();
  spinner.join();
  return 0;
}