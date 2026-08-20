#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <my_robot_interfaces/msg/nav_goal.hpp>
#include <my_robot_interfaces/msg/nav_status.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using NavGoal = my_robot_interfaces::msg::NavGoal;
using NavStatus = my_robot_interfaces::msg::NavStatus;
using Bool = std_msgs::msg::Bool;

/**
 * @brief 导航命令节点 my_robot_nav_commander
 *
 * 模块化设计：本节点只负责"导航"这一件事，通过话题 / action 与外界解耦，
 * 不依赖机械臂 / 夹爪代码，可独立运行（M5 模块）。
 *
 * 接口：
 *   in : nav_cmd     (my_robot_interfaces/NavGoal)  目标位姿 (frame_id, x, y, yaw)
 *   in : nav_cancel  (std_msgs/Bool)                true -> 取消当前导航
 *   out: nav_status  (my_robot_interfaces/NavStatus) IDLE/RUNNING/ARRIVED/FAILED + 当前 map 系位姿
 */
class NavCommander
{
public:
  explicit NavCommander(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    status_pub_ = node_->create_publisher<NavStatus>("nav_status", 10);

    nav_goal_sub_ = node_->create_subscription<NavGoal>(
      "nav_cmd", 10, std::bind(&NavCommander::navGoalCallback, this, std::placeholders::_1));

    nav_cancel_sub_ = node_->create_subscription<Bool>(
      "nav_cancel", 10, std::bind(&NavCommander::navCancelCallback, this, std::placeholders::_1));

    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 周期发布状态（含当前 map 系位姿），5 Hz
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { publishStatus(); });

    status_ = NavStatus::IDLE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<NavStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<NavGoal>::SharedPtr nav_goal_sub_;
  rclcpp::Subscription<Bool>::SharedPtr nav_cancel_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  int32_t status_ = NavStatus::IDLE;

  void navGoalCallback(const NavGoal & msg)
  {
    if (status_ == NavStatus::RUNNING) {
      RCLCPP_WARN(node_->get_logger(), "导航进行中，请先发送 nav_cancel 取消后再下发新目标");
      return;
    }

    if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(node_->get_logger(), "navigate_to_pose action server 不可用，请先启动 Nav2");
      status_ = NavStatus::FAILED;
      return;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = msg.frame_id.empty() ? "map" : msg.frame_id;
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = msg.x;
    goal.pose.pose.position.y = msg.y;
    goal.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, msg.yaw);
    q.normalize();
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(
      node_->get_logger(), "发送导航目标: frame=%s (%.2f, %.2f, yaw=%.2f)",
      goal.pose.header.frame_id.c_str(), msg.x, msg.y, msg.yaw);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavCommander::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&NavCommander::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&NavCommander::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal, send_goal_options);
  }

  void navCancelCallback(const Bool & msg)
  {
    if (msg.data && status_ == NavStatus::RUNNING) {
      RCLCPP_INFO(node_->get_logger(), "收到取消命令，取消当前导航目标");
      client_->async_cancel_all_goals();
    }
  }

  void goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(node_->get_logger(), "导航目标已被接受");
      status_ = NavStatus::RUNNING;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "导航目标被拒绝");
      status_ = NavStatus::FAILED;
    }
  }

  void feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    (void)feedback;
    // 如需导航进度，可在 nav_status 中扩展；当前仅保持 RUNNING 状态
  }

  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "导航成功到达目标");
        status_ = NavStatus::ARRIVED;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "导航被中止（可能规划失败/陷入死区）");
        status_ = NavStatus::FAILED;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "导航被取消");
        status_ = NavStatus::IDLE;
        break;
      default:
        status_ = NavStatus::FAILED;
        break;
    }
  }

  void publishStatus(int32_t forced_status = -1)
  {
    NavStatus msg;
    msg.status = (forced_status >= 0) ? forced_status : status_;

    // 从 TF 获取 map -> base_footlink 的当前位姿
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", "base_footlink", tf2::TimePointZero,
        std::chrono::milliseconds(100));
      msg.current_pose.header.frame_id = "map";
      msg.current_pose.header.stamp = node_->now();
      msg.current_pose.pose.position.x = transform.transform.translation.x;
      msg.current_pose.pose.position.y = transform.transform.translation.y;
      msg.current_pose.pose.position.z = transform.transform.translation.z;
      msg.current_pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException & ex) {
      // map TF 尚未发布（SLAM/AMCL 未就绪），仅发布状态码
      msg.current_pose.header.frame_id = "";
    }
    status_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_robot_nav_commander");
  auto nav_commander = std::make_shared<NavCommander>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
