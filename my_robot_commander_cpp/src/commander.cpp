// my_robot_commander：MoveIt 手臂/夹爪执行入口（M11 起同时提供 action 接口）
//
// 两套接口并存：
//
//  1) 话题（原有，保留兼容，适合手动调试，fire-and-forget）
//       /pose_cmd    my_robot_interfaces/PoseCommand   位姿目标（base_footlink 系）
//       /joint_cmd   Float64MultiArray[6]              关节目标 joint1..joint6
//       /gripper_cmd example_interfaces/Bool           true=张开 false=闭合
//
//  2) action（新增，M9/M12 的编排节点用）
//       /arm_task    my_robot_interfaces/action/ArmTask
//     为什么要加它：话题是"发完不管"，编排节点无法知道规划成没成、执行完没完，
//     抓取失败也无从重试。action 能返回 success/message，并支持取消。
//
// 线程模型（重要）：
//   MoveGroupInterface 内部依赖订阅（/joint_states、TF 等）持续被 spin。
//   如果 action 回调（规划+执行，阻塞数秒）和这些内部订阅共用一个回调组，
//   即使是多线程执行器也会互相饿死 → plan() 卡死。
//   所以 action server 挂在独立回调组，主函数用 MultiThreadedExecutor。

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <my_robot_interfaces/action/arm_task.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCommand = my_robot_interfaces::msg::PoseCommand;
using ArmTask = my_robot_interfaces::action::ArmTask;
using GoalHandleArmTask = rclcpp_action::ServerGoalHandle<ArmTask>;
using namespace std::placeholders;

namespace {
constexpr char kArmFrame[] = "base_footlink";  // 手臂位姿目标的参考系
// 执行后"到位校验"允许的位置误差（米）。超过就判这次动作失败。
constexpr double kAchieveTol = 0.005;  // 5mm
}  // namespace

class commander
{
public:
    explicit commander(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        // action server 使用独立回调组，避免与 MoveIt 内部订阅互相饿死
        action_cb_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 规划预算上限（秒）。**必须明显小于调用方的 arm_timeout**：
        // 之前把 planner 预算直接设成 goal->timeout（两边都是 60s），结果规划
        // 正好在第 60.0016s 出解，而 orchestrator 的 60s 超时同时触发 → 判超时、
        // 发取消 → 任务失败。规划 + 执行 + 传输都要在这个 timeout 之内，所以
        // 给规划单独设一个上限。
        node_->declare_parameter("plan_time_cap", 20.0);
        plan_time_cap_ = node_->get_parameter("plan_time_cap").as_double();

        arm = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm->setMaxAccelerationScalingFactor(1.0);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setPlanningTime(10.0);
        arm->setNumPlanningAttempts(10);   // 采样规划有随机性，多试几次提高成功率

        gripper = std::make_shared<MoveGroupInterface>(node_, "gripper");
        gripper->setMaxAccelerationScalingFactor(1.0);
        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setPlanningTime(5.0);

        gripper_sub_ = node_->create_subscription<Bool>(
            "gripper_cmd", 10, std::bind(&commander::gripperCallback, this, _1));
        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_cmd", 10, std::bind(&commander::jointCmdCallback, this, _1));
        pose_cmd_sub_ = node_->create_subscription<PoseCommand>(
            "pose_cmd", 10, std::bind(&commander::poseCmdCallback, this, _1));

        action_server_ = rclcpp_action::create_server<ArmTask>(
            node_, "arm_task",
            std::bind(&commander::handleGoal, this, _1, _2),
            std::bind(&commander::handleCancel, this, _1),
            std::bind(&commander::handleAccepted, this, _1),
            rcl_action_server_get_default_options(),
            action_cb_group_);

        RCLCPP_INFO(node_->get_logger(),
                    "my_robot_commander 就绪：/arm_task (action) + "
                    "/pose_cmd /joint_cmd /gripper_cmd (话题)");
    }

    // ================= 基础动作（返回是否成功） =================

    /// 规划并执行；返回是否"规划成功且开始执行"。注意 execute() 是阻塞的，
    /// 返回后表示轨迹已执行结束（不代表每一点都完美跟踪）。
    bool planAndExecute(const std::shared_ptr<MoveGroupInterface>& group,
                        const std::string& what, std::string& err)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto code = group->plan(plan);
        if (code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            err = what + " 规划失败（MoveItErrorCode=" +
                  std::to_string(code.val) + "）";
            RCLCPP_ERROR(node_->get_logger(), "%s", err.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "%s 规划成功，开始执行", what.c_str());
        group->execute(plan);
        RCLCPP_INFO(node_->get_logger(), "%s 执行结束", what.c_str());
        return true;
    }

    bool goNamed(const std::string& group_name, const std::string& target,
                 std::string& err)
    {
        auto group = (group_name == "gripper") ? gripper : arm;
        if (group_name != "arm" && group_name != "gripper")
        {
            err = "group_name 只支持 'arm' 或 'gripper'，收到: " + group_name;
            return false;
        }
        group->setStartStateToCurrentState();
        group->setNamedTarget(target);
        return planAndExecute(group, group_name + " -> " + target, err);
    }

    bool goJoint(const std::vector<double>& joints, std::string& err)
    {
        if (joints.size() != 6)
        {
            err = "joint_values 需要 6 个值（joint1..joint6），收到 " +
                  std::to_string(joints.size()) + " 个";
            return false;
        }
        arm->setStartStateToCurrentState();
        arm->setJointValueTarget(joints);
        return planAndExecute(arm, "arm -> joint target", err);
    }

    /// 执行后校验：用 FK 算出末端实际位姿，与目标比位置误差，超阈值即判失败。
    ///
    /// 这是防止"规划/执行报成功、手臂却没到位"的最后一道闸门。
    /// 为什么必须有它：MoveIt 的 setApproximateJointValueTarget() 会把
    /// KinematicsQueryOptions.return_approximate_solution 置 true，IK 没收敛也
    /// 把最接近的解当成功返回。实测：命令 tool_link 到 (0.42,0.078,0.60)，
    /// 手臂实际停在 (0.343,0.034,0.734)（偏 14.4cm，Gazebo 物理位姿 + TF 双确认），
    /// action 却报 success；紧接着的笛卡尔下压起点就离目标 0.25m（本该 0.10m），
    /// 直线插补在最后一个点失败 → "笛卡尔路径只算到 97.96%"，整个抓取中止。
    bool verifyAchieved(const geometry_msgs::msg::PoseStamped& target,
                        std::string& err)
    {
        auto state = arm->getCurrentState(1.0);
        if (!state)
        {
            RCLCPP_WARN(node_->get_logger(), "拿不到当前状态，跳过到位校验");
            return true;
        }
        const Eigen::Isometry3d& got =
            state->getGlobalLinkTransform(arm->getEndEffectorLink());
        const Eigen::Vector3d want(target.pose.position.x,
                                   target.pose.position.y,
                                   target.pose.position.z);
        const double d = (got.translation() - want).norm();
        if (d > kAchieveTol)
        {
            err = "执行后末端位置误差 " + std::to_string(d * 1000.0) +
                  " mm（阈值 " + std::to_string(kAchieveTol * 1000.0) +
                  " mm）：手臂没到位";
            RCLCPP_ERROR(node_->get_logger(), "%s", err.c_str());
            return false;
        }
        return true;
    }

    /// 位姿 → 关节角的**精确** IK，返回多组候选解（按"离当前构型由近到远"排序）。
    ///
    /// 为什么需要它（实测踩过的两个坑）：
    ///  1) 不能用 setApproximateJointValueTarget()：它请求"近似解"
    ///     （KinematicsQueryOptions.return_approximate_solution=true），IK 没收敛
    ///     也报成功 → 命令 (0.42,0.078,0.60) 手臂停在 (0.343,0.034,0.734)，
    ///     偏 14.4cm，随后笛卡尔下压起点离目标 0.25m → 最后一点插补失败。
    ///  2) 也不能只信一组解：本臂对同一目标位姿有多组解。预抓取位姿
    ///     (0.42,0.078,0.60) 有一组"基座 joint1≈-2.89 rad（转 166°）"的解，
    ///     它离 carry 构型 4.55 rad，而该分支与当前构型之间没有无碰通路 →
    ///     RRTConnect 20s 找不到路径 → MoveItErrorCode=99999，抓取直接失败；
    ///     另一组紧凑解离 carry 只有 1.35 rad。
    ///  所以这里用 当前状态 / SRDF 的 home / carry 三种子分别解 IK，去重后按
    ///  关节空间距离排序返回，由调用方"由近到远逐个尝试规划"——近解优先（少动、
    ///  且几乎总能规划出来），近解不行再退到远解，避免一次坏种子就判死整个任务。
    std::vector<std::vector<double>> poseIkCandidates(
        const geometry_msgs::msg::PoseStamped& target, std::string& err)
    {
        std::vector<std::vector<double>> out;
        const auto model = arm->getRobotModel();
        const moveit::core::JointModelGroup* jmg =
            model ? model->getJointModelGroup(arm->getName()) : nullptr;
        auto current = arm->getCurrentState(1.0);
        if (jmg == nullptr || !current)
        {
            err = "拿不到关节模型或当前状态，无法做位姿 IK";
            return out;
        }
        std::vector<double> q_now;
        current->copyJointGroupPositions(jmg, q_now);
        const std::string eef = arm->getEndEffectorLink();

        std::vector<std::pair<double, std::vector<double>>> found;
        for (const std::string nm : {"", "home", "carry"})
        {
            moveit::core::RobotState seed(*current);
            if (!nm.empty() && !seed.setToDefaultValues(jmg, nm))
            {
                continue;  // SRDF 里没有这个命名状态
            }
            if (!seed.setFromIK(jmg, target.pose, eef, 0.2))
            {
                continue;
            }
            std::vector<double> q;
            seed.copyJointGroupPositions(jmg, q);
            double d2 = 0.0;
            for (size_t i = 0; i < q_now.size() && i < q.size(); ++i)
            {
                const double dd = q_now[i] - q[i];
                d2 += dd * dd;
            }
            // 去重：与已有候选差别很小就不重复加
            bool dup = false;
            for (const auto& f : found)
            {
                double e2 = 0.0;
                for (size_t i = 0; i < f.second.size() && i < q.size(); ++i)
                {
                    const double dd = f.second[i] - q[i];
                    e2 += dd * dd;
                }
                if (e2 < 1e-6)
                {
                    dup = true;
                    break;
                }
            }
            if (!dup)
            {
                found.emplace_back(std::sqrt(d2), q);
            }
        }

        std::sort(found.begin(), found.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& f : found)
        {
            out.push_back(f.second);
        }
        if (out.empty())
        {
            err = "位姿 IK 无解（精确解）：目标超出工作空间或姿态不可达";
        }
        else
        {
            std::string ds;
            for (const auto& f : found)
            {
                ds += " " + std::to_string(f.first);
            }
            RCLCPP_INFO(node_->get_logger(),
                        "位姿 IK 得到 %zu 组候选解，距当前构型(rad):%s",
                        out.size(), ds.c_str());
        }
        return out;
    }

    bool goPose(double x, double y, double z, double roll, double pitch,
                double yaw, bool use_cartesian, std::string& err)
    {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = kArmFrame;
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

        if (!use_cartesian)
        {
            // 【重要】位姿目标改成"先解一次 IK，再用关节目标规划"。
            //
            // 原因：OMPL 对**位姿目标**要在目标区域反复采样 IK 才能判定是否到达，
            // 而本臂 IK 在目标位姿附近的可解区域很窄，实测一个自由空间位姿能耗光
            // 整个规划预算（60s）甚至根本规划不出来；而**关节目标**是秒级完成的
            // （SRDF 里的 carry 等命名目标就是关节目标，一直是 ~1s）。
            //
            // setJointValueTarget(PoseStamped) 内部做一次**精确** IK，并把结果当作
            // JointValueTarget 交给规划器 —— 既保留"按位姿下指令"的接口语义，
            // 又让规划退化成关节空间问题。
            //
            // 【坑，切勿改回 setApproximateJointValueTarget()】
            //  该接口把 KinematicsQueryOptions.return_approximate_solution 置 true：
            //  IK 没收敛也把最接近的解当成功返回。实测命令 tool_link 到
            //  (0.42,0.078,0.60)，手臂停在 (0.343,0.034,0.734)（偏 14.4cm）还报成功，
            //  直接导致抓取时序里"预抓取"停在 z≈0.73，随后的笛卡尔下压起点离目标
            //  0.25m（本该 0.10m）→ 直线插补最后一点失败 → 整个抓取中止。
            //  注意 setJointValueTarget 即使失败也会把"部分结果"留在目标里，
            //  所以失败必须立刻返回，不能再拿它去规划。
            //
            //  再补一层（关键）：位姿 → 关节角要"多组种子择优"。
            //  本臂对同一目标位姿有多组解。实测预抓取位姿 (0.42,0.078,0.60) 会
            //  被解到"基座 joint1≈-2.89 rad（转了 166°）"那一组，它离当前 carry
            //  构型 4.55 rad；而该分支与当前构型之间没有无碰通路 → RRTConnect
            //  20s 找不到路径 → MoveItErrorCode=99999，抓取直接失败。
            //  另一组紧凑解（joint1≈+0.25）离 carry 只有 1.35 rad，规划秒过。
            //  所以这里用当前状态 / SRDF 的 home / carry 三种子分别解 IK，
            //  按"离当前构型由近到远"排序后**逐个尝试规划+执行+到位校验**：
            //  近解优先（少动、且几乎总能规划出来），近解规划不出来再退到远解，
            //  避免一次坏种子就判死整个抓取任务。
            const auto candidates = poseIkCandidates(target_pose, err);
            if (candidates.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "%s", err.c_str());
                return false;
            }
            std::string last_err = err;
            for (size_t i = 0; i < candidates.size(); ++i)
            {
                arm->setStartStateToCurrentState();
                if (!arm->setJointValueTarget(candidates[i]))
                {
                    last_err = "第 " + std::to_string(i + 1) + " 组解设置失败";
                    continue;
                }
                RCLCPP_INFO(node_->get_logger(), "尝试第 %zu/%zu 组 IK 解",
                            i + 1, candidates.size());
                std::string e;
                if (!planAndExecute(arm, "arm -> pose(IK+关节目标)", e))
                {
                    last_err = e;
                    continue;  // 这一组规划不出来，试下一组
                }
                if (verifyAchieved(target_pose, e))
                {
                    return true;
                }
                last_err = e;  // 到位校验没过，试下一组
            }
            err = "位姿目标 " + std::to_string(candidates.size()) +
                  " 组 IK 解都失败，最后一条：" + last_err;
            RCLCPP_ERROR(node_->get_logger(), "%s", err.c_str());
            return false;
        }

        // 笛卡尔直线：抓取的竖直进给必须走这条路径。
        // 否则 MoveIt 可能选到另一组 IK 解，产生关节空间大跳变，
        // 表现为"手臂甩一下才到位"，还会把已经对好的方块蹭飞。
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);
        moveit_msgs::msg::RobotTrajectory trajectory;

        const double jump_threshold = 0.0;
        const double eef_step = 0.005;  // 5mm

        double fraction = arm->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.99)
        {
            err = "笛卡尔路径只算到 " + std::to_string(fraction * 100.0) +
                  "%（目标可能不可达或会撞障碍）";
            RCLCPP_ERROR(node_->get_logger(), "%s", err.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "笛卡尔路径完成度 %.1f%%，开始执行", fraction * 100.0);
        arm->execute(trajectory);
        return verifyAchieved(target_pose, err);
    }

    // ================= action server =================

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const ArmTask::Goal> goal)
    {
        if (busy_.load())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "已有 arm_task 在执行，拒绝新目标");
            return rclcpp_action::GoalResponse::REJECT;
        }
        const std::string t = goal->task_type;
        if (t != ArmTask::Goal::NAMED && t != ArmTask::Goal::POSE &&
            t != ArmTask::Goal::JOINT)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "未知 task_type: %s，拒绝", t.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(node_->get_logger(), "接受 arm_task：type=%s", t.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleArmTask>)
    {
        // 规划/执行是阻塞调用，无法中断；这里只接受取消请求，
        // 执行线程会在下一个检查点看到 canceling 并放弃发布结果。
        RCLCPP_WARN(node_->get_logger(),
                    "收到 arm_task 取消请求（当前动作不可中断，将在结束后生效）");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleArmTask> goal_handle)
    {
        std::thread{std::bind(&commander::executeGoal, this, _1),
                    goal_handle}.detach();
    }

    void executeGoal(const std::shared_ptr<GoalHandleArmTask> goal_handle)
    {
        busy_.store(true);
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ArmTask::Feedback>();
        auto result = std::make_shared<ArmTask::Result>();

        feedback->phase = "planning";
        feedback->progress = 0.0;
        goal_handle->publish_feedback(feedback);

        if (goal->timeout > 0.0)
        {
            // 规划只花掉 timeout 的一部分，给执行/传输留余量（避免与调用方的
            // 超时判定在同一时刻撞车，见构造函数里的说明）
            const double budget = std::min(goal->timeout, plan_time_cap_);
            arm->setPlanningTime(budget);
            RCLCPP_INFO(node_->get_logger(),
                        "本次规划预算 %.1fs（goal.timeout=%.1fs，上限 %.1fs）",
                        budget, goal->timeout, plan_time_cap_);
        }

        std::string err;
        bool ok = false;
        try
        {
            if (goal->task_type == ArmTask::Goal::NAMED)
            {
                ok = goNamed(goal->group_name, goal->named_target, err);
            }
            else if (goal->task_type == ArmTask::Goal::POSE)
            {
                const auto& p = goal->target_pose;
                tf2::Quaternion q(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                ok = goPose(p.position.x, p.position.y, p.position.z,
                            roll, pitch, yaw, goal->use_cartesian, err);
            }
            else if (goal->task_type == ArmTask::Goal::JOINT)
            {
                ok = goJoint(goal->joint_values, err);
            }
            else
            {
                err = "未知 task_type: " + goal->task_type;
            }
        }
        catch (const std::exception& e)
        {
            ok = false;
            err = std::string("执行异常: ") + e.what();
        }

        feedback->phase = "done";
        feedback->progress = 1.0;
        goal_handle->publish_feedback(feedback);

        result->success = ok;
        result->message = ok ? "ok" : err;

        if (goal_handle->is_canceling())
        {
            goal_handle->canceled(result);
        }
        else if (ok)
        {
            goal_handle->succeed(result);
        }
        else
        {
            goal_handle->abort(result);
        }
        busy_.store(false);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MoveGroupInterface> arm;
    std::shared_ptr<MoveGroupInterface> gripper;

    rclcpp::Subscription<Bool>::SharedPtr gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCommand>::SharedPtr pose_cmd_sub_;

    rclcpp::CallbackGroup::SharedPtr action_cb_group_;
    rclcpp_action::Server<ArmTask>::SharedPtr action_server_;
    std::atomic<bool> busy_{false};
    double plan_time_cap_ = 20.0;   // 规划预算上限，见构造函数注释

    // ---------- 话题回调（手动调试用，忽略成功与否） ----------
    void openGripper()  { std::string e; goNamed("gripper", "gripper_open", e); }
    void closeGripper() { std::string e; goNamed("gripper", "gripper_closed", e); }

    void gripperCallback(const Bool& msg)
    {
        if (msg.data) { openGripper(); } else { closeGripper(); }
    }

    void jointCmdCallback(const FloatArray& msg)
    {
        if (msg.data.size() == 6)
        {
            std::string e;
            goJoint(msg.data, e);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "joint_cmd 需要 6 个值，收到 %zu 个", msg.data.size());
        }
    }

    void poseCmdCallback(const PoseCommand& msg)
    {
        std::string e;
        goPose(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw,
               msg.use_cartisian, e);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // MoveGroupInterface 内部会构造 RobotModelLoader，需要从本节点的参数里读
    // robot_description / robot_description_semantic / kinematics 等一堆 MoveIt 参数。
    // 这些参数由 launch 文件用 MoveItConfigsBuilder().to_dict() 传进来，
    // 但都是"未声明"参数，必须打开自动声明，否则读不到、构造机器人模型直接失败：
    //   "Could not find parameter robot_description ... Unable to construct robot model"
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("my_robot_commander", options);

    // 必须先起执行器再构造 commander：MoveGroupInterface 构造时要查 TF 和
    // 关节状态，节点不 spin 会卡在这里。
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto commander_node = std::make_shared<commander>(node);

    spinner.join();

    rclcpp::shutdown();
    return 0;
}
